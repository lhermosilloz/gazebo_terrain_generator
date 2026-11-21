"""
Road smoothing for DEM heightmaps.

The heightmap comes from Mapbox terrain-DEM (SRTM-class data, ~30 m) upsampled to the
heightmap size, so roads come out bumpy along their length and tilted across. This
module conditions the heightmap along road corridors before the PNG is written:

1. Road centerlines (GeoJSON lines carrying Mapbox streets-v8 ``class``/``type``/
   ``structure`` properties) are mapped to heightmap pixel space. Bridges, tunnels,
   ferries, rail, paths and other non-drivable classes are skipped, and each class gets
   a typical paved width.
2. The lines are noded into a network and sampled about once per heightmap pixel. The
   terrain height is read at every sample and the profile is smoothed on the network by
   solving   (I + lam * D^T D) h = z,   where D measures how far each sample sits from
   the mean of its neighbours (a discrete curvature). This keeps grades (a straight
   ramp has zero curvature), removes bumps shorter than ``smoothing_length``, and makes
   roads that meet at a junction share one height there.
3. Each corridor is flattened across to the smoothed centerline height and feathered
   back into the original terrain over ``shoulder_width``. Pixels farther than
   half-width + shoulder from every road are left untouched. On coarse heightmaps the
   flat part is kept at least about one cell wide, since Gazebo interpolates between
   samples and a narrower road could not be flat.

The default smoothing length (80 m) keeps the vertical curves of roads designed for
about 60 km/h and up (full undulations of 150 m or more) and removes the shorter bumps,
which at ~30 m DEM resolution are mostly noise, trees and buildings. Mountain roads on
steep slopes benefit from 120-200 m.

Ideas borrowed from ekumenlabs/terminus (Apache-2.0, Python 2, unmaintained since 2017):
a road-class whitelist taken from the OSM ``highway`` tag and clipping roads to the
area bounds. Terminus itself assigns z = 0 to every road and has no elevation
handling, so nothing else could be reused.
"""

import math

import numpy as np
import shapely
from scipy import sparse
from scipy.ndimage import map_coordinates
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree
from shapely.geometry import LineString, MultiLineString, GeometryCollection, box


DEFAULT_SMOOTHING_LENGTH_M = 80.0
DEFAULT_SHOULDER_WIDTH_M = 4.0

# Typical paved width (m) of one mapped way per Mapbox streets-v8 road class.
# Motorways/trunks are usually mapped as one way per carriageway in OSM.
# Classes not listed here (path, ferry, rail, aerialway, golf, construction and the
# point classes such as turning_circle) are not smoothed.
ROAD_CLASS_WIDTHS_M = {
    'motorway': 14.0,
    'trunk': 12.0,
    'primary': 11.0,
    'secondary': 9.0,
    'tertiary': 8.0,
    'motorway_link': 7.0,
    'trunk_link': 7.0,
    'primary_link': 7.0,
    'secondary_link': 6.5,
    'tertiary_link': 6.5,
    'street': 7.0,
    'street_limited': 5.0,
    'pedestrian': 5.0,
    'service': 4.5,
    'track': 3.5,
}

# Finer widths for some service-road subtypes (matched as substrings of ``type``).
ROAD_TYPE_WIDTHS_M = {
    'driveway': 3.5,
    'alley': 4.0,
    'parking_aisle': 6.0,
}

# Elevated or underground ways do not follow the terrain surface.
SKIP_STRUCTURES = {'bridge', 'tunnel'}


def road_width_m(properties):
    """Return the paved width in metres for a road feature, or None to skip it."""
    props = properties or {}
    if str(props.get('structure', 'none')).lower() in SKIP_STRUCTURES:
        return None
    width = ROAD_CLASS_WIDTHS_M.get(props.get('class'))
    if width is None:
        return None
    road_type = str(props.get('type') or '')
    for key, type_width in ROAD_TYPE_WIDTHS_M.items():
        if key in road_type:
            return type_width
    return width


def _iter_lines(geom):
    """Yield the LineStrings contained in any shapely geometry."""
    if geom is None or geom.is_empty:
        return
    if isinstance(geom, LineString):
        yield geom
    elif isinstance(geom, (MultiLineString, GeometryCollection)):
        for part in geom.geoms:
            yield from _iter_lines(part)


def lonlat_to_heightmap_px(lon, lat, bounds, shape):
    """
    Map lon/lat to fractional heightmap pixel coordinates (column, row).

    Uses the same linear lon/lat mapping as the launch-point and building helpers, scaled
    by (size - 1) because Gazebo places heightmap sample i at i / (size - 1) of the terrain.

    Args:
        lon, lat: scalars or arrays in degrees.
        bounds (dict): {"southwest": (lat, lon), "northeast": (lat, lon), ...}.
        shape (tuple): heightmap (rows, cols).
    """
    lat_min, lon_min = bounds['southwest']
    lat_max, lon_max = bounds['northeast']
    rows, cols = shape
    col = (np.asarray(lon, dtype=np.float64) - lon_min) / (lon_max - lon_min) * (cols - 1)
    row = (lat_max - np.asarray(lat, dtype=np.float64)) / (lat_max - lat_min) * (rows - 1)
    return col, row


def roads_from_geojson(geojson, bounds, shape):
    """
    Convert a GeoJSON FeatureCollection of road lines to heightmap pixel space.

    Returns:
        list of (coords, width_m) where coords is a (k, 2) array of (column, row).
    """
    roads = []
    for feature in (geojson or {}).get('features', []):
        geometry = feature.get('geometry') or {}
        width = road_width_m(feature.get('properties'))
        if width is None:
            continue
        if geometry.get('type') == 'LineString':
            parts = [geometry.get('coordinates') or []]
        elif geometry.get('type') == 'MultiLineString':
            parts = geometry.get('coordinates') or []
        else:
            continue
        for part in parts:
            coords = np.asarray(part, dtype=np.float64)
            if coords.ndim != 2 or len(coords) < 2:
                continue
            col, row = lonlat_to_heightmap_px(coords[:, 0], coords[:, 1], bounds, shape)
            roads.append((np.column_stack([col, row]), width))
    return roads


def _smoothstep_falloff(dist, half_width, shoulder):
    """1 inside the road, smooth (C1) ramp to 0 across the shoulder, 0 beyond."""
    u = np.clip((dist - half_width) / shoulder, 0.0, 1.0)
    return 1.0 - u * u * (3.0 - 2.0 * u)


def _build_network(lines, ds, link_tol):
    """
    Sample noded road segments every <= ds metres and connect the samples into a graph.

    Returns:
        nodes (n, 2) metric coordinates, edges (m, 2) node index pairs, and for each
        segment the array of its node indices (in order).
    """
    nodes = []
    edges = []
    seg_nodes = []
    endpoint_ids = {}
    endpoint_segs = {}
    node_seg = []  # segment index for interior nodes, -1 for (possibly shared) endpoints
    n_nodes = 0

    def endpoint(xy, seg_idx):
        nonlocal n_nodes
        key = (round(float(xy[0]), 3), round(float(xy[1]), 3))
        node_id = endpoint_ids.get(key)
        if node_id is None:
            node_id = n_nodes
            endpoint_ids[key] = node_id
            nodes.append(np.asarray(xy, dtype=np.float64).reshape(1, 2))
            node_seg.append(np.array([-1]))
            n_nodes += 1
        endpoint_segs.setdefault(node_id, set()).add(seg_idx)
        return node_id

    for seg_idx, seg in enumerate(lines):
        n = max(1, int(math.ceil(seg.length / ds)))
        pts = shapely.get_coordinates(shapely.line_interpolate_point(seg, np.linspace(0.0, seg.length, n + 1)))
        ids = np.empty(n + 1, dtype=np.int64)
        ids[0] = endpoint(pts[0], seg_idx)
        if n > 1:
            ids[1:n] = np.arange(n_nodes, n_nodes + n - 1)
            nodes.append(pts[1:n])
            node_seg.append(np.full(n - 1, seg_idx))
            n_nodes += n - 1
        ids[n] = endpoint(pts[n], seg_idx)
        seg_nodes.append(ids)
        edges.append(np.column_stack([ids[:-1], ids[1:]]))

    nodes = np.vstack(nodes)
    node_seg = np.concatenate(node_seg)
    edges = np.vstack(edges)

    # Link segment ends to nearby nodes of other segments. Vector-tile clipping and
    # simplification leave sub-metre gaps at tile seams and some junctions.
    end_ids = np.fromiter(endpoint_segs.keys(), dtype=np.int64)
    tree = cKDTree(nodes)
    extra = []
    for end_id, near in zip(end_ids, tree.query_ball_point(nodes[end_ids], r=link_tol)):
        segs = endpoint_segs[end_id]
        for j in near:
            if j == end_id:
                continue
            if node_seg[j] >= 0 and node_seg[j] in segs:
                continue  # interior node of a segment this endpoint is already on
            if node_seg[j] < 0 and endpoint_segs[j] & segs:
                continue  # other end of a (very short) segment this endpoint is already on
            extra.append((end_id, j))
    if extra:
        edges = np.vstack([edges, np.asarray(extra, dtype=np.int64)])
    edges = edges[edges[:, 0] != edges[:, 1]]
    return nodes, edges, seg_nodes


def _smooth_profile(z, edges, n_nodes, lam):
    """Solve (I + lam * D^T D) h = z on the road graph."""
    i, j = edges[:, 0], edges[:, 1]
    adj = sparse.coo_matrix((np.ones(len(i) * 2), (np.concatenate([i, j]), np.concatenate([j, i]))),
                            shape=(n_nodes, n_nodes)).tocsr()
    adj.data[:] = 1.0  # collapse duplicate edges
    deg = np.asarray(adj.sum(axis=1)).ravel()
    rows = np.flatnonzero(deg >= 2)  # free ends get no curvature penalty (keeps grades to the end)
    if len(rows) == 0:
        return z.copy()
    inv_deg = sparse.diags(1.0 / np.maximum(deg, 1.0))
    d_op = (sparse.identity(n_nodes, format='csr') - inv_deg @ adj)[rows]
    system = (sparse.identity(n_nodes, format='csc') + lam * (d_op.T @ d_op)).tocsc()
    return spsolve(system, z)


def _chunks(seg_nodes, seg_width, min_half_width, shoulder, ds):
    """
    Split every noded segment into short runs of nodes for rasterisation.

    Yields (segment index, own_lo, own_hi, node ids). The node ids are the chunk's own
    nodes extended on both sides by enough neighbouring nodes (from the same segment, or
    through an end shared with exactly one other segment) that every pixel within reach
    finds its true closest point; edges own_lo .. own_hi - 1 of that polyline are the
    chunk's own part.
    """
    ends = {}
    for s, ids in enumerate(seg_nodes):
        ends.setdefault(int(ids[0]), []).append((s, 0))
        ends.setdefault(int(ids[-1]), []).append((s, -1))

    def beyond(s, node, count):
        """Up to `count` nodes moving away from `node` along the one other segment ending there."""
        others = [(s2, pos) for s2, pos in ends.get(int(node), []) if s2 != s]
        if len(others) != 1 or count <= 0:
            return np.empty(0, dtype=np.int64)
        s2, pos = others[0]
        ids2 = seg_nodes[s2]
        return ids2[1:count + 1] if pos == 0 else ids2[-2::-1][:count]

    for s, ids in enumerate(seg_nodes):
        reach = max(seg_width[s] / 2.0, min_half_width) + shoulder
        overlap = int(math.ceil(reach / ds)) + 1
        chunk = max(8, int(math.ceil(2.5 * reach / ds)))
        last = len(ids) - 1
        for start in range(0, last, chunk):
            stop = min(start + chunk, last)  # own nodes are ids[start..stop]
            before = ids[max(0, start - overlap):start]
            if start < overlap:
                before = np.concatenate([beyond(s, ids[0], overlap - start)[::-1], before])
            after = ids[stop + 1:stop + 1 + overlap]
            if stop + overlap > last:
                after = np.concatenate([after, beyond(s, ids[-1], stop + overlap - last)])
            cid = np.concatenate([before, ids[start:stop + 1], after]).astype(np.int64)
            yield s, len(before), len(before) + stop - start, cid


def smooth_roads(height, roads, size_x, size_y,
                 smoothing_length=DEFAULT_SMOOTHING_LENGTH_M,
                 shoulder_width=DEFAULT_SHOULDER_WIDTH_M):
    """
    Flatten and smooth road corridors in a heightmap.

    Args:
        height (np.ndarray): 2-D heightmap (any numeric dtype, any vertical unit).
        roads (list): (coords, width_m) pairs; coords is a (k, 2) array of fractional
            (column, row) heightmap pixels, as returned by roads_from_geojson().
        size_x, size_y (float): terrain extent in metres along columns / rows.
        smoothing_length (float): bumps along a road shorter than this wavelength (m)
            are removed; longer grade changes are kept.
        shoulder_width (float): width (m) of the blend from the road edge back to the
            original terrain. Rounded up to one heightmap pixel.

    Returns:
        (np.ndarray float64, dict): the smoothed heightmap and statistics.
    """
    h = np.asarray(height, dtype=np.float64)
    rows_n, cols_n = h.shape
    sx = float(size_x) / (cols_n - 1)
    sy = float(size_y) / (rows_n - 1)
    pix = min(sx, sy)
    stats = {'roads': 0, 'segments': 0, 'length_km': 0.0, 'pixels_changed': 0,
             'fraction_changed': 0.0, 'max_change': 0.0}

    smoothing_length = max(float(smoothing_length), 2.0 * pix)
    shoulder = max(float(shoulder_width), max(sx, sy))

    # --- 1. Road lines in a metric frame (x east along columns, y south along rows) ---
    frame = box(0.0, 0.0, (cols_n - 1) * sx, (rows_n - 1) * sy)
    lines, widths = [], []
    for coords, width in roads:
        metric = np.column_stack([coords[:, 0] * sx, coords[:, 1] * sy])
        if len(metric) < 2 or not np.all(np.isfinite(metric)):
            continue
        for part in _iter_lines(LineString(metric).intersection(frame)):
            if part.length > 1e-6:
                lines.append(part)
                widths.append(float(width))
    if not lines:
        return h.copy(), stats
    widths = np.asarray(widths)
    stats['roads'] = len(lines)

    # --- 2. Node the network (split at crossings, merge duplicates) ---
    segments = [s for s in _iter_lines(shapely.unary_union(lines)) if s.length > 1e-6]
    if not segments:
        return h.copy(), stats
    stats['segments'] = len(segments)
    stats['length_km'] = round(sum(s.length for s in segments) / 1000.0, 3)

    # Width of each noded segment: the widest source road it lies on.
    tree = shapely.STRtree(lines)
    mids = shapely.line_interpolate_point(segments, 0.5, normalized=True)
    seg_idx, line_idx = tree.query(mids, predicate='dwithin', distance=0.05 * pix + 1e-6)
    seg_width = np.zeros(len(segments))
    np.maximum.at(seg_width, seg_idx, widths[line_idx])
    missing = np.flatnonzero(seg_width == 0)
    if len(missing):
        nearest = tree.nearest(mids[missing])
        seg_width[missing] = widths[nearest]

    # --- 3. Sample the network and smooth the height profile ---
    ds = float(np.clip(pix, smoothing_length / 200.0, smoothing_length / 8.0))
    nodes, edges, seg_nodes = _build_network(segments, ds, link_tol=max(1.0, 0.5 * ds))
    z = map_coordinates(h, [nodes[:, 1] / sy, nodes[:, 0] / sx], order=1, mode='nearest')
    k_cut = 2.0 * math.pi / smoothing_length
    lam = 4.0 / (k_cut * ds) ** 4  # chain response 1 / (1 + lam (1 - cos(k ds))^2) = 1/2 at k_cut
    zs = _smooth_profile(z, edges, len(nodes), lam)

    # --- 4. Rasterise corridors chunk by chunk with exact point-to-segment distances ---
    # Each pixel near a chunk gets the lateral distance to it and the smoothed height at
    # its projection. Overlapping corridors (junctions, hairpins, parallel roads) are
    # blended continuously: weight = max over chunks, target = average weighted by w^4,
    # so a pixel inside one road's core keeps that road's height.
    w_max = np.zeros(h.shape, dtype=np.float32)
    p_sum = np.zeros(h.shape, dtype=np.float32)
    pt_sum = np.zeros(h.shape, dtype=np.float32)
    min_half_width = 0.75 * math.hypot(sx, sy)  # keep at least the cells a road crosses flat
    for s, own_lo, own_hi, cid in _chunks(seg_nodes, seg_width, min_half_width, shoulder, ds):
        half_width = max(seg_width[s] / 2.0, min_half_width)
        reach = half_width + shoulder
        p = nodes[cid]
        ph = zs[cid]
        own = p[own_lo:own_hi + 1]
        c0 = max(int(math.floor((own[:, 0].min() - reach) / sx)), 0)
        c1 = min(int(math.ceil((own[:, 0].max() + reach) / sx)) + 1, cols_n)
        r0 = max(int(math.floor((own[:, 1].min() - reach) / sy)), 0)
        r1 = min(int(math.ceil((own[:, 1].max() + reach) / sy)) + 1, rows_n)
        if c0 >= c1 or r0 >= r1:
            continue
        gx = (np.arange(c0, c1) * sx)[None, :, None]
        gy = (np.arange(r0, r1) * sy)[:, None, None]
        a = p[:-1]
        ab = p[1:] - a
        len2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
        # (rows, cols, segments) projection parameter, clamped to each segment
        t = np.clip(((gx - a[:, 0]) * ab[:, 0] + (gy - a[:, 1]) * ab[:, 1]) / len2, 0.0, 1.0)
        d2 = (gx - a[:, 0] - t * ab[:, 0]) ** 2 + (gy - a[:, 1] - t * ab[:, 1]) ** 2
        k = np.argmin(d2, axis=2)[..., None]
        d = np.sqrt(np.take_along_axis(d2, k, axis=2)[..., 0])
        tk = np.take_along_axis(t, k, axis=2)[..., 0]
        k = k[..., 0]
        target = ph[k] + tk * (ph[k + 1] - ph[k])
        w = _smoothstep_falloff(d, half_width, shoulder)
        # The polyline runs past the chunk into its neighbours; a pixel belongs to the
        # chunk whose own part holds its closest point, so joints leave no seams.
        w[(k < own_lo) | (k >= own_hi)] = 0.0
        # Beyond a real end (dead end, junction) pixels project onto the end node. Such a
        # cap still counts for coverage but must not pull the height of a pixel that
        # projects inside another road.
        capped = ((k == 0) & (tk <= 0.0)) | ((k == len(cid) - 2) & (tk >= 1.0))
        pw = w ** 4 * np.where(capped, 1e-3, 1.0)
        win = (slice(r0, r1), slice(c0, c1))
        np.maximum(w_max[win], w, out=w_max[win], casting='unsafe')
        p_sum[win] += pw
        pt_sum[win] += pw * target

    # --- 5. Blend: flat smoothed road inside, feathered shoulder, untouched beyond ---
    out = h.copy()
    m = p_sum > 0
    target = pt_sum[m].astype(np.float64) / p_sum[m]
    out[m] = h[m] + w_max[m] * (target - h[m])

    delta = np.abs(out[m] - h[m])
    stats['pixels_changed'] = int(np.count_nonzero(delta > 0))
    stats['fraction_changed'] = round(stats['pixels_changed'] / h.size, 4)
    stats['max_change'] = float(delta.max()) if delta.size else 0.0
    return out, stats


def smooth_heightmap_roads(heightmap, geojson, bounds, size_x, size_y,
                           smoothing_length=DEFAULT_SMOOTHING_LENGTH_M,
                           shoulder_width=DEFAULT_SHOULDER_WIDTH_M):
    """
    Apply smooth_roads() to an integer heightmap (uint8 or uint16) and keep its dtype.

    Args:
        heightmap (np.ndarray): normalized uint8/uint16 heightmap.
        geojson (dict): GeoJSON FeatureCollection of road lines (lon/lat).
        bounds (dict): {"southwest": (lat, lon), "northeast": (lat, lon), ...}.
        size_x, size_y (float): terrain extent in metres.

    Returns:
        (np.ndarray, dict): smoothed heightmap (same dtype/shape) and statistics.
    """
    roads = roads_from_geojson(geojson, bounds, heightmap.shape)
    smoothed, stats = smooth_roads(heightmap, roads, size_x, size_y,
                                   smoothing_length=smoothing_length, shoulder_width=shoulder_width)
    info = np.iinfo(heightmap.dtype)
    result = np.clip(np.rint(smoothed), info.min, info.max).astype(heightmap.dtype)
    return result, stats
