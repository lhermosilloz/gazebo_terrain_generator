"""
Procedural forest from the satellite texture's vegetation mask.

Pipeline (all pure Python / NumPy / OpenCV, no Blender):

1. Vegetation mask from mesh/aerial.png: a colour test that also works in
   terrain shadow (normalised green-red difference plus excess green), then
   tree canopy is told apart from lawns and fields by texture: lawns are smooth
   and wide, canopies are rough or dark. Morphological cleanup and a minimum
   patch size follow.
2. Exclusions: building footprints (buildings.geojson), roads and water (the
   cached Mapbox streets-v8 vector tiles in building_tiles/), the area outside
   the drawn polygon, and a clear zone around every spawn point.
3. Placement: Poisson-disk style dart throwing inside the mask at the requested
   density, plus one tree in every isolated canopy patch that got none. Above
   max_trees, 80% of the budget keeps full density nearest the spawn point and
   the rest is spread thinly over the remaining canopy. Crown size follows the
   local canopy width; z comes from the heightmap so trunks start below the ground.
4. Models: low-poly procedural trees (broadleaf and conifer variants) merged
   into mesh/trees.dae, one sub-mesh per colour, and a trunks-only collision
   mesh mesh/trees_collision.dae.

It also writes mesh/vegetation_mask.png (255 = tree canopy, 128 = other
vegetation, same pixel grid as aerial.png) and trees.csv (numeric columns, one
row per tree) so the result can be reused elsewhere, e.g. with Blender's
"Import CSV" geometry node.

World frame: x east, y north, origin at the launch point, the same frame the
world template uses. The terrain is centred at (pose_x, pose_y), and the aerial
texture starts at the terrain's north-west corner and covers
texture_size x texture_size metres (see generate_ortho and the <texture><size>
in the world templates).
"""

import csv
import glob
import json
import math
import os
import time

import cv2
import numpy as np


class ForestGenerator:

    # Defaults shared by the server and the world generator (the web UI has the same values)
    DEFAULT_DENSITY = 150.0      # trees per hectare of detected canopy
    DEFAULT_MAX_TREES = 20000

    # ---------------- vegetation mask ----------------
    # Images finer than this are downsampled before processing (metres/pixel)
    PROC_MIN_MPP = 0.3
    # Never process a grid larger than this (pixels per side)
    PROC_MAX_DIM = 8192
    # Pixels darker than this (luminance 0-255) use the shadow colour test
    DARK_Y = 45.0
    # Pixels darker than this are too noisy to classify (deep shadow / black)
    MIN_Y = 10.0
    # Normalised green-red difference (G-R)/(G+R): sunlit and shadowed vegetation
    NGRDI_MIN = 0.03
    NGRDI_DARK_MIN = 0.08
    # Excess green (2G-R-B)/(R+G+B) and saturation for sunlit vegetation
    EXG_MIN = 0.04
    SAT_MIN = 0.15
    # Sunlit pixels brighter than this are not vegetation (roofs, concrete)
    MAX_Y = 200.0
    # Texture: mean |Laplacian| / brightness over TEXTURE_WINDOW_M, then the median
    # over LAWN_MIN_WIDTH_M. Measured on the Apple Park sample (medians): lawn 0.10,
    # sports turf 0.16, large tree crowns 0.21, forest and shrubs 0.3+
    TEXTURE_WINDOW_M = 3.0
    SMOOTH_TEXTURE = 0.185
    TEXTURE_SCALE = 500.0  # texture is stored as uint8 (value * scale) for the median filter
    # Smooth vegetation this wide (or wider) is lawn / field, not canopy
    LAWN_MIN_WIDTH_M = 6.0
    CLOSE_M = 2.0
    OPEN_M = 1.5
    MIN_BLOB_M2 = 12.0
    # Stay this far inside the drawn polygon / terrain edge
    EDGE_MARGIN_M = 2.0

    # ---------------- exclusions ----------------
    BUILDING_MARGIN_M = 1.0
    ROAD_MARGIN_M = 1.0
    # Full carriageway width per Mapbox streets-v8 road class (metres)
    ROAD_WIDTH_M = {
        'motorway': 14.0, 'trunk': 12.0, 'primary': 10.0, 'secondary': 9.0,
        'tertiary': 8.0, 'street': 7.0, 'street_limited': 6.0, 'service': 4.0,
        'pedestrian': 4.0, 'construction': 6.0, 'track': 3.0, 'path': 1.5,
        'major_rail': 4.0, 'minor_rail': 3.0, 'service_rail': 3.0,
        'golf': 2.0,
    }
    ROAD_DEFAULT_WIDTH_M = 5.0
    ROAD_SKIP_CLASSES = {'ferry', 'aerialway'}
    WATERWAY_WIDTH_M = {'river': 10.0, 'canal': 8.0, 'stream': 2.5, 'drain': 2.0, 'ditch': 1.5}
    # Mapped water areas are shrunk by this much before excluding them (banks keep their trees)
    WATER_BANK_M = 3.0
    # No trunk within this radius of the spawn point; crowns stay this far clear
    SPAWN_CLEARANCE_M = 12.0
    SPAWN_CROWN_CLEARANCE_M = 4.0

    # ---------------- placement ----------------
    MIN_SPACING_M = 2.5
    MAX_SPACING_M = 40.0
    CANDIDATE_FACTOR = 4
    # Share of max_trees placed at full density nearest the spawn point when the cap is hit
    NEAR_SPAWN_SHARE = 0.8
    CROWN_MIN_M = 1.2
    CROWN_MAX_M = 5.5
    TREE_MAX_HEIGHT_M = 35.0
    # Trunks extend this far below the lowest ground point under the trunk
    TRUNK_SINK_M = 0.6

    # ---------------- appearance ----------------
    TRUNK_SIDES = 5
    TEMPLATES_PER_KIND = 3
    TRUNK_COLOR = (0.33, 0.23, 0.15)
    BROADLEAF_COLORS = [(0.16, 0.33, 0.10), (0.24, 0.42, 0.14), (0.34, 0.50, 0.20)]
    CONIFER_COLORS = [(0.08, 0.22, 0.11), (0.13, 0.30, 0.15)]

    def __init__(self, aerial_path, heightmap, heightmap_z_resolution,
                 size_x, size_y, size_z, pose_x, pose_y, pose_z,
                 output_dir, origin_lat=None, origin_lon=None, texture_size=None,
                 buildings_geojson=None, vector_tiles_dir=None,
                 density=DEFAULT_DENSITY, max_trees=DEFAULT_MAX_TREES, spawn_xy=(0.0, 0.0), seed=1234,
                 progress=None, extra_spawn_xy=()):
        """
        Args:
            aerial_path (str): mesh/aerial.png (square, padded with gray 128).
            heightmap: PIL image or 2D array of the heightmap written to mesh/height_map.png.
            heightmap_z_resolution (int): 65535 (16-bit) or 255 (8-bit).
            size_x, size_y, size_z (float): terrain size in metres (as in the world file).
            pose_x, pose_y, pose_z (float): terrain offset from the launch point (as in the world file).
            output_dir (str): world directory; writes mesh/trees*.dae, mesh/vegetation_mask.png, trees.csv.
            origin_lat, origin_lon (float): centre of the bounds, the origin of the buildings' AEQD frame.
                Needed only for the vector exclusions.
            texture_size (float|None): metres covered by aerial.png; defaults to max(size_x, size_y).
            buildings_geojson (str|None): footprints to keep trees out of.
            vector_tiles_dir (str|None): cached streets-v8 tiles ({z}/{x}/{y}.json) for roads and water.
            density (float): trees per hectare of detected tree canopy.
            max_trees (int): hard cap on the number of trees.
            spawn_xy (tuple): world xy kept clear of trunks.
            seed (int): random seed, so the same inputs give the same forest.
            progress (callable|None): receives human-readable status lines.
            extra_spawn_xy (iterable): world xy of additional spawn points (multi-spawn), kept
                clear like spawn_xy. The tree cap still keeps full density around spawn_xy only.
        """
        self.aerial_path = aerial_path
        self.heightmap = np.asarray(heightmap, dtype=np.float32)
        self.heightmap_z_resolution = float(heightmap_z_resolution)
        self.size_x, self.size_y, self.size_z = float(size_x), float(size_y), float(size_z)
        self.pose_x, self.pose_y, self.pose_z = float(pose_x), float(pose_y), float(pose_z)
        self.texture_size = float(texture_size) if texture_size else max(self.size_x, self.size_y)
        self.output_dir = output_dir
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.buildings_geojson = buildings_geojson
        self.vector_tiles_dir = vector_tiles_dir
        self.density = max(0.0, float(density))
        self.max_trees = max(0, int(max_trees))
        self.spawn_xy = (float(spawn_xy[0]), float(spawn_xy[1]))
        # Every point that gets the spawn clear zone (the primary first)
        self.clear_xy = [self.spawn_xy] + [(float(x), float(y)) for x, y in extra_spawn_xy]
        self.rng = np.random.default_rng(seed)
        self._progress = progress
        # North-west corner of the terrain = origin of the texture
        self.x0 = self.pose_x - self.size_x / 2.0
        self.y1 = self.pose_y + self.size_y / 2.0
        self.stats = {}

    # ------------------------------------------------------------------ utils
    def log(self, msg):
        if self._progress:
            self._progress(msg)
        else:
            print(msg)

    def _kernel(self, diameter_m, mpp, minimum=1):
        """Elliptical structuring element of the given diameter in metres, or None if < 2 px."""
        d = int(round(diameter_m / mpp))
        d = max(d, minimum)
        if d < 2:
            return None
        if d % 2 == 0:
            d += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (d, d))

    def _morph(self, mask, op, diameter_m, mpp, minimum=1):
        k = self._kernel(diameter_m, mpp, minimum)
        if k is None:
            return mask
        return cv2.morphologyEx(mask, op, k)

    def world_to_px(self, x, y, mpp):
        """World metres -> continuous pixel coordinates (u right/east, v down/south)."""
        return (np.asarray(x) - self.x0) / mpp, (self.y1 - np.asarray(y)) / mpp

    def px_to_world(self, u, v, mpp):
        return self.x0 + np.asarray(u) * mpp, self.y1 - np.asarray(v) * mpp

    def ground_z(self, x, y):
        """Bilinear heightmap lookup at world xy, in world z."""
        hm = self.heightmap
        n_rows, n_cols = hm.shape
        fx = (np.asarray(x, dtype=np.float64) - self.x0) / self.size_x * (n_cols - 1)
        fy = (self.y1 - np.asarray(y, dtype=np.float64)) / self.size_y * (n_rows - 1)
        fx = np.clip(fx, 0, n_cols - 1.000001)
        fy = np.clip(fy, 0, n_rows - 1.000001)
        ix, iy = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
        tx, ty = fx - ix, fy - iy
        h00 = hm[iy, ix]
        h10 = hm[iy, ix + 1]
        h01 = hm[iy + 1, ix]
        h11 = hm[iy + 1, ix + 1]
        h = (h00 * (1 - tx) * (1 - ty) + h10 * tx * (1 - ty) + h01 * (1 - tx) * ty + h11 * tx * ty)
        return h / self.heightmap_z_resolution * self.size_z + self.pose_z

    # ------------------------------------------------------- vegetation mask
    def classify(self, img, mpp):
        """
        Returns (veg, smooth, luma) for a BGR image: veg = vegetation colour,
        smooth = sunlit vegetation with little texture (lawn candidates),
        luma = blurred luminance (used later for tree type and shade).
        Processed in horizontal strips to bound memory on large textures.
        """
        h, w = img.shape[:2]
        sigma = max(0.5, 0.5 / mpp)
        win = max(3, int(round(self.TEXTURE_WINDOW_M / mpp)) | 1)
        margin = int(math.ceil(3 * sigma)) + win + 2
        veg = np.zeros((h, w), np.uint8)
        sunlit_all = np.zeros((h, w), np.uint8)
        texture_u8 = np.zeros((h, w), np.uint8)
        luma = np.zeros((h, w), np.uint8)
        step = 1024
        for y0 in range(0, h, step):
            y1 = min(h, y0 + step)
            a0, a1 = max(0, y0 - margin), min(h, y1 + margin)
            strip = img[a0:a1]
            f = cv2.GaussianBlur(strip, (0, 0), sigma).astype(np.float32)
            b, g, r = f[..., 0], f[..., 1], f[..., 2]
            lum = 0.299 * r + 0.587 * g + 0.114 * b
            ngrdi = (g - r) / (g + r + 1.0)
            exg = (2.0 * g - r - b) / (r + g + b + 1.0)
            mx = np.maximum(np.maximum(r, g), b)
            mn = np.minimum(np.minimum(r, g), b)
            sat = (mx - mn) / (mx + 1.0)
            bright = lum >= self.DARK_Y
            # Sunlit: clearly green, saturated, not bluer than green (pools, water, glass)
            sunlit = (bright & (ngrdi > self.NGRDI_MIN) & (exg > self.EXG_MIN) & (sat > self.SAT_MIN)
                      & (g >= b) & (lum < self.MAX_Y))
            # Shadow: skylight makes everything bluish, so only ask for green clearly above red
            shadow = (~bright) & (ngrdi > self.NGRDI_DARK_MIN) & (lum > self.MIN_Y)
            v = sunlit | shadow

            # Fine texture: mean |Laplacian| relative to brightness. Mowing stripes and
            # field markings are wide and barely register; leaves and crown shadows do.
            gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY).astype(np.float32)
            lap = np.abs(cv2.Laplacian(cv2.GaussianBlur(gray, (0, 0), 0.7), cv2.CV_32F, ksize=3))
            texture = cv2.blur(lap, (win, win)) / (cv2.blur(gray, (win, win)) + 10.0)

            o0, o1 = y0 - a0, y0 - a0 + (y1 - y0)
            veg[y0:y1] = v[o0:o1]
            sunlit_all[y0:y1] = sunlit[o0:o1]
            texture_u8[y0:y1] = np.clip(texture[o0:o1] * self.TEXTURE_SCALE, 0, 255).astype(np.uint8)
            luma[y0:y1] = np.clip(lum[o0:o1], 0, 255).astype(np.uint8)

        # Per pixel, turf with field lines and the edge of a crown overlap; the median
        # over a lawn-sized window does not (Apple Park: turf ~0.16, crowns >= 0.21).
        k = max(3, int(round(self.LAWN_MIN_WIDTH_M / mpp)) | 1)
        texture_u8 = cv2.medianBlur(texture_u8, min(k, 255))
        smooth = sunlit_all & (texture_u8 < int(self.SMOOTH_TEXTURE * self.TEXTURE_SCALE))
        return veg, smooth.astype(np.uint8), luma

    def valid_area(self, img, mpp):
        """Pixels inside the terrain and inside the drawn polygon (aerial is gray 128 outside)."""
        h, w = img.shape[:2]
        valid = np.zeros((h, w), np.uint8)
        valid[:min(h, int(self.size_y / mpp)), :min(w, int(self.size_x / mpp))] = 1
        gray = (np.abs(img.astype(np.int16) - 128).max(axis=2) <= 2).astype(np.uint8)
        # Only large gray areas are the fill; single gray pixels can be real asphalt
        gray = self._morph(gray, cv2.MORPH_OPEN, 3.0, mpp, minimum=3)
        valid &= (gray == 0)
        k = self._kernel(2 * self.EDGE_MARGIN_M, mpp, minimum=3)
        return cv2.erode(valid, k, borderValue=0)

    # ------------------------------------------------------------ exclusions
    def _lonlat_to_world(self):
        """Returns f(lon_array, lat_array) -> (x, y) in the world frame, matching GeoJSONToDAE."""
        from pyproj import CRS, Transformer
        local = CRS.from_proj4(
            f"+proj=aeqd +lat_0={self.origin_lat} +lon_0={self.origin_lon} "
            "+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs")
        tr = Transformer.from_crs("EPSG:4326", local, always_xy=True)

        def f(lon, lat):
            x, y = tr.transform(np.asarray(lon, dtype=np.float64), np.asarray(lat, dtype=np.float64))
            return x + self.pose_x, y + self.pose_y
        return f

    @staticmethod
    def _rings(geom_type, coords):
        """Yield (ring_coords, is_polygon) for GeoJSON-like geometry."""
        if geom_type == 'Polygon':
            for ring in coords[:1]:  # outer ring only; holes in footprints don't matter here
                yield ring, True
        elif geom_type == 'MultiPolygon':
            for poly in coords:
                for ring in poly[:1]:
                    yield ring, True
        elif geom_type == 'LineString':
            yield coords, False
        elif geom_type == 'MultiLineString':
            for line in coords:
                yield line, False

    def _draw(self, mask, to_world, ring, is_polygon, mpp, width_m=0.0):
        pts = np.asarray(ring, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] < 2:
            return
        x, y = to_world(pts[:, 0], pts[:, 1])
        u, v = self.world_to_px(x, y, mpp)
        shift = 3
        p = np.round(np.stack([u, v], axis=1) * (1 << shift)).astype(np.int32)
        if is_polygon and len(p) >= 3:
            cv2.fillPoly(mask, [p], 1, lineType=cv2.LINE_8, shift=shift)
        if width_m > 0:
            thickness = max(1, int(round(width_m / mpp)))
            cv2.polylines(mask, [p], is_polygon, 1, thickness=thickness, lineType=cv2.LINE_8, shift=shift)

    def exclusion_mask(self, shape, mpp):
        """Rasterise buildings, roads, water and the spawn zone into a mask (1 = no trees)."""
        excl = np.zeros(shape, np.uint8)
        counts = {'buildings': 0, 'roads': 0, 'water': 0}
        have_frame = self.origin_lat is not None and self.origin_lon is not None

        if have_frame and self.buildings_geojson and os.path.isfile(self.buildings_geojson):
            to_world = self._lonlat_to_world()
            bmask = np.zeros(shape, np.uint8)
            with open(self.buildings_geojson) as f:
                data = json.load(f)
            for feat in data.get('features', []):
                geom = feat.get('geometry') or {}
                for ring, is_poly in self._rings(geom.get('type'), geom.get('coordinates') or []):
                    self._draw(bmask, to_world, ring, is_poly, mpp)
                    counts['buildings'] += 1
            k = self._kernel(2 * self.BUILDING_MARGIN_M, mpp, minimum=3)
            excl |= cv2.dilate(bmask, k)

        if have_frame and self.vector_tiles_dir and os.path.isdir(self.vector_tiles_dir):
            to_world = self._lonlat_to_world()
            water_area = np.zeros(shape, np.uint8)
            for kind, ring, is_poly, width in self._vector_features():
                if kind == 'water_area':
                    self._draw(water_area, to_world, ring, is_poly, mpp)
                    counts['water'] += 1
                else:
                    self._draw(excl, to_world, ring, is_poly, mpp, width)
                    counts[kind] += 1
            # Mapped water areas of creeks and ponds usually include the banks, where trees
            # do grow: keep trees out of the water itself, not off the banks
            k = self._kernel(2 * self.WATER_BANK_M, mpp)
            excl |= cv2.erode(water_area, k) if k is not None else water_area

        # Spawn clear zone around every spawn point (and its helipad)
        for sx, sy in self.clear_xy:
            u, v = self.world_to_px(sx, sy, mpp)
            cv2.circle(excl, (int(round(float(u))), int(round(float(v)))),
                       int(math.ceil(self.SPAWN_CLEARANCE_M / mpp)), 1, -1)
        self.stats['excluded_features'] = counts
        return excl

    def _vector_features(self):
        """Roads, waterways and water areas from cached streets-v8 tiles ({z}/{x}/{y}.json)."""
        import mercantile
        from utils.building_downloader import BuildingDownloader
        from utils.road_smoother import road_width_m as smoothed_road_width_m
        converter = BuildingDownloader()
        for path in sorted(glob.glob(os.path.join(self.vector_tiles_dir, '*', '*', '*.json'))):
            try:
                z = int(os.path.basename(os.path.dirname(os.path.dirname(path))))
                x = int(os.path.basename(os.path.dirname(path)))
                y = int(os.path.splitext(os.path.basename(path))[0])
                with open(path) as f:
                    tile = json.load(f)
            except (ValueError, OSError, json.JSONDecodeError):
                continue
            bounds = mercantile.bounds(x, y, z)
            for layer_name in ('road', 'waterway', 'water'):
                layer = tile.get(layer_name)
                if not layer:
                    continue
                extent = layer.get('extent', 4096)
                for feat in layer.get('features', []):
                    props = feat.get('properties', {}) or {}
                    if layer_name == 'road':
                        cls = props.get('class', '')
                        if cls in self.ROAD_SKIP_CLASSES or props.get('structure') == 'tunnel':
                            continue
                        # Never narrower than the corridor road smoothing flattens for this
                        # road, so no trunk stands on a smoothed carriageway
                        width = max(self.ROAD_WIDTH_M.get(cls, self.ROAD_DEFAULT_WIDTH_M),
                                    smoothed_road_width_m(props) or 0.0) + 2 * self.ROAD_MARGIN_M
                        kind = 'roads'
                    elif layer_name == 'waterway':
                        width = self.WATERWAY_WIDTH_M.get(props.get('class', ''), 2.0)
                        kind = 'water'
                    else:
                        width = 0.0
                        kind = 'water_area'
                    gj = converter._feature_to_geojson(feat, bounds, extent)
                    if not gj:
                        continue
                    geom = gj['geometry']
                    for ring, is_poly in self._rings(geom['type'], geom['coordinates']):
                        yield kind, ring, is_poly, width

    # -------------------------------------------------------------- placement
    class _DartThrower:
        """Poisson-disk style sampling: accept a candidate only if no accepted point is
        closer than the radius. Grid cells of r/sqrt(2) hold at most one point each."""

        def __init__(self, r_px):
            self.r2 = r_px * r_px
            self.cell = r_px / math.sqrt(2.0)
            self.grid = {}
            self.u, self.v = [], []

        def free(self, uu, vv, r2=None):
            r2 = self.r2 if r2 is None else r2
            gx, gy = int(uu // self.cell), int(vv // self.cell)
            for dx in (-2, -1, 0, 1, 2):
                for dy in (-2, -1, 0, 1, 2):
                    j = self.grid.get((gx + dx, gy + dy))
                    if j is not None:
                        du, dv = self.u[j] - uu, self.v[j] - vv
                        if du * du + dv * dv < r2:
                            return False
            return True

        def add(self, uu, vv):
            self.grid.setdefault((int(uu // self.cell), int(vv // self.cell)), len(self.u))
            self.u.append(uu)
            self.v.append(vv)

    def _sample_region(self, flat, w, n, mpp):
        """Up to n Poisson-disk points over the canopy pixels `flat` (flat indices).
        Returns (thrower, mean spacing in metres)."""
        density_m2 = n / (len(flat) * mpp * mpp)
        spacing_mean = 1.0 / math.sqrt(density_m2)
        r_min = float(np.clip(0.65 * spacing_mean, self.MIN_SPACING_M, self.MAX_SPACING_M))
        thrower = self._DartThrower(r_min / mpp)
        k = min(len(flat) * 4, n * self.CANDIDATE_FACTOR + 1000)
        pick = flat[self.rng.integers(0, len(flat), size=k)]
        cu = (pick % w) + self.rng.random(k)
        cv_ = (pick // w) + self.rng.random(k)
        for uu, vv in zip(cu.tolist(), cv_.tolist()):
            if thrower.free(uu, vv):
                thrower.add(uu, vv)
                if len(thrower.u) >= n:
                    break
        return thrower, spacing_mean, r_min

    def place(self, canopy, luma, mpp):
        """Returns per-tree arrays (x, y, crown, height, trunk_r, conifer, shade, yaw)."""
        rng = self.rng
        h, w = canopy.shape
        flat = np.flatnonzero(canopy)
        n_canopy_px = len(flat)
        area_m2 = n_canopy_px * mpp * mpp
        n_target = int(round(self.density * area_m2 / 1e4))
        if n_canopy_px and self.density > 0:
            # A little canopy at a low density rounds to zero trees; plant at least one so the
            # patch seeding below still gives every isolated canopy patch a tree
            n_target = max(n_target, 1)
        cap_hit = n_target > self.max_trees
        self.stats.update(canopy_area_ha=round(area_m2 / 1e4, 2), target_trees=n_target, cap_hit=cap_hit)
        if n_target <= 0 or self.max_trees <= 0 or n_canopy_px == 0:
            return None

        if not cap_hit:
            regions = [(flat, n_target)]
        else:
            # Over the cap: robots work around the spawn point, so the trees nearest to it keep
            # the requested density and the rest of the budget is spread thinly over the remaining canopy.
            n_near = int(round(self.NEAR_SPAWN_SHARE * self.max_trees))
            su, sv = self.world_to_px(self.spawn_xy[0], self.spawn_xy[1], mpp)
            d2 = ((flat % w) - float(su)) ** 2 + ((flat // w) - float(sv)) ** 2
            k_near = int(min(n_canopy_px - 1, max(1, n_near / (self.density / 1e4 * mpp * mpp))))
            order = np.argpartition(d2, k_near - 1)
            near, far = flat[order[:k_near]], flat[order[k_near:]]
            radius = math.sqrt(float(d2[order[k_near - 1]])) * mpp
            del d2, order
            n_far = self.max_trees - n_near
            far_density = n_far / (len(far) * mpp * mpp / 1e4) if len(far) else 0.0
            regions = [(near, n_near), (far, n_far)]
            self.stats.update(full_density_radius_m=round(radius), far_density_per_ha=round(far_density, 1))
            self.log(f"Tree cap reached: {n_target} trees wanted at {self.density:g}/ha over "
                     f"{area_m2 / 1e4:.1f} ha of canopy, placing {self.max_trees}: full density within "
                     f"{radius:.0f} m of the spawn point, about {far_density:.1f}/ha beyond. "
                     f"Raise 'Max Trees' for a denser forest.")

        acc_u, acc_v, acc_cap, r_mins = [], [], [], []
        throwers = []
        for region_flat, n_region in regions:
            if n_region <= 0 or len(region_flat) == 0:
                continue
            thrower, spacing_mean, r_min = self._sample_region(region_flat, w, n_region, mpp)
            throwers.append(thrower)
            acc_u += thrower.u
            acc_v += thrower.v
            acc_cap += [float(np.clip(0.6 * spacing_mean, 2.0, self.CROWN_MAX_M))] * len(thrower.u)
            r_mins.append(round(r_min, 2))
        del flat, regions
        n_poisson = len(acc_u)

        # --- one tree in every isolated canopy patch that got none (only while under the cap)
        n_labels, labels = cv2.connectedComponents(canopy, connectivity=8)
        dist = cv2.distanceTransform(canopy, cv2.DIST_L2, 5)
        if not cap_hit and n_labels > 1 and throwers:
            from scipy import ndimage
            thrower = throwers[0]
            au = np.asarray(acc_u, dtype=np.int64).clip(0, w - 1)
            av = np.asarray(acc_v, dtype=np.int64).clip(0, h - 1)
            has_tree = np.zeros(n_labels, bool)
            has_tree[labels[av, au]] = True
            empty = np.flatnonzero(~has_tree)
            empty = empty[empty > 0]
            if len(empty):
                pos = ndimage.maximum_position(dist, labels, empty)
                seed_r2 = (self.MIN_SPACING_M / mpp) ** 2
                crown_cap = acc_cap[0] if acc_cap else self.CROWN_MAX_M
                for (pv, pu) in pos:
                    if len(acc_u) >= self.max_trees:
                        break
                    uu, vv = pu + 0.5, pv + 0.5
                    if thrower.free(uu, vv, seed_r2):
                        thrower.add(uu, vv)
                        acc_u.append(uu)
                        acc_v.append(vv)
                        acc_cap.append(crown_cap)
        del labels
        n_seeded = len(acc_u) - n_poisson
        if not acc_u:
            return None

        u = np.asarray(acc_u)
        v = np.asarray(acc_v)
        iu = u.astype(np.int64).clip(0, w - 1)
        iv = v.astype(np.int64).clip(0, h - 1)
        x, y = self.px_to_world(u, v, mpp)

        # --- sizes: crowns follow the local canopy width
        edge_m = dist[iv, iu] * mpp
        crown = np.minimum(edge_m + 0.5, np.asarray(acc_cap)) * rng.uniform(0.85, 1.15, len(u))
        to_spawn = np.min([np.hypot(x - sx, y - sy) for sx, sy in self.clear_xy], axis=0)
        crown = np.minimum(crown, to_spawn - self.SPAWN_CROWN_CLEARANCE_M)
        crown = np.clip(crown, self.CROWN_MIN_M, self.CROWN_MAX_M)

        # --- type: darker canopy is more likely conifer (dense evergreen / mountain forest)
        lum = luma[iv, iu].astype(np.float64)
        p_conifer = np.clip(0.15 + 0.75 * np.clip((90.0 - lum) / 50.0, 0, 1), 0, 0.9)
        conifer = rng.random(len(u)) < p_conifer

        height = np.where(conifer,
                          crown * rng.uniform(3.0, 4.2, len(u)),
                          crown * rng.uniform(2.2, 2.8, len(u)) + rng.uniform(1.0, 2.5, len(u)))
        height = np.clip(height, 2.5, self.TREE_MAX_HEIGHT_M)
        trunk_r = np.clip(0.018 * height + 0.03, 0.08, 0.5)

        # --- shade: lighter canopy in the photo -> lighter crown colour
        shade_bl = np.digitize(lum, [60.0, 95.0])
        jitter = rng.random(len(u))
        shade_bl = np.clip(shade_bl + (jitter < 0.15) - (jitter > 0.85), 0, len(self.BROADLEAF_COLORS) - 1)
        shade_cf = rng.integers(0, len(self.CONIFER_COLORS), len(u))
        shade = np.where(conifer, shade_cf, shade_bl)
        yaw = rng.uniform(0, 2 * math.pi, len(u))

        self.stats.update(trees=len(u), poisson_trees=n_poisson, patch_seed_trees=n_seeded,
                          min_spacing_m=r_mins, conifers=int(conifer.sum()))
        return dict(x=x, y=y, crown=crown, height=height, trunk_r=trunk_r,
                    conifer=conifer, shade=shade, yaw=yaw)

    # ----------------------------------------------------------------- meshes
    def _ellipsoid_template(self, rng, n_seg=8, n_ring=3, noise=0.12):
        """Unit low-poly blob: n_seg x n_ring vertices plus two poles."""
        verts = [(0.0, 0.0, -1.0)]
        for i in range(n_ring):
            phi = -math.pi / 2 + math.pi * (i + 1) / (n_ring + 1)
            for j in range(n_seg):
                th = 2 * math.pi * (j + 0.5 * (i % 2)) / n_seg
                verts.append((math.cos(phi) * math.cos(th), math.cos(phi) * math.sin(th), math.sin(phi)))
        verts.append((0.0, 0.0, 1.0))
        verts = np.asarray(verts)
        verts *= rng.uniform(1 - noise, 1 + noise, (len(verts), 1))
        faces = []
        top = len(verts) - 1
        for j in range(n_seg):
            faces.append((0, 1 + (j + 1) % n_seg, 1 + j))
        for i in range(n_ring - 1):
            a, b = 1 + i * n_seg, 1 + (i + 1) * n_seg
            for j in range(n_seg):
                j1 = (j + 1) % n_seg
                faces.append((a + j, a + j1, b + j))
                faces.append((a + j1, b + j1, b + j))
        last = 1 + (n_ring - 1) * n_seg
        for j in range(n_seg):
            faces.append((last + j, last + (j + 1) % n_seg, top))
        return verts, np.asarray(faces, dtype=np.int64)

    def _conifer_template(self, rng, sides=7):
        """Unit conifer crown (z 0..1, radius <= 1): three stacked cones with base caps."""
        tiers = [(0.00, 0.55, 1.00), (0.30, 0.45, 0.74), (0.55, 0.45, 0.48)]
        verts, faces = [], []
        for base_z, tier_h, radius in tiers:
            radius *= rng.uniform(0.92, 1.08)
            o = len(verts)
            rot = rng.uniform(0, 2 * math.pi)
            for j in range(sides):
                th = rot + 2 * math.pi * j / sides
                verts.append((radius * math.cos(th), radius * math.sin(th), base_z))
            verts.append((0.0, 0.0, min(1.0, base_z + tier_h)))  # apex
            verts.append((0.0, 0.0, base_z))  # base centre
            apex, centre = o + sides, o + sides + 1
            for j in range(sides):
                j1 = (j + 1) % sides
                faces.append((o + j, o + j1, apex))
                faces.append((o + j1, o + j, centre))
        return np.asarray(verts), np.asarray(faces, dtype=np.int64)

    def _trunk_template(self, capped):
        """Unit prism: radius 1, z 0..1. The top cap is only needed for collision."""
        s = self.TRUNK_SIDES
        verts = []
        for z in (0.0, 1.0):
            for j in range(s):
                th = 2 * math.pi * j / s
                verts.append((math.cos(th), math.sin(th), z))
        faces = []
        for j in range(s):
            j1 = (j + 1) % s
            faces.append((j, j1, s + j))
            faces.append((j1, s + j1, s + j))
        if capped:
            verts.append((0.0, 0.0, 1.0))
            c = len(verts) - 1
            for j in range(s):
                faces.append((s + j, s + (j + 1) % s, c))
        return np.asarray(verts), np.asarray(faces, dtype=np.int64)

    @staticmethod
    def _instance(tverts, tfaces, scale, yaw, trans):
        """Place a template at many positions: scale (n,3), yaw (n,), trans (n,3)."""
        n = len(yaw)
        if n == 0:
            return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
        c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        v = tverts[None, :, :] * scale[:, None, :]
        x = v[..., 0] * c - v[..., 1] * s
        y = v[..., 0] * s + v[..., 1] * c
        verts = np.stack([x, y, v[..., 2]], axis=-1) + trans[:, None, :]
        faces = tfaces[None, :, :] + (np.arange(n) * len(tverts))[:, None, None]
        return verts.reshape(-1, 3), faces.reshape(-1, 3)

    def build_meshes(self, trees):
        """Returns ({name: (verts, faces, rgb)}, (collision_verts, collision_faces))."""
        rng = self.rng
        n = len(trees['x'])
        x, y = trees['x'], trees['y']
        crown, height, trunk_r = trees['crown'], trees['height'], trees['trunk_r']
        conifer, shade, yaw = trees['conifer'], trees['shade'], trees['yaw']

        # Base z: lowest ground under the trunk footprint, so trunks never float on slopes
        probe = np.maximum(trunk_r, 0.6)
        zs = [self.ground_z(x, y)]
        for ang in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
            zs.append(self.ground_z(x + probe * math.cos(ang), y + probe * math.sin(ang)))
        base = np.min(np.stack(zs), axis=0)
        trees['z'] = base

        # Trunks: from below ground up into the crown
        trunk_top = np.where(conifer, 0.55 * height, height - 0.9 * crown)
        trunk_top = np.maximum(trunk_top, 1.0)
        trunk_len = trunk_top + self.TRUNK_SINK_M
        t_scale = np.stack([trunk_r, trunk_r, trunk_len], axis=1)
        t_trans = np.stack([x, y, base - self.TRUNK_SINK_M], axis=1)
        tv, tf = self._trunk_template(capped=False)
        trunks = self._instance(tv, tf, t_scale, yaw, t_trans)
        cv, cf = self._trunk_template(capped=True)
        collision = self._instance(cv, cf, t_scale, yaw, t_trans)

        meshes = {'trunks': (trunks[0], trunks[1], self.TRUNK_COLOR)}
        templates_bl = [self._ellipsoid_template(rng) for _ in range(self.TEMPLATES_PER_KIND)]
        templates_cf = [self._conifer_template(rng) for _ in range(self.TEMPLATES_PER_KIND)]
        variant = rng.integers(0, self.TEMPLATES_PER_KIND, n)

        # Broadleaf crowns: ellipsoid centred one vertical radius below the top
        rz = crown * rng.uniform(0.75, 1.0, n)
        for si, color in enumerate(self.BROADLEAF_COLORS):
            parts_v, parts_f, off = [], [], 0
            for ti, (bv, bf) in enumerate(templates_bl):
                sel = (~conifer) & (shade == si) & (variant == ti)
                if not sel.any():
                    continue
                scale = np.stack([crown[sel], crown[sel], rz[sel]], axis=1)
                trans = np.stack([x[sel], y[sel], base[sel] + height[sel] - rz[sel]], axis=1)
                v, f = self._instance(bv, bf, scale, yaw[sel], trans)
                parts_v.append(v)
                parts_f.append(f + off)
                off += len(v)
            if parts_v:
                meshes[f'crowns_broadleaf_{si}'] = (np.concatenate(parts_v), np.concatenate(parts_f), color)

        # Conifer crowns: from 25% of the height up to the top
        crown_base = 0.25 * height
        for si, color in enumerate(self.CONIFER_COLORS):
            parts_v, parts_f, off = [], [], 0
            for ti, (bv, bf) in enumerate(templates_cf):
                sel = conifer & (shade == si) & (variant == ti)
                if not sel.any():
                    continue
                scale = np.stack([crown[sel], crown[sel], height[sel] - crown_base[sel]], axis=1)
                trans = np.stack([x[sel], y[sel], base[sel] + crown_base[sel]], axis=1)
                v, f = self._instance(bv, bf, scale, yaw[sel], trans)
                parts_v.append(v)
                parts_f.append(f + off)
                off += len(v)
            if parts_v:
                meshes[f'crowns_conifer_{si}'] = (np.concatenate(parts_v), np.concatenate(parts_f), color)
        return meshes, collision

    @staticmethod
    def _vertex_normals(verts, faces):
        fn = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]], verts[faces[:, 2]] - verts[faces[:, 0]])
        vn = np.zeros_like(verts)
        for k in range(3):
            np.add.at(vn, faces[:, k], fn)
        length = np.linalg.norm(vn, axis=1, keepdims=True)
        length[length == 0] = 1.0
        return vn / length

    def write_dae(self, path, meshes):
        """Write {name: (verts, faces, rgb)} as one COLLADA file, one sub-mesh per entry."""
        import collada
        from collada import geometry, material, scene, source
        doc = collada.Collada()
        nodes = []
        for name, (verts, faces, rgb) in meshes.items():
            if len(faces) == 0:
                continue
            color = (float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0)
            effect = material.Effect(f'{name}-effect', [], 'phong',
                                     diffuse=color, ambient=color,
                                     specular=(0.03, 0.03, 0.03, 1.0), shininess=2.0,
                                     reflective=None, reflectivity=None,
                                     transparent=None, transparency=None)
            mat = material.Material(f'{name}-material', name, effect)
            doc.effects.append(effect)
            doc.materials.append(mat)
            normals = self._vertex_normals(verts, faces)
            # Millimetre positions and 3-decimal normals keep the text file small
            vsrc = source.FloatSource(f'{name}-positions', np.round(verts, 3).ravel(), ('X', 'Y', 'Z'))
            nsrc = source.FloatSource(f'{name}-normals', np.round(normals, 3).ravel(), ('X', 'Y', 'Z'))
            geom = geometry.Geometry(doc, f'{name}-geometry', name, [vsrc, nsrc])
            # Separate offsets with the index repeated: gz-common's COLLADA loader steps through
            # <p> by the number of <input>s, so inputs sharing one offset are misread.
            inputs = source.InputList()
            inputs.addInput(0, 'VERTEX', f'#{name}-positions')
            inputs.addInput(1, 'NORMAL', f'#{name}-normals')
            triset = geom.createTriangleSet(np.repeat(faces.ravel(), 2), inputs, f'{name}-symbol')
            geom.primitives.append(triset)
            doc.geometries.append(geom)
            matnode = scene.MaterialNode(f'{name}-symbol', mat, inputs=[])
            nodes.append(scene.Node(name, children=[scene.GeometryNode(geom, [matnode])]))
        sc = scene.Scene('scene', nodes)
        doc.scenes.append(sc)
        doc.scene = sc
        doc.write(path)

    # ---------------------------------------------------------------- outputs
    def write_csv(self, path, trees):
        cols = ['x', 'y', 'z', 'yaw', 'height', 'crown_radius', 'trunk_radius', 'conifer']
        with open(path, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(cols)
            for row in zip(trees['x'], trees['y'], trees['z'], trees['yaw'], trees['height'],
                           trees['crown'], trees['trunk_r'], trees['conifer']):
                wr.writerow([f'{row[0]:.2f}', f'{row[1]:.2f}', f'{row[2]:.2f}', f'{row[3]:.3f}',
                             f'{row[4]:.2f}', f'{row[5]:.2f}', f'{row[6]:.2f}', int(row[7])])

    def output_paths(self):
        mesh_dir = os.path.join(self.output_dir, 'mesh')
        return {
            'trees_dae': os.path.join(mesh_dir, 'trees.dae'),
            'collision_dae': os.path.join(mesh_dir, 'trees_collision.dae'),
            'mask_png': os.path.join(mesh_dir, 'vegetation_mask.png'),
            'csv': os.path.join(self.output_dir, 'trees.csv'),
        }

    def remove_outputs(self):
        for p in self.output_paths().values():
            if os.path.isfile(p):
                os.remove(p)

    # -------------------------------------------------------------------- run
    def run(self):
        """Generate the forest. Returns a stats dict; 'trees' is 0 when nothing was written."""
        t0 = time.time()
        self.remove_outputs()
        img = cv2.imread(self.aerial_path, cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(f"Cannot read {self.aerial_path}")
        src_h, src_w = img.shape[:2]
        n_src = src_w
        mpp = self.texture_size / n_src
        scale = min(1.0, self.PROC_MAX_DIM / n_src, mpp / self.PROC_MIN_MPP)
        if scale < 1.0:
            size = max(1, int(round(n_src * scale)))
            img = cv2.resize(img, (size, size), interpolation=cv2.INTER_AREA)
        mpp = self.texture_size / img.shape[1]
        self.stats['mask_m_per_px'] = round(mpp, 3)

        valid = self.valid_area(img, mpp)
        veg, smooth, luma = self.classify(img, mpp)
        veg &= valid
        del img

        # Lawns and fields: smooth vegetation that survives an opening of LAWN_MIN_WIDTH_M
        lawn = self._morph(smooth, cv2.MORPH_OPEN, self.LAWN_MIN_WIDTH_M, mpp)
        k = self._kernel(self.CLOSE_M, mpp)
        if k is not None:
            lawn = cv2.dilate(lawn, k)
        lawn &= veg
        canopy = veg & (1 - lawn)
        canopy = self._morph(canopy, cv2.MORPH_CLOSE, self.CLOSE_M, mpp, minimum=3)
        canopy = self._morph(canopy, cv2.MORPH_OPEN, self.OPEN_M, mpp)

        excl = self.exclusion_mask(canopy.shape, mpp)
        canopy &= valid & (1 - excl)

        n_lab, labels, cc_stats, _ = cv2.connectedComponentsWithStats(canopy, connectivity=8)
        min_px = max(2, int(math.ceil(self.MIN_BLOB_M2 / (mpp * mpp))))
        keep = np.zeros(n_lab, np.uint8)
        keep[1:] = cc_stats[1:, cv2.CC_STAT_AREA] >= min_px
        canopy = keep[labels]
        del labels

        veg_area = float(np.count_nonzero(veg | canopy)) * mpp * mpp
        self.stats['vegetation_area_ha'] = round(veg_area / 1e4, 2)
        self.stats['canopy_patches'] = int(keep.sum())
        self.mask_vis = (canopy, veg, excl, mpp)  # kept for previews

        paths = self.output_paths()
        os.makedirs(os.path.dirname(paths['mask_png']), exist_ok=True)
        mask_png = np.where(canopy > 0, 255, np.where(veg > 0, 128, 0)).astype(np.uint8)
        if mask_png.shape[:2] != (src_h, src_w):
            # Keep the documented contract (same pixel grid as aerial.png) when the texture
            # was downsampled for processing
            mask_png = cv2.resize(mask_png, (src_w, src_h), interpolation=cv2.INTER_NEAREST_EXACT)
        cv2.imwrite(paths['mask_png'], mask_png)
        del mask_png

        t_mask = time.time()
        trees = self.place(canopy, luma, mpp)
        t_place = time.time()
        if trees is None or len(trees['x']) == 0:
            self.stats.update(trees=0, seconds=round(time.time() - t0, 2))
            self.log("No tree canopy found in the satellite texture; no trees added.")
            return self.stats

        meshes, collision = self.build_meshes(trees)
        self.write_dae(paths['trees_dae'], meshes)
        # The collision mesh keeps its normals: gz-physics' dartsim plugin drops sub-meshes without them
        self.write_dae(paths['collision_dae'], {'trunks_collision': (collision[0], collision[1], self.TRUNK_COLOR)})
        self.write_csv(paths['csv'], trees)
        self.trees = trees
        t_end = time.time()

        n_tris = sum(len(m[1]) for m in meshes.values())
        self.stats.update(
            visual_triangles=int(n_tris), collision_triangles=int(len(collision[1])),
            seconds_mask=round(t_mask - t0, 2), seconds_placement=round(t_place - t_mask, 2),
            seconds_mesh=round(t_end - t_place, 2), seconds=round(t_end - t0, 2),
            trees_dae_mb=round(os.path.getsize(paths['trees_dae']) / 1e6, 2),
            collision_dae_mb=round(os.path.getsize(paths['collision_dae']) / 1e6, 2))
        self.log(f"Planted {self.stats['trees']} trees on {self.stats['canopy_area_ha']} ha of canopy "
                 f"({self.stats['conifers']} conifers) in {self.stats['seconds']} s.")
        return self.stats
