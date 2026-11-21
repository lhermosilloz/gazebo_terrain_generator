"""
Road centerlines for road smoothing, from the Mapbox streets-v8 "road" layer.

Uses the same vector tiles, downloader and on-disk cache layout as building_downloader
({tiles_dir}/{zoom}/{x}/{y}.json holding every decoded layer), so when buildings are
enabled the tiles are downloaded only once.
"""
import json
import os
from multiprocessing import Pool, cpu_count

import mercantile
import numpy as np
import shapely
from shapely.geometry import LineString, MultiLineString, box, mapping, shape

from utils.building_downloader import BuildingDownloader
from utils.maptile_utils import MapTileUtils
from utils.param import GlobalParam

# Properties kept per road (see https://docs.mapbox.com/data/tilesets/reference/mapbox-streets-v8/#road)
ROAD_PROPERTIES = ('class', 'type', 'structure', 'layer', 'surface')


def tile_road_features(tile_path: str, tile_x: int, tile_y: int, zoom: int) -> list:
    """
    Read the "road" layer of one cached vector tile as GeoJSON line features (lon/lat).

    Lines are clipped to the tile's own extent: tiles carry a small buffer of their
    neighbours' data, and keeping it would duplicate roads across tile seams.
    """
    with open(tile_path) as f:
        tile = json.load(f)
    layer = tile.get('road')
    if not layer:
        return []
    extent = layer.get('extent', 4096)
    bounds = mercantile.bounds(tile_x, tile_y, zoom)
    tile_box = box(0, 0, extent, extent)

    def to_lonlat(coords):
        # mapbox_vector_tile decodes with y = 0 at the bottom (south) of the tile
        return np.column_stack([
            bounds.west + coords[:, 0] / extent * (bounds.east - bounds.west),
            bounds.south + coords[:, 1] / extent * (bounds.north - bounds.south),
        ])

    features = []
    for feature in layer.get('features', []):
        geometry = feature.get('geometry') or {}
        if geometry.get('type') not in ('LineString', 'MultiLineString'):
            continue  # points (turning circles, junctions) and areas carry no centerline
        try:
            clipped = shape(geometry).intersection(tile_box)
        except Exception as e:
            print(f"Skipping invalid road geometry in tile {zoom}/{tile_x}/{tile_y}: {e}")
            continue
        parts = [g for g in getattr(clipped, 'geoms', [clipped])
                 if isinstance(g, LineString) and not g.is_empty and g.length > 0]
        if not parts:
            continue
        lines = shapely.transform(parts[0] if len(parts) == 1 else MultiLineString(parts), to_lonlat)
        properties = feature.get('properties', {})
        features.append({
            'type': 'Feature',
            'properties': {k: properties[k] for k in ROAD_PROPERTIES if k in properties},
            'geometry': mapping(lines),
        })
    return features


def download_road_data(bound_array, tiles_dir: str, model_path: str, api_key: str = None,
                       zoom_level: int = GlobalParam.DEM_BUILDING_RESOLUTION) -> dict:
    """
    Download (or reuse cached) streets-v8 vector tiles and write {model_path}/roads.geojson.

    Args:
        bound_array: dict with "northwest" and "southeast" (lat, lon) corners.
        tiles_dir: vector tile cache, shared with download_streetmap_data().
        model_path: map directory; roads.geojson is written here.
        api_key: Mapbox access token.
        zoom_level: vector tile zoom (15 has full street detail).

    Returns:
        dict: {"features": road features written, "tiles": tiles in the area,
               "missing": tiles that could not be downloaded}.
    """
    nw_lat, nw_lon = map(float, bound_array["northwest"])
    se_lat, se_lon = map(float, bound_array["southeast"])
    nw_tile_x, nw_tile_y = MapTileUtils.lat_lon_to_tile(nw_lat, nw_lon, zoom_level)
    se_tile_x, se_tile_y = MapTileUtils.lat_lon_to_tile(se_lat, se_lon, zoom_level)
    tile_x_start, tile_x_end = sorted((nw_tile_x, se_tile_x))
    tile_y_start, tile_y_end = sorted((nw_tile_y, se_tile_y))

    zoom_dir = os.path.join(tiles_dir, str(zoom_level))
    tasks = []
    for tile_x in range(tile_x_start, tile_x_end + 1):
        tile_x_dir = os.path.join(zoom_dir, str(tile_x))
        MapTileUtils.dir_check(tile_x_dir)
        for tile_y in range(tile_y_start, tile_y_end + 1):
            if not os.path.isfile(os.path.join(tile_x_dir, f"{tile_y}.json")):
                tasks.append((zoom_level, tile_x, tile_y, tile_x_dir, api_key))
    if tasks:
        print(f"Downloading {len(tasks)} road tiles…")
        with Pool(processes=min(cpu_count(), len(tasks))) as pool:
            pool.starmap(BuildingDownloader.download_tile, tasks)

    features = []
    missing = 0
    for tile_x in range(tile_x_start, tile_x_end + 1):
        for tile_y in range(tile_y_start, tile_y_end + 1):
            tile_path = os.path.join(zoom_dir, str(tile_x), f"{tile_y}.json")
            if not os.path.isfile(tile_path):
                missing += 1
                continue
            features.extend(tile_road_features(tile_path, tile_x, tile_y, zoom_level))
    if missing:
        print(f"Warning: {missing} road tiles could not be downloaded; roads there are not smoothed")

    os.makedirs(model_path, exist_ok=True)
    with open(os.path.join(model_path, 'roads.geojson'), 'w') as f:
        json.dump({'type': 'FeatureCollection', 'features': features}, f)
    print(f"Saved {len(features)} road features to {os.path.join(model_path, 'roads.geojson')}")
    n_tiles = (tile_x_end - tile_x_start + 1) * (tile_y_end - tile_y_start + 1)
    return {"features": len(features), "tiles": n_tiles, "missing": missing}
