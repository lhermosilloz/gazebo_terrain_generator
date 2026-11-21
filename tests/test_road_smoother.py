"""
Offline tests for scripts/utils/road_smoother.py (no network, no Mapbox token).

Run from the repo root:
    python -m unittest tests.test_road_smoother -v
"""
import math
import os
import sys
import unittest

import numpy as np
from scipy.ndimage import gaussian_filter, map_coordinates

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

from utils.road_smoother import (  # noqa: E402
    roads_from_geojson, smooth_heightmap_roads, smooth_roads, road_width_m,
)


def road_metrics(height_m, roads, size_x, size_y, step=1.0, piece=5.0):
    """
    Profile metrics along road centrelines of a heightmap given in metres.

    Returns:
        dict with
          cross_slope_pct: mean |left - right| / spacing at +-0.8 half-width, in %
          grade_change_pct: RMS change of grade between consecutive `piece`-metre pieces, in %
          bump_rms_m: RMS deviation of the profile from a 30 m moving average (vehicle-scale
              bumps), in metres
    """
    rows, cols = height_m.shape
    sx, sy = size_x / (cols - 1), size_y / (rows - 1)
    cross, dgrade, bumps = [], [], []
    for coords, width in roads:
        xy = np.column_stack([coords[:, 0] * sx, coords[:, 1] * sy])
        seg = np.hypot(*np.diff(xy, axis=0).T)
        t = np.concatenate([[0.0], np.cumsum(seg)])
        if t[-1] < 3 * piece:
            continue
        td = np.arange(0.0, t[-1], step)
        px = np.interp(td, t, xy[:, 0])
        py = np.interp(td, t, xy[:, 1])
        tx, ty = np.gradient(px), np.gradient(py)
        norm = np.hypot(tx, ty)
        nx, ny = -ty / norm, tx / norm
        off = 0.8 * width / 2.0

        def sample(x, y):
            return map_coordinates(height_m, [y / sy, x / sx], order=1, mode='nearest')

        centre = sample(px, py)
        left = sample(px + nx * off, py + ny * off)
        right = sample(px - nx * off, py - ny * off)
        cross.append(np.abs(left - right) / (2 * off) * 100.0)
        k = int(round(piece / step))
        grades = np.diff(centre[::k]) / piece * 100.0
        dgrade.append(np.diff(grades))
        win = int(round(30.0 / step))
        if len(centre) > 2 * win:
            kernel = np.ones(win) / win
            smooth = np.convolve(centre, kernel, mode='valid')
            core = centre[win // 2: win // 2 + len(smooth)]
            bumps.append(core - smooth)
    cat = np.concatenate
    return {
        'cross_slope_pct': float(np.mean(cat(cross))),
        'grade_change_pct': float(np.sqrt(np.mean(cat(dgrade) ** 2))),
        'bump_rms_m': float(np.sqrt(np.mean(cat(bumps) ** 2))) if bumps else float('nan'),
    }


def corridor_distance(shape, roads, size_x, size_y):
    """Distance (m) from every pixel to the nearest road centreline (dense rasterisation)."""
    from scipy.ndimage import distance_transform_edt
    rows, cols = shape
    sx, sy = size_x / (cols - 1), size_y / (rows - 1)
    line = np.zeros(shape, dtype=bool)
    for coords, _ in roads:
        seg = np.hypot(*(np.diff(coords, axis=0) * [sx, sy]).T)
        t = np.concatenate([[0.0], np.cumsum(seg)])
        td = np.arange(0.0, t[-1], 0.25 * min(sx, sy))
        c = np.rint(np.interp(td, t, coords[:, 0])).astype(int)
        r = np.rint(np.interp(td, t, coords[:, 1])).astype(int)
        ok = (c >= 0) & (c < cols) & (r >= 0) & (r < rows)
        line[r[ok], c[ok]] = True
    return distance_transform_edt(~line, sampling=(sy, sx))


class SyntheticTerrain:
    """513 x 513 heightmap of a 1000 m x 800 m area: hills + DEM-like noise + tilt."""

    def __init__(self, seed=0):
        rng = np.random.default_rng(seed)
        self.rows = self.cols = 513
        self.size_x, self.size_y = 1000.0, 800.0
        x = np.linspace(0, self.size_x, self.cols)[None, :]
        y = np.linspace(0, self.size_y, self.rows)[:, None]
        hills = 15.0 * np.sin(2 * np.pi * x / 700.0) * np.cos(2 * np.pi * y / 900.0) + 0.02 * x + 0.01 * y
        noise = gaussian_filter(rng.standard_normal((self.rows, self.cols)), sigma=4.0)
        noise *= 1.2 / noise.std()  # ~1.2 m RMS bumps at ~20-40 m wavelength, like SRTM noise
        self.height_m = hills + noise
        self.min, self.max = self.height_m.min(), self.height_m.max()
        self.size_z = self.max - self.min
        self.u16 = np.rint((self.height_m - self.min) / self.size_z * 65535).astype(np.uint16)
        # Arbitrary geographic placement for the GeoJSON round trip
        self.bounds = {'southwest': (37.0, -122.0), 'northeast': (37.0072, -121.9888)}

    def to_m(self, img):
        return img.astype(np.float64) / 65535.0 * self.size_z + self.min

    def px_to_lonlat(self, col, row):
        lat_min, lon_min = self.bounds['southwest']
        lat_max, lon_max = self.bounds['northeast']
        lon = lon_min + np.asarray(col) / (self.cols - 1) * (lon_max - lon_min)
        lat = lat_max - np.asarray(row) / (self.rows - 1) * (lat_max - lat_min)
        return lon, lat

    def feature(self, cols, rows, props):
        lon, lat = self.px_to_lonlat(cols, rows)
        return {'type': 'Feature', 'properties': props,
                'geometry': {'type': 'LineString', 'coordinates': np.column_stack([lon, lat]).tolist()}}


class RoadSmootherTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.terrain = T = SyntheticTerrain()
        s = np.linspace(0, 1, 200)
        curve_c = 20 + 470 * s
        curve_r = 300 + 90 * np.sin(2 * np.pi * s * 1.3)
        cls.geojson = {'type': 'FeatureCollection', 'features': [
            T.feature(40 + 420 * s, 50 + 380 * s, {'class': 'street', 'structure': 'none'}),
            T.feature(curve_c, curve_r, {'class': 'primary', 'structure': 'none'}),
            # Service road ending on the primary road (T-junction)
            T.feature([256, 256], [40, curve_r[np.argmin(np.abs(curve_c - 256))]], {'class': 'service', 'type': 'service'}),
            # Must be ignored
            T.feature([420, 500], [40, 120], {'class': 'primary', 'structure': 'bridge'}),
            T.feature([60, 140], [460, 500], {'class': 'path', 'type': 'footway', 'structure': 'none'}),
        ]}
        cls.result, cls.stats = smooth_heightmap_roads(
            T.u16, cls.geojson, T.bounds, T.size_x, T.size_y, smoothing_length=80.0, shoulder_width=4.0)
        cls.roads = roads_from_geojson(cls.geojson, T.bounds, T.u16.shape)

    def test_class_filter(self):
        self.assertEqual(road_width_m({'class': 'primary', 'structure': 'bridge'}), None)
        self.assertEqual(road_width_m({'class': 'street', 'structure': 'tunnel'}), None)
        self.assertEqual(road_width_m({'class': 'path'}), None)
        self.assertEqual(road_width_m({'class': 'ferry'}), None)
        self.assertEqual(road_width_m({'class': 'service', 'type': 'service:driveway'}), 3.5)
        self.assertEqual(len(self.roads), 3)

    def test_dtype_and_range(self):
        self.assertEqual(self.result.dtype, np.uint16)
        self.assertEqual(self.result.shape, self.terrain.u16.shape)
        self.assertGreater(self.stats['pixels_changed'], 0)

    def test_road_profile_is_smoother(self):
        T = self.terrain
        before = road_metrics(T.to_m(T.u16), self.roads, T.size_x, T.size_y)
        after = road_metrics(T.to_m(self.result), self.roads, T.size_x, T.size_y)
        print('\nsynthetic before', before, '\nsynthetic after ', after, '\nstats', self.stats)
        self.assertLess(after['cross_slope_pct'], 0.05 * before['cross_slope_pct'])
        self.assertLess(after['grade_change_pct'], 0.3 * before['grade_change_pct'])
        self.assertLess(after['bump_rms_m'], 0.3 * before['bump_rms_m'])

    def test_off_road_terrain_unchanged(self):
        T = self.terrain
        dist = corridor_distance(T.u16.shape, self.roads, T.size_x, T.size_y)
        px = max(T.size_x / (T.cols - 1), T.size_y / (T.rows - 1))
        far = dist > 11.0 / 2 + 4.0 + 2 * px  # widest road half width + shoulder + margin
        self.assertTrue(np.array_equal(self.result[far], T.u16[far]))
        # The bridge and the footpath were skipped: their surroundings are untouched
        changed = self.result != T.u16
        self.assertFalse(changed[40:120, 420:500][dist[40:120, 420:500] > 10].any())
        self.assertFalse(changed[455:505, 60:140].any())

    def test_junction_is_continuous(self):
        # Height along the service road where it meets the primary road equals the primary road there
        T = self.terrain
        h = T.to_m(self.result)
        coords, _ = self.roads[2]
        jc, jr = coords[-1]
        sx, sy = T.size_x / (T.cols - 1), T.size_y / (T.rows - 1)
        near = map_coordinates(h, [[jr - 2.0 / sy * 3], [jc]], order=1)[0]  # 6 m up the service road
        at = map_coordinates(h, [[jr], [jc]], order=1)[0]
        self.assertLess(abs(near - at), 0.5)

    def test_grade_is_preserved_on_a_plane(self):
        # A straight road up a tilted plane keeps its grade; only the cross slope is removed.
        rows = cols = 257
        size = 500.0
        x = np.linspace(0, size, cols)[None, :]
        y = np.linspace(0, size, rows)[:, None]
        plane = 0.06 * x + 0.04 * y + 0 * x * y  # 6 % east, 4 % south
        road = [(np.array([[20.0, 128.0], [236.0, 128.0]]), 8.0)]  # runs east, 8 m wide
        out, _ = smooth_roads(plane, road, size, size, smoothing_length=50.0, shoulder_width=4.0)
        centre = out[128, 30:227]
        grade = np.diff(centre) / (size / (cols - 1)) * 100.0
        self.assertTrue(np.allclose(grade, 6.0, atol=1e-6), grade)
        # Centreline heights unchanged on a plane, and the road is flat across:
        # rows 126 and 130 are 3.9 m either side, inside the 4 m half width.
        self.assertTrue(np.allclose(out[128, 30:227], plane[128, 30:227]))
        self.assertTrue(np.allclose(out[126, 30:227], out[130, 30:227], atol=1e-9))
        self.assertTrue(np.allclose(out[126, 30:227], plane[128, 30:227], atol=1e-9))

    def test_no_roads_is_identity(self):
        T = self.terrain
        empty = {'type': 'FeatureCollection', 'features': []}
        out, stats = smooth_heightmap_roads(T.u16, empty, T.bounds, T.size_x, T.size_y)
        self.assertTrue(np.array_equal(out, T.u16))
        self.assertEqual(stats['pixels_changed'], 0)

    def test_uint8_heightmap(self):
        T = self.terrain
        u8 = np.rint(T.u16 / 257.0).astype(np.uint8)
        out, stats = smooth_heightmap_roads(u8, self.geojson, T.bounds, T.size_x, T.size_y)
        self.assertEqual(out.dtype, np.uint8)
        self.assertGreater(stats['pixels_changed'], 0)


if __name__ == '__main__':
    unittest.main()
