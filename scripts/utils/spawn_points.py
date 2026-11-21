"""
Multiple spawn points (multi-spawn feature).

The primary spawn point is the existing launch location: it is the world origin
(<spherical_coordinates>) and, when helipads are on, carries the original
helipad. Additional spawn points are placed relative to it.

This module holds the parts that do not need the generated heightmap:
parsing/validating the spawn points sent by the web UI, and rendering the
SDF snippets and spawn_points.json from poses computed by the generator.
"""
import json
import math
import os

from shapely.geometry import Point, Polygon

# Upper bound on spawn points per world (primary included). Each helipad is a
# Fuel <include>, so this also bounds the number of helipads.
MAX_SPAWN_POINTS = 32

HELIPAD_URI = 'https://fuel.gazebosim.org/1.0/saiaravind19/models/helipad'

# Planar size of the Fuel helipad model (5 m x 5 m), used for overlap warnings
HELIPAD_SIZE_M = 5.0


def spawn_name(index: int) -> str:
    """Name of the spawn point at 0-based index (primary = spawn_1)."""
    return f"spawn_{index + 1}"


def parse_spawn_points(raw, polygon_vertices, launch_location=None):
    """
    Parse and validate the "spawnPoints" form field.

    Args:
        raw (str|None): JSON array of {"lng", "lat", "yawDeg"} objects. The first
            entry is the primary spawn point; its coordinates are replaced by
            launch_location when given (the launch location is authoritative).
        polygon_vertices (list): [[lng, lat], ...] polygon the points must lie in.
        launch_location (list|None): [lng, lat] of the primary spawn point.

    Returns:
        list[dict] | None: [{"name", "lng", "lat", "yaw_deg"}], primary first,
        or None when the field is missing or empty (single spawn point, as before).

    Raises:
        ValueError: malformed input, too many points, or an additional spawn
            point outside the polygon.
    """
    if raw is None or str(raw).strip() in ('', 'null', '[]'):
        return None
    try:
        items = json.loads(raw)
    except (TypeError, ValueError) as e:
        raise ValueError(f"spawnPoints is not valid JSON: {e}")
    if not isinstance(items, list):
        raise ValueError("spawnPoints must be a JSON array")
    if not items:
        return None
    if len(items) > MAX_SPAWN_POINTS:
        raise ValueError(f"At most {MAX_SPAWN_POINTS} spawn points are supported (got {len(items)})")

    # Small buffer so points exactly on an edge (allowed by the UI check) pass
    polygon = Polygon(polygon_vertices).buffer(1e-9)
    points = []
    for i, item in enumerate(items):
        if not isinstance(item, dict):
            raise ValueError(f"Spawn point {i + 1} must be an object")
        try:
            lng = float(item['lng'])
            lat = float(item['lat'])
            yaw_deg = float(item.get('yawDeg', 0) or 0)
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"Spawn point {i + 1} needs numeric lng, lat and yawDeg")
        if not all(math.isfinite(v) for v in (lng, lat, yaw_deg)):
            raise ValueError(f"Spawn point {i + 1} has a non-finite value")
        if i == 0 and launch_location is not None:
            lng, lat = float(launch_location[0]), float(launch_location[1])
        elif i > 0 and not polygon.covers(Point(lng, lat)):
            raise ValueError(f"Spawn point {i + 1} ({lat:.6f}, {lng:.6f}) is outside the polygon")
        # Normalise yaw to (-180, 180]
        yaw_deg = -((-yaw_deg + 180.0) % 360.0 - 180.0) + 0.0
        points.append({"name": spawn_name(i), "lng": lng, "lat": lat, "yaw_deg": yaw_deg})
    return points


def spawn_feature_active(spawn_points) -> bool:
    """
    True when spawn points change the output: more than one point, or a
    non-zero yaw on the primary. With a single spawn point at yaw 0 the world
    is written exactly as before this feature existed.
    """
    if not spawn_points:
        return False
    return len(spawn_points) > 1 or abs(spawn_points[0].get("yaw_deg", 0.0)) > 1e-9


def frame_names(model_name: str, count: int) -> list:
    """
    Names for the world-level spawn frames. Frame names share the world's
    namespace with the terrain model (named after the world), so fall back to a
    longer prefix in the unlikely case the world itself is called spawn_N.
    """
    names = [spawn_name(i) for i in range(count)]
    if model_name in names:
        names = [f"spawn_point_{i + 1}" for i in range(count)]
    return names


def _fmt(value: float, digits: int = 3) -> str:
    """Compact fixed-point formatting for SDF poses (no '-0')."""
    text = f"{round(value, digits):.{digits}f}".rstrip('0').rstrip('.')
    return '0' if text in ('-0', '') else text


def helipad_include(name: str, x: float, y: float, z: float, yaw: float) -> str:
    """One helipad <include> block, indented for the terrain model."""
    return (
        f'        <include>\n'
        f'            <name>{name}</name>\n'
        f'            <uri>{HELIPAD_URI}</uri>\n'
        f'            <pose>{_fmt(x)} {_fmt(y)} {_fmt(z)} 0 0 {_fmt(yaw, 6)}</pose>\n'
        f'            <static>true</static>\n'
        f'        </include>'
    )


def frames_block(spawn_poses) -> str:
    """World-level <frame> elements, one per spawn pose."""
    if not spawn_poses:
        return ""
    lines = [
        '        <!-- Spawn points: ENU poses relative to the world origin (the primary spawn point).',
        '             z is the terrain surface, or the helipad surface where a helipad was placed.',
        '             Place a robot at one with e.g. <pose relative_to="' + spawn_poses[-1]["frame"] + '">0 0 0.2 0 0 0</pose>.',
        '             The same poses, with GPS coordinates, are listed in spawn_points.json. -->',
    ]
    for p in spawn_poses:
        lines.append(f'        <frame name="{p["frame"]}">')
        lines.append(f'            <pose>{_fmt(p["x"])} {_fmt(p["y"])} {_fmt(p["z"])} 0 0 {_fmt(p["yaw"], 6)}</pose>')
        lines.append('        </frame>')
    # Leading newline: blank line between </spherical_coordinates> and the frames
    return '\n' + '\n'.join(lines)


def write_spawn_points_json(output_dir, model_name, spawn_poses, origin_lat, origin_lon,
                            origin_elevation, include_helipad, helipad_height):
    """
    Write spawn_points.json next to the world file.

    alt is origin_elevation + z: the altitude Gazebo's NavSat sensor reports at
    the spawn pose, given the <spherical_coordinates> written to the world
    (ignoring Earth curvature, < 0.1 m over 1 km).
    """
    origin_elevation = float(origin_elevation)
    data = {
        "world": model_name,
        "description": (
            "x, y, z: metres in the world frame (ENU: x east, y north, z up), relative to the "
            "world origin, which is the primary spawn point (spawn_1). z is the terrain surface, "
            "or the helipad surface where the spawn point has a helipad. yaw: radians, "
            "counter-clockwise from east (yaw_deg in degrees). lat/lon: WGS84 degrees. "
            "alt: metres AMSL as Gazebo reports it at the pose (origin.elevation + z). "
            "ground_amsl: DEM terrain elevation at lat/lon. Each spawn point is also a world-level "
            "SDF <frame> named after it."
        ),
        "origin": {
            "lat": float(origin_lat),
            "lon": float(origin_lon),
            "elevation": round(origin_elevation, 3),
        },
        "helipads": bool(include_helipad),
        "helipad_height": float(helipad_height) if include_helipad else None,
        "spawn_points": [
            {
                "name": p["frame"],
                "primary": p["primary"],
                "lat": float(p["lat"]),
                "lon": float(p["lon"]),
                "alt": round(origin_elevation + float(p["z"]), 3),
                "ground_amsl": None if p.get("ground_amsl") is None else round(float(p["ground_amsl"]), 3),
                "x": round(float(p["x"]), 3),
                "y": round(float(p["y"]), 3),
                "z": round(float(p["z"]), 3),
                "yaw": round(float(p["yaw"]), 6),
                "yaw_deg": round(float(p["yaw_deg"]), 3),
                "helipad": p.get("helipad"),
            }
            for p in spawn_poses
        ],
    }
    with open(os.path.join(output_dir, 'spawn_points.json'), 'w') as f:
        json.dump(data, f, indent=2)
