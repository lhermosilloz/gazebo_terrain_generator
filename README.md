# Gazebo Terrain Generator  [![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/saiaravind19/gazebo_terrain_generator)

A super easy-to-use tool for generating 3D Gazebo terrain using real-world elevation and satellite data. Draw a polygon on a map, set one or more spawn locations, and get a ready-to-use `.world` file with a textured heightmap, smoothed roads, trees and optional 3D buildings.


<video src="https://github.com/user-attachments/assets/42289e73-c66a-4605-85c6-c95d13139d44" controls width="100%"></video>


## Features

- **Real-World Terrain**: Generate 3D Gazebo worlds from actual elevation and satellite imagery of any location on Earth.
- **3D Buildings**: Toggle OSM building footprints extruded as 3D meshes.
- **Configurable Spawn Locations**: Drag a marker on the map to set the robot spawn point — GPS coordinates are baked in automatically. Add more numbered spawn points for multi-robot setups; each becomes an SDF frame and can get its own helipad.
- **Smooth Roads**: Drivable roads are flattened across and smoothed along their length, so vehicles don't tilt or bounce on elevation-data noise.
- **Trees**: Low-poly trees are planted where the satellite texture shows tree canopy, kept off buildings, roads, water and spawn points.
- **Ground Friction**: Pick a ground surface (asphalt, sand, ice, ...) or set the friction coefficient (Gazebo Harmonic, needs a small plugin).
- **Satellite Texture**: Stitched from configurable tile sources (Google, OSM, etc.).
- **High-Precision Heightmap**: 16-bit (~0.008m precision), 8-bit for Fortress compatibility.
- **Downloadable Output**: Get a `.zip` — unzip anywhere and run.


**Supports:** [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) (recommended) · [Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)


## 🔑 Prerequisites

- **Mapbox API Key** — Required for satellite imagery, elevation data, geocoding and street map data. Sign up at [mapbox.com](https://www.mapbox.com/), copy your public token (`pk.eyJ1...`), then paste it in the web UI under **Settings → Mapbox API Key**.
  > Your token is stored in the browser only never server-side.

  Buildings, Smooth Roads and Include Trees all read the same Mapbox Streets v8 vector tiles (buildings, roads and water). The tiles are downloaded once per generation. Smooth Roads and Include Trees are on by default, so a default generation requests them even with buildings off. Turn all three off to skip them.

- **Python 3.12 or newer** with the `venv` module. On Ubuntu/Debian, `venv` comes in a separate package:

  ```bash
  sudo apt install python3-venv
  ```

- **CMake and the Gazebo Harmonic development packages** (optional) — only needed for the **Ground Surface** setting. See [Ground friction](#ground-friction).


## 🛠️ Setup

Clone the repo, create a virtual environment, install the dependencies, and start the server:

```bash
git clone https://github.com/your-org/gazebo_terrain_generator.git
cd gazebo_terrain_generator
python3 -m venv .venv
source .venv/bin/activate
pip install $(python -c "import tomllib; print(*tomllib.load(open('pyproject.toml', 'rb'))['project']['dependencies'], sep='\n')")
python scripts/server.py
```

The `pip install` line reads the dependency list from `pyproject.toml` and installs it.

On Windows (PowerShell), use `python` instead of `python3` and activate with `.venv\Scripts\Activate.ps1`; the other commands are the same.

Open [http://localhost:8080](http://localhost:8080) in your browser.

Each time you open a new terminal, activate the environment again (`source .venv/bin/activate`) before starting the server.


## 🚀 Generate a World

1. **Search** — type a place name or GPS coordinates (`lat, lng`)
2. **Draw** — use the polygon tool to outline your area on the map
3. **Set spawn** — drag the pin to where your robot should spawn. For more robots, add spawn points with the pin button in the map controls (optional)
4. **Configure** (optional) — open Settings to adjust zoom, tile source, buildings, helipad, ground surface, roads and trees
5. **Generate** — click **Generate Terrain** and give your world a name
6. **Download** — click **Download World** to get a `.zip`


## ⚙️ Settings

| Setting | Default | Description |
|---|---|---|
| Zoom Level | 17 | Satellite tile zoom — higher means more detail and more tiles |
| Include Buildings | On | Download OSM footprints and extrude as 3D meshes |
| Include Helipad | Off | Adds a helipad at the spawn location (at every spawn point, see Helipad at Every Spawn) |
| Helipad Height | 3.0 m | Height above ground (only relevant when helipad is enabled) |
| Map Tile Source | Google Maps Satellite | Tile provider URL template |
| Parallel Downloads | 4 | Concurrent tile download threads |
| Target Gazebo Version | Harmonic (and above) | Controls heightmap bit depth. Use **Fortress ** for Ignition 6 |
| Spawn Points | Red pin only | Extra spawn points (up to 32 in total, inside the polygon), each with a yaw in degrees counter-clockwise from east. Set on the map; not saved across page reloads. See [Multiple spawn points](#multiple-spawn-points) |
| Helipad at Every Spawn | On | Shown while Include Helipad is on. On: one helipad per spawn point. Off: only spawn point 1 gets a helipad |
| Ground Surface | Gazebo default | Friction preset for the terrain, buildings and tree trunks: Asphalt (μ 0.8), Concrete (0.7), Dirt / gravel (0.55), Sand (0.4), Grass (0.35), Snow (0.2), Ice (0.1) or Custom. Harmonic only, needs the [GroundFriction plugin](#ground-friction) |
| Friction Coefficient (mu) | 1 | Coulomb friction used in both directions (≥ 0). Editing it switches the surface to Custom. Each contact uses the lower of the ground's and the robot's μ, so the ground can't add grip the robot doesn't have |
| Smooth Roads | On | Flattens drivable roads across and smooths them along their length in the heightmap. See [Smooth roads](#smooth-roads) |
| Road Smoothing Length | 80 m | Bumps along a road shorter than this are removed; longer grade changes are kept. Use 120–200 m for mountain roads (10–1000) |
| Road Shoulder Width | 4 m | Width of the blend from the road edge back to the natural terrain. Wider gives gentler embankments and cuts (0–100) |
| Include Trees | On | Plants trees where the satellite texture shows tree canopy. See [Trees](#trees) |
| Tree Density | 150 per ha | Trees per hectare of detected canopy (1–1000) |
| Max Trees | 20000 | Upper limit on the number of trees (0–200000; 0 turns trees off). Above it, the trees nearest spawn point 1 keep full density and the rest of the forest is thinned |

### Output Path

Files are saved to `/tmp/gazebo_terrain_generator/{world_name}/` by default. Override with:

```bash
export GAZEBO_TERRAIN_OUTPUT_PATH=/your/custom/path
```

### Multiple spawn points

Spawn point 1 is the red pin and stays the world origin. To add more, press the pin button in the map controls (or **Settings → Add spawn point**) and click inside the polygon; Esc cancels. The new points are numbered blue markers: drag one to move it, right-click it (or use its trash button in Settings) to remove it. Settings lists every point with a yaw field.

Every spawn point becomes a world-level SDF frame named `spawn_1`, `spawn_2`, …, placed on the terrain (or on its helipad). Place a robot on one with `relative_to`, leaving a little clearance:

```xml
<include>
  <uri>model://my_robot</uri>
  <pose relative_to="spawn_2">0 0 0.2 0 0 0</pose>
</include>
```

`spawn_points.json` lists the same poses with GPS latitude, longitude and altitude. A world with only the red pin at yaw 0 is written exactly as before, without frames or JSON.

### Ground friction

Gazebo's default physics engine (DART) ignores `<surface><friction>` on heightmap and mesh collisions, so the **Ground Surface** setting is applied by a small Gazebo Harmonic system plugin that you build once on each machine that runs the world:

```bash
cmake -S gazebo_plugins/ground_friction -B build/ground_friction
cmake --build build/ground_friction
cmake --install build/ground_friction   # copies libGroundFriction.so to ~/.gz/sim/plugins
```

Requirements and details are in [gazebo_plugins/ground_friction/README.md](gazebo_plugins/ground_friction/README.md). Without the plugin the world still loads, Gazebo logs `Failed to load system plugin [GroundFriction]` and the ground keeps μ = 1; the generation console tells you whether the plugin was found. The helipads keep their own friction (μ = 1). Fortress worlds always keep the default friction.

### Smooth roads

The road centerlines come from the Mapbox Streets v8 road layer. Each road class gets a typical width (3.5 m for a track up to 14 m for a motorway). Before the heightmap is written, each road is flattened across to a smoothed centerline height and blended back into the terrain over the shoulder width; terrain farther from a road is left untouched. The heightmap, normal map, spawn heights, building bases and trees all use the smoothed terrain. Bridges, tunnels, ferries, rail and footpaths are not smoothed, and roads inside the polygon's bounding box are smoothed even outside the polygon. The console reports how much road was smoothed. If the road data can't be downloaded, the world is generated without smoothing. With Smooth Roads off, the output is identical to before.

### Trees

Tree canopy is detected in the satellite texture by colour and texture, so lawns and sports fields are not mistaken for forest. Trees are kept off building footprints, roads, water and the area outside the polygon. No trunk stands within 12 m of any spawn point, and crowns stay another 4 m clear. All trees are merged into one mesh. Only the trunks have collision, so drones can fly through crowns and cameras still see them. If tree generation fails, the world is still written without trees and the console says why. Detection was tuned on summer imagery: leafless, autumn, snow-covered or dry-season trees may be missed.


## 📁 Output Structure

```
{world_name}/
  {world_name}.world    — Gazebo world file with the terrain model inlined: heightmap, texture, buildings,
                          trees, helipads, spawn frames (relative URI, no GZ_SIM_RESOURCE_PATH needed)
  model.config          — model metadata (enables model:// usage in other worlds)
  spawn_points.json     — spawn point poses, GPS coordinates and helipads (only with extra spawn points or a spawn yaw)
  trees.csv             — one row per tree: x, y, z, yaw, height, crown radius, trunk radius, conifer flag
                          (when trees are on; e.g. for Blender)
  mesh/
    height_map.png      — 16-bit grayscale heightmap (8-bit for Fortress), with smoothed roads
    aerial.png          — stitched satellite texture
    normal_map.png      — normal map derived from heightmap
    buildings.dae       — 3D building mesh (present only when buildings are enabled and OSM data exists)
    trees.dae           — all trees in one mesh (when trees are on)
    trees_collision.dae — tree trunks only, used for collision
    vegetation_mask.png — 255 = tree canopy, 128 = other vegetation, same pixel grid as aerial.png
```

The download `.zip` contains all of these files.


## 🏁 Running the Generated World

<details>
<summary>Gazebo Harmonic (and above)</summary>

```bash
gz sim /tmp/gazebo_terrain_generator/{world_name}/{world_name}.world
```

</details>

<details>
<summary>Gazebo Fortress / Citadel</summary>

```bash
ign gazebo /tmp/gazebo_terrain_generator/{world_name}/{world_name}.world
```

</details>

Or unzip the downloaded archive and run with the local path.

## Sample Worlds Example

1. Apple Tech park
```bash
gz sim <path to gazbo_terrain_generator>/sample_worlds/applepark/applepark.world
```
2. Joshimath
```bash
gz sim <path to gazbo_terrain_generator>/sample_worlds/Joshimath/Joshimath.world
```

## Roadmap

Done:

- ~~Spawning multiple drones / robots (single helipad)~~ — [multiple spawn points](#multiple-spawn-points), each with an SDF frame and optionally its own helipad
- ~~Friction components to the ground~~ — **Ground Surface** setting, applied by the [GroundFriction plugin](#ground-friction) (Harmonic)
- ~~Roads are not smooth (explore Terminus)~~ — [Smooth Roads](#smooth-roads)
- ~~Generating forest based on ortho image vegetation masks (Blender plugin)~~ — [Trees](#trees), generated in Python without Blender; `trees.csv` and `vegetation_mask.png` can be reused in Blender

Next:

- Ground friction for Fortress, and per-surface friction (roads, grass, buildings) instead of one value for the whole terrain
- Bridges and tunnels: roads end at the abutment and no deck is modelled. Close parallel carriageways that are mapped as separate roads can keep a slight tilt
- Road surfaces that look and feel different (texture, markings, material); today only the heightmap changes
- Spawn points on rooftops (a spawn point on a building footprint ends up inside the mesh) and spawn points saved across page reloads
- Tree detection beyond summer imagery (autumn, snow, leafless or dry-season trees) and more realistic tree models; very large forests make big meshes, so keep **Max Trees** moderate
- Fortress worlds have only 255 height levels, which leaves visible steps on gently sloping roads
- More accurate placement on the heightmap: spawn points, buildings and trees can sit up to ~0.3 m off the collision surface on steep terrain with a coarse heightmap. Leave a small clearance when spawning robots
- GPS altitude with helipads: `<spherical_coordinates>` elevation includes the helipad height, so altitude reads high by that amount (kept as-is for compatibility)

## Contributors

Special thanks to [Karinca Robotics](https://karincarobotics.com) and [Polymath Robotics](https://polymathrobotics.com).

## License

BSD 3-Clause License. See [LICENSE](LICENSE) for details.
