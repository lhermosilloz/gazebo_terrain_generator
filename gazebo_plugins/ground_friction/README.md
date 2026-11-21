# GroundFriction system plugin

Generated worlds use this Gazebo Harmonic (gz-sim 8) system when **Settings → Ground Surface** is set to anything other than *Gazebo default*. It applies the chosen friction coefficient to the terrain, the buildings and anything else nested in the terrain model.

## Why a plugin is needed

Gazebo's default physics engine, DART, ignores `<surface><friction>` on heightmap and mesh collisions. gz-sim adds those shapes through `AttachHeightmapShape` / `AttachMeshShape`, which never read the SDF surface, so the terrain always has DART's default μ = 1. For each contact DART uses the lower of the two μ values, so a robot touching the terrain always gets min(robot μ, 1).

This plugin sets the contact's friction through gz-sim's contact-surface customization event, the same hook the TrackController system uses.

## Build and install (once per machine)

You need CMake, a C++17 compiler and the gz-sim 8 development files (`libgz-sim8-dev`, `libgz-physics7-dev`, `libgz-plugin2-dev`; the `gz-harmonic` apt metapackage installs them). The ROS 2 Jazzy `ros-jazzy-gz-*-vendor` packages also include them. Run these commands from the repository root:

```bash
cmake -S gazebo_plugins/ground_friction -B build/ground_friction
cmake --build build/ground_friction
cmake --install build/ground_friction   # copies libGroundFriction.so to ~/.gz/sim/plugins
```

gz-sim searches `~/.gz/sim/plugins` by default, so you don't need to set anything else. To use the build directory without installing, run:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/ground_friction
```

If the world asks for the plugin and Gazebo can't find it, Gazebo logs `Failed to load system plugin [GroundFriction]`. The world still loads, with the default friction.

## SDF

```xml
<model name="terrain">
  <plugin filename="GroundFriction" name="gazebo_terrain_generator::GroundFriction">
    <mu>0.4</mu>    <!-- required, >= 0 -->
    <mu2>0.4</mu2>  <!-- optional, defaults to mu -->
  </plugin>
  ...
</model>
```

- Collisions in the model, including nested models, use `<mu>`/`<mu2>`. A collision that sets its own `<surface><friction><ode><mu>` keeps that value. The Fuel helipad does this, with μ = 1.
- Every other collision uses its own SDF `<ode><mu>`/`<mu2>`, or 1 if it sets none.
- Each contact uses the lower value in each direction, which is DART's own rule. Contacts that don't touch this model are not changed.

## Limitations

- The plugin is built for Gazebo Harmonic (gz-sim 8) only. Fortress worlds keep the default friction.
- It has only been tested with the DART engine. The bullet-featherstone engine in gz-physics 7 can't collide with heightmaps at all.
