import math
import os
from utils.param import GlobalParam
from utils.spawn_points import helipad_include, frames_block, write_spawn_points_json


class FileWriter:

    @staticmethod
    def read_template(template_file_name):
        with open(template_file_name, "r") as template_file:
            return str(template_file.read())

    @staticmethod
    def write_world_file(sdf_template, model_name,
                         size_x, size_y, size_z, pose_x, pose_y, pose_z,
                         launch_lat, launch_lon, origin_elevation,
                         include_buildings, output_dir,
                         texture_size=None, include_helipad=False, helipad_height=5.0,
                         friction_coefficient=None, spawn_poses=None, include_trees=False):
        '''
        Write model.config and {model_name}.world into output_dir.

        The terrain model (heightmap, buildings, helipad) is inlined directly
        into the world file — no separate model.sdf is produced.

        Args:
            sdf_template (str): Content of the Gazebo world template.
            model_name (str): Name of the world/model.
            size_x, size_y, size_z (float): Terrain dimensions in meters.
            pose_x, pose_y, pose_z (float): Terrain offset from launch point in meters.
            launch_lat, launch_lon (float): Launch location GPS coordinates.
            origin_elevation (float): Launch location elevation in meters AMSL.
            include_buildings (bool): Whether to include the buildings model in the world.
            output_dir (str): Root model directory (flat — all assets live here).
            texture_size (float|None): UV size for aerial texture; defaults to max(size_x, size_y).
            include_helipad (bool): Whether to include a helipad at world origin.
            friction_coefficient (float|None): Ground friction coefficient (mu) applied by the
                GroundFriction system plugin; None keeps Gazebo's default friction.
            spawn_poses (list|None): Multi-spawn poses from GazeboTerrainGenerator.get_spawn_poses().
                When given, one helipad is placed per pose that has one (instead of the single
                helipad at the origin), each pose becomes a world-level <frame>, and
                spawn_points.json is written. None keeps the single-spawn output.
            include_trees (bool): Whether to include the procedural forest (mesh/trees.dae,
                written by ForestGenerator) in the world.
        '''
        texture_size_val = texture_size if texture_size is not None else max(size_x, size_y)
        camera_z = round(size_z + pose_z + 200, 1)

        # --- Buildings block ---
        dae_file = os.path.join(output_dir, 'mesh', 'buildings.dae')
        if include_buildings and os.path.isfile(dae_file):
            building_template = FileWriter.read_template(
                os.path.join(GlobalParam.TEMPLATE_DIR_PATH, 'building_template.sdf')
            )
            buildings_sdf_block = (building_template
                .replace("$MODELNAME$", model_name)
                .replace("$POSX$", str(pose_x))
                .replace("$POSY$", str(pose_y)))
        else:
            buildings_sdf_block = ""

        # Elevation of the world origin written to <spherical_coordinates>. A helipad
        # at the origin raises it by helipad_height (unchanged single-spawn behaviour).
        world_origin_elevation = origin_elevation + (helipad_height if include_helipad else 0)

        # --- Helipad block ---
        if spawn_poses:
            # Multi-spawn: one helipad per spawn point that has one (primary keeps its name)
            helipad_block = '\n'.join(
                helipad_include(p["helipad"], p["x"], p["y"], p["z"], p["yaw"])
                for p in spawn_poses if p.get("helipad")
            )
        elif include_helipad:
            helipad_block = (
                f'        <include>\n'
                f'            <name>{model_name}_helipad</name>\n'
                f'            <uri>https://fuel.gazebosim.org/1.0/saiaravind19/models/helipad</uri>\n'
                f'            <pose>0 0 {helipad_height} 0 0 0</pose>\n'
                f'            <static>true</static>\n'
                f'        </include>'
            )
        else:
            helipad_block = ""

        # --- Ground friction block ---
        friction_block = FileWriter.ground_friction_block(friction_coefficient)

        # --- Trees block (forest from the vegetation mask) ---
        trees_dae = os.path.join(output_dir, 'mesh', 'trees.dae')
        trees_collision_dae = os.path.join(output_dir, 'mesh', 'trees_collision.dae')
        if include_trees and os.path.isfile(trees_dae) and os.path.isfile(trees_collision_dae):
            trees_sdf_block = (FileWriter.read_template(
                os.path.join(GlobalParam.TEMPLATE_DIR_PATH, 'trees_template.sdf'))
                .replace("$MODELNAME$", model_name))
        else:
            trees_sdf_block = ""

        os.makedirs(output_dir, exist_ok=True)

        # --- Spawn frames + spawn_points.json (multi-spawn) ---
        spawn_frames_block = frames_block(spawn_poses) if spawn_poses else ""
        # Drop the marker's whole line when empty so the single-spawn world is unchanged
        sdf_template = (sdf_template
            .replace("$SPAWN_FRAMES$\n", spawn_frames_block + "\n" if spawn_frames_block else "")
            .replace("$SPAWN_FRAMES$", spawn_frames_block))
        spawn_json = os.path.join(output_dir, 'spawn_points.json')
        if spawn_poses:
            write_spawn_points_json(output_dir, model_name, spawn_poses, launch_lat, launch_lon,
                                    world_origin_elevation, include_helipad, helipad_height)
        elif os.path.isfile(spawn_json):
            os.remove(spawn_json)

        # --- Write model.config ---
        model_config = (FileWriter.read_template(
            os.path.join(GlobalParam.TEMPLATE_DIR_PATH, 'model_config_template.xml'))
            .replace("$MODELNAME$", model_name))
        with open(os.path.join(output_dir, "model.config"), "w") as f:
            f.write(model_config)

        # --- Write world file (terrain model inlined) ---
        # Drop the line of an empty optional block (and the blank line after $TREES$), so a
        # world without ground friction or trees is written exactly as before those features
        if not friction_block:
            sdf_template = sdf_template.replace("$GROUND_FRICTION$\n", "")
        if not trees_sdf_block:
            sdf_template = sdf_template.replace("$TREES$\n\n", "")
        sdf_template = (sdf_template
            .replace("$MODELNAME$", model_name)
            .replace("$SIZEX$", str(size_x))
            .replace("$SIZEY$", str(size_y))
            .replace("$SIZEZ$", str(size_z))
            .replace("$POSX$", str(pose_x))
            .replace("$POSY$", str(pose_y))
            .replace("$POSZ$", str(pose_z))
            .replace("$TEXTURE_SIZE$", str(texture_size_val))
            .replace("$BUILDING$", buildings_sdf_block)
            .replace("$TREES$", trees_sdf_block)
            .replace("$HELIPAD$", helipad_block)
            .replace("$GROUND_FRICTION$", friction_block)
            .replace("$ORIGIN_LAT$", str(launch_lat))
            .replace("$ORIGIN_LONG$", str(launch_lon))
            .replace("$ORIGIN_ELEVATION$", str(world_origin_elevation))
            .replace("$CAMERA_Z$", str(camera_z)))
        with open(os.path.join(output_dir, model_name + ".world"), "w") as f:
            f.write(sdf_template)

    # --- Ground friction (roadmap: "Friction components to the ground") ---
    # gz-sim's DART physics ignores <surface><friction> on heightmap and mesh
    # collisions, so the coefficient is applied by a small system plugin
    # (gazebo_plugins/ground_friction) attached to the terrain model.
    GROUND_FRICTION_PLUGIN_FILE = "GroundFriction"
    GROUND_FRICTION_PLUGIN_NAME = "gazebo_terrain_generator::GroundFriction"

    @staticmethod
    def format_friction(friction_coefficient):
        '''
        Shortest text that reads back as exactly the same coefficient (0.1, 0.1234567,
        1e-05), unlike '%g', which rounds to 6 significant digits. -0 is written as 0.0.
        '''
        return repr(float(friction_coefficient) + 0.0)

    @staticmethod
    def ground_friction_block(friction_coefficient):
        '''
        SDF that sets the friction of the collisions in the terrain model (heightmap,
        buildings and anything else nested in it). Collisions that declare their own
        <surface><friction><ode><mu>, such as the Fuel helipad, keep it.

        Args:
            friction_coefficient (float|None): Coulomb friction coefficient (mu), used for
                both friction directions. None returns "" so the world keeps Gazebo's default.
        '''
        if friction_coefficient is None:
            return ""
        mu = float(friction_coefficient)
        if not math.isfinite(mu) or mu < 0:
            raise ValueError(f"Friction coefficient must be a finite number >= 0, got {friction_coefficient!r}")
        mu_text = FileWriter.format_friction(mu)
        return (
            f'            <!-- Ground friction (mu = {mu_text}). Gazebo\'s DART physics ignores <surface><friction>\n'
            f'                 on heightmap and mesh collisions, so the GroundFriction system plugin applies it\n'
            f'                 to the terrain, buildings and tree trunks of this model; collisions that set their\n'
            f'                 own friction (e.g. the helipad) keep it. Each contact uses the lower of the two mu.\n'
            f'                 Build it once from gazebo_plugins/ground_friction in gazebo_terrain_generator.\n'
            f'                 Without it Gazebo logs "Failed to load system plugin" and keeps mu = 1. -->\n'
            f'            <plugin filename="{FileWriter.GROUND_FRICTION_PLUGIN_FILE}" name="{FileWriter.GROUND_FRICTION_PLUGIN_NAME}">\n'
            f'                <mu>{mu_text}</mu>\n'
            f'                <mu2>{mu_text}</mu2>\n'
            f'            </plugin>'
        )
