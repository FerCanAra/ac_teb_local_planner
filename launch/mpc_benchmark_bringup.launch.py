# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Main launch for Benchmark control simulatoring                          |
# +-------------------------------------------------------------------------+

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
import yaml
import os
import tempfile

mvsimDir = get_package_share_directory("mvsim")
benchmarkDir = get_package_share_directory(
    "benchmark_bringup"
)  # original (maps, c1/c3 files, world assets)
acTebDir = get_package_share_directory(
    "ac_teb_local_planner"
)  # AC-TEB (world, params, rviz, c2 config)

# ---------------------------------------------------------------------------
#                            FIX PATHS TO FILES
# ---------------------------------------------------------------------------
MVSIM_ROS2_PARAMS_FILE = os.path.join(
    mvsimDir, "mvsim_tutorial", "mvsim_ros2_params.yaml"
)  # mvsim parameters file
MVSIM_WORLD_FILE = os.path.join(
    acTebDir, "definitions/world", "greenhouse_control.world.xml"
)  # default world file
MAP_FILE = os.path.join(
    benchmarkDir, "maps", "greenhouse_benchmark.yaml"
)  # Default slam map file
RVIZ2_FILE_C2 = os.path.join(
    acTebDir, "rviz", "c2_greenhouse.rviz"
)  # Default rviz2 config file for category 2
RVIZ2_FILE_C3 = os.path.join(
    benchmarkDir, "rviz", "c3_greenhouse.rviz"
)  # Default rviz2 config file for category 3

PID_MPC_PARAMS = os.path.join(
    benchmarkDir, "control_params", "c1_pid_params.yaml"
)  # c1 parameters file

TEB_MPC_PARAMS = os.path.join(
    benchmarkDir, "control_params", "c2_mpc_teb_params.yaml"
)  # c2 parameters file
THETA_STAR_PARAMS = os.path.join(
    benchmarkDir, "control_params", "c3_theta_star_params.yaml"
)  # c2 and c3 parameters file
NAV2_PARAMS_C2 = os.path.join(
    acTebDir, "config", "c2_nav2_config.yaml"
)  # c2 nav2 config file (bt_navigator, map_server, costmaps, ...)
NAV2_PARAMS_C3 = os.path.join(
    benchmarkDir, "configuration", "c3_nav2_config.yaml"
)  # c3 nav2 config file

# [AC-TEB] AC-TEB parameters (same content as c2_mpc_teb_params.yaml,
# with plugin: "ac_teb_local_planner::AcTebLocalPlannerROS" and the aisle: block)
AC_TEB_PARAMS = os.path.join(acTebDir, "params", "c2_mpc_ac_teb_params.yaml")

# [AC-TEB] Category 2 controllers for the comparison (launch arg controller:=...).
# All of them use the same camera-only costmaps of c2_nav2_config.yaml; only the
# FollowPath block of the controller_server changes.
#   ac_teb : AC-TEB (aisle centring on)
#   teb    : same plugin and tuning with aisle.enabled = False (= original TEB)
#   mppi, dwb, rpp : Nav2 Humble controllers with the same speed limits
CONTROLLER_PARAMS = {
    "ac_teb": AC_TEB_PARAMS,
    "teb": AC_TEB_PARAMS,
    "mppi": os.path.join(acTebDir, "params", "c2_mppi_params.yaml"),
    "dwb": os.path.join(acTebDir, "params", "c2_dwb_params.yaml"),
    "rpp": os.path.join(acTebDir, "params", "c2_rpp_params.yaml"),
}

# [AC-TEB] Base ground segmentation (Intel pointcloud_groundfloor_segmentation).
# MVSim publishes the RGB-D camera only as an unorganized point cloud (/camera1_points).
# mvsim_depth_bridge re-projects it into a RealSense-like depth image + camera_info:
#   /camera1/depth/image_rect_raw, /camera1/depth/camera_info
# which is exactly the input of the Intel node (sensor.name = "camera1").
# Outputs: /segmentation/labeled_points (label: 3 ground, 5 obstacle) and
#          /segmentation/obstacle_points, both in base_link.
GROUND_SEG_PACKAGE = "pointcloud_groundfloor_segmentation"
GROUND_SEG_EXECUTABLE = "pointcloud_groundfloor_segmentation"
CAMERA_NAME = "camera1"  # sensor_name of the rgbd_camera in the MVSim world
GROUND_SEG_PARAMS = {
    "base_frame": "base_link",
    "use_best_effort_qos": False,
    "sensor.name": CAMERA_NAME,
    "sensor.max_surface_height": 0.12,  # GreenSeg paper, Table 2
    "sensor.min_distance_to_ego": 0.3,
    "sensor.max_incline": 30.0,
    "sensor.robot_height": 0.5,
}


def generate_launch_description():
    # ---------------------------------------------------------------------------
    #                            DECLARE ARGUMENTS
    # ---------------------------------------------------------------------------

    # mvsim arguments
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file",
        default_value=TextSubstitution(text=MVSIM_WORLD_FILE),
        description="Path to the *.world.xml file to load",
    )
    headless_launch_arg = DeclareLaunchArgument("headless", default_value="False")
    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization", default_value="True"
    )
    publish_tf_odom2baselink_arg = DeclareLaunchArgument(
        "publish_tf_odom2baselink", default_value="True"
    )
    force_publish_vehicle_namespace_arg = DeclareLaunchArgument(
        "force_publish_vehicle_namespace", default_value="False"
    )
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="True")
    use_nav2_arg = DeclareLaunchArgument("use_nav2", default_value="True")
    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file", default_value=RVIZ2_FILE_C2
    )
    # Nav2 arguments
    declare_slam_launch_arg = DeclareLaunchArgument(
        "slam", default_value="False", description="Default value for SLAM"
    )
    declare_map_launch_arg = DeclareLaunchArgument(
        "map", default_value=MAP_FILE, description="Full path to map yaml file to load"
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (MVSim) clock if true",
    )
    declare_autostart_cmd = DeclareLaunchArgument("autostart", default_value="True")
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="False"
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn", default_value="False"
    )
    # Params configuration files
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=NAV2_PARAMS_C3,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_params_file_teb = DeclareLaunchArgument(
        "params_file_teb",
        default_value=TEB_MPC_PARAMS,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_params_file_theta = DeclareLaunchArgument(
        "params_file_theta",
        default_value=THETA_STAR_PARAMS,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    # Arguments for benchmark
    declare_category_arg = DeclareLaunchArgument("category", default_value="1")
    declare_payload_arg = DeclareLaunchArgument("payload", default_value="0")
    declare_terrain_slope_arg = DeclareLaunchArgument(
        "terrain_slope", default_value="false"
    )
    declare_change_terrain_arg = DeclareLaunchArgument(
        "change_terrain", default_value="false"
    )
    # [AC-TEB] perception
    declare_use_ground_seg_arg = DeclareLaunchArgument(
        "use_ground_segmentation",
        default_value="True",
        description="Launch the base ground segmentation (camera obstacles for the local costmap)",
    )

    # [AC-TEB] controller under test and ECC logger
    declare_controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="ac_teb",
        description="Category 2 controller: ac_teb | teb | mppi | dwb | rpp",
    )
    declare_ecc_logger_arg = DeclareLaunchArgument(
        "ecc_logger", default_value="True", description="Record the run (ecc_logger.py)"
    )
    declare_tag_arg = DeclareLaunchArgument(
        "tag",
        default_value="",
        description="Label of the configuration in the ECC folder name",
    )
    declare_results_dir_arg = DeclareLaunchArgument(
        "results_dir",
        default_value=os.path.expanduser("~/ECC"),
        description="One subfolder <date_time>_<controller> per run",
    )

    # ---------------------------------------------------------------------------
    #                            VALIDATE ARGUMENTS
    # ---------------------------------------------------------------------------
    def validate_args(context):
        category = int(LaunchConfiguration("category").perform(context))
        payload = float(LaunchConfiguration("payload").perform(context))
        terrain_slope = LaunchConfiguration("terrain_slope").perform(context).lower()
        change_terrain = LaunchConfiguration("change_terrain").perform(context).lower()

        # Validate category
        if category not in (1, 2, 3):
            raise ValueError(
                f"[ERROR] category must be 1, 2, or 3. Received: {category}"
            )

        # Validate payload
        if not (0 <= payload <= 70):
            raise ValueError(
                f"[ERROR] payload must be between 0 and 70. Received: {payload}"
            )

        # Validate booleans
        if terrain_slope not in ("true", "false"):
            raise ValueError(
                f"[ERROR] terrain_slope must be true or false. Received: {terrain_slope}"
            )
        if change_terrain not in ("true", "false"):
            raise ValueError(
                f"[ERROR] change_terrain must be true or false. Received: {change_terrain}"
            )

        print("[OK] All custom parameters validated successfully.")

    validate_action = OpaqueFunction(function=validate_args)

    # ---------------------------------------------------------------------------
    #                         SELECTION CATEGORY NODES
    # ---------------------------------------------------------------------------
    def select_nodes(context, *args, **kwargs):
        actions = []

        category = int(LaunchConfiguration("category").perform(context))
        terrain_slope = LaunchConfiguration("terrain_slope").perform(context).lower()
        change_terrain = LaunchConfiguration("change_terrain").perform(context).lower()

        if terrain_slope == "true" and change_terrain == "true":
            actions.append(SetLaunchConfiguration("min_terrain", "-0.3"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.3"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "true"))

        elif terrain_slope == "false" and change_terrain == "true":
            actions.append(SetLaunchConfiguration("min_terrain", "-0.00001"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.00001"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "true"))

        elif terrain_slope == "true" and change_terrain == "false":
            actions.append(SetLaunchConfiguration("min_terrain", "-0.3"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.3"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "false"))

        else:  # terrain_slope == "false" and change_terrain == "false"
            actions.append(SetLaunchConfiguration("min_terrain", "-0.00001"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.00001"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "false"))

        #############################
        # CATEGORY 1
        #############################
        if category == 1:
            actions.append(SetLaunchConfiguration(name="use_rviz", value="False"))
            actions.append(SetLaunchConfiguration(name="use_nav2", value="False"))
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="velocity_cat1",
                    name="velocity_cat1",
                    output="screen",
                )
            )
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="plotter_c1_benchmark",
                    name="plotter_c1_benchmark",
                    output="screen",
                )
            )

        #############################
        # CATEGORY 2
        #############################
        # elif category == 2:
        #     # actions.append(SetLaunchConfiguration("map", ""))  # No map for category 2
        #     # [AC-TEB] route_cat2 sends the goals to Nav2: without it the robot never moves
        #     actions.append(
        #         Node(
        #             package="ac_teb_local_planner",
        #             executable="route_cat2.py",
        #             name="route_cat2",
        #             output="screen",
        #         )
        #     )

        #############################
        # CATEGORY 3
        #############################
        elif category == 3:
            actions.append(SetLaunchConfiguration("rviz_config_file", RVIZ2_FILE_C3))
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="route_cat3",
                    name="route_cat3",
                    output="screen",
                )
            )
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="plotter_c3_benchmark",
                    name="plotter_c3_benchmark",
                    output="screen",
                )
            )

        return actions

    selected_nodes = OpaqueFunction(function=select_nodes)

    # -------------------------------------------------------------------------
    # Nav2 param rewrite
    # -------------------------------------------------------------------------
    def generate_nav2_params(context):
        c = int(LaunchConfiguration("category").perform(context))
        rewrites = {}

        # [AC-TEB] nested dictionaries (footprint_model, aisle, ...) are
        # flattened to dotted keys: FollowPath.aisle.enabled, ...
        # (str(dict) would pass the whole dictionary as a single string)
        def flatten_params(prefix, d, out):
            for k, v in d.items():
                key = f"{prefix}.{k}"
                if isinstance(v, dict):
                    flatten_params(key, v, out)
                else:
                    out[key] = str(v)

        if c == 2:
            # [AC-TEB] the FollowPath block of the selected controller replaces
            # the one of c2_nav2_config.yaml entirely (lists such as the MPPI/DWB
            # critics cannot go through RewrittenYaml string rewrites).
            ctrl = LaunchConfiguration("controller").perform(context).lower()
            if ctrl not in CONTROLLER_PARAMS:
                raise ValueError(
                    f"[ERROR] controller must be one of {list(CONTROLLER_PARAMS)}. Received: {ctrl}"
                )
            with open(NAV2_PARAMS_C2) as f:
                nav2 = yaml.safe_load(f)
            with open(CONTROLLER_PARAMS[ctrl]) as f:
                follow = yaml.safe_load(f)["controller_server"]["ros__parameters"][
                    "FollowPath"
                ]
            if ctrl == "teb":
                follow.setdefault("aisle", {})["enabled"] = False
            nav2["controller_server"]["ros__parameters"]["FollowPath"] = follow
            tmp = tempfile.NamedTemporaryFile(
                "w", prefix=f"nav2_c2_{ctrl}_", suffix=".yaml", delete=False
            )
            yaml.safe_dump(nav2, tmp, default_flow_style=False, sort_keys=False)
            tmp.close()
            print(
                f"[OK] Category 2 controller: {ctrl} ({CONTROLLER_PARAMS[ctrl]}) -> {tmp.name}"
            )
            return [SetLaunchConfiguration("nav2_params", tmp.name)]

        elif c == 3:
            base_yaml = NAV2_PARAMS_C3
            with open(THETA_STAR_PARAMS) as f:
                data = yaml.safe_load(f)

            flatten_params(
                "controller_server.ros__parameters.FollowPath",
                data["controller_server"]["ros__parameters"]["FollowPath"],
                rewrites,
            )
            flatten_params(
                "planner_server.ros__parameters.GridBased",
                data["planner_server"]["ros__parameters"]["GridBased"],
                rewrites,
            )

        else:
            return [SetLaunchConfiguration("nav2_params", NAV2_PARAMS_C3)]

        rewritten = RewrittenYaml(
            source_file=base_yaml,
            param_rewrites=rewrites,
            convert_types=True,
        )

        return [SetLaunchConfiguration("nav2_params", rewritten)]

    nav2_param_rewrite = OpaqueFunction(function=generate_nav2_params)

    # ---------------------------------------------------------------------------
    #                                ROS2 NODES
    # ---------------------------------------------------------------------------
    # mvsim node
    mvsim_node = Node(
        package="mvsim",
        executable="mvsim_node",
        name="mvsim",
        output="screen",
        parameters=[
            MVSIM_ROS2_PARAMS_FILE,
            {
                "world_file": LaunchConfiguration("world_file"),
                "headless": LaunchConfiguration("headless"),
                # [AC-TEB] category 2: map->odom comes from the static TF below
                # (MVSim's dynamic one arrives late under camera load)
                "do_fake_localization": PythonExpression(
                    [
                        "'False' if '",
                        LaunchConfiguration("category"),
                        "' == '2' else '",
                        LaunchConfiguration("do_fake_localization"),
                        "'",
                    ]
                ),
                "publish_tf_odom2baselink": LaunchConfiguration(
                    "publish_tf_odom2baselink"
                ),
                "force_publish_vehicle_namespace": LaunchConfiguration(
                    "force_publish_vehicle_namespace"
                ),
            },
        ],
    )

    # [AC-TEB] map -> odom (identity, MVSim odometry = ground truth) as a
    # static TF: never extrapolates, so the controller can always transform the
    # global plan into odom.
    map_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_static_tf",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("category"), "' == '2'"])
        ),
    )

    # RViz2 node
    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    # acml init pose publisher, necesary for Nav2 localization
    init_pose_node = Node(
        package="benchmark_bringup",
        executable="odom_mvsim2teb",
        name="init_pose_publisher",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("category"), "' != '1'"])
        ),
    )

    # [AC-TEB] Perception chain (only with Nav2, i.e. categories 2 and 3):
    #   MVSim /camera1_points -> mvsim_depth_bridge -> /camera1/depth/{image_rect_raw,camera_info}
    #   -> Intel ground segmentation -> /segmentation/{labeled_points,obstacle_points}
    perception_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("use_ground_segmentation"),
                "'.lower() == 'true' and '",
                LaunchConfiguration("category"),
                "' != '1'",
            ]
        )
    )

    depth_bridge_node = Node(
        package="ac_teb_local_planner",
        executable="mvsim_depth_bridge.py",
        name="mvsim_depth_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": f"/{CAMERA_NAME}_points",
                "camera_name": CAMERA_NAME,
                # rgbd_camera.sensor.xml (640x480, f=400) at half resolution,
                # same as spatial_downsample of depth_filter on the real robot:
                # 4x fewer points for segmentation, costmaps and RViz
                "width": 320,
                "height": 240,
                "fx": 200.0,
                "fy": 200.0,
                "cx": 160.0,
                "cy": 120.0,
                "min_depth": 0.3,
                "max_depth": 3.0,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        condition=perception_condition,
    )

    ground_seg_node = Node(
        package=GROUND_SEG_PACKAGE,
        executable=GROUND_SEG_EXECUTABLE,
        name="groundfloor_segmentation",
        output="screen",
        parameters=[
            GROUND_SEG_PARAMS,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=perception_condition,
    )

    # [AC-TEB] ECC logger: ~/ECC/<date_time>_<controller>/{timeseries,summary}.csv
    ecc_logger_node = Node(
        package="ac_teb_local_planner",
        executable="ecc_logger.py",
        name="ecc_logger",
        output="screen",
        parameters=[
            {
                "results_dir": LaunchConfiguration("results_dir"),
                "controller": LaunchConfiguration("controller"),
                "tag": LaunchConfiguration("tag"),
                "robot_radius": 0.36,  # paper r_robot (footprint of the clearance metric)
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("ecc_logger"),
                    "'.lower() == 'true' and '",
                    LaunchConfiguration("category"),
                    "' == '2'",
                ]
            )
        ),
    )

    # Nav2 bringup
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
        launch_arguments={
            "slam": LaunchConfiguration("slam"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("nav2_params"),
            "map": LaunchConfiguration("map"),
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": LaunchConfiguration("use_composition"),
            "use_respawn": LaunchConfiguration("use_respawn"),
        }.items(),
    )

    # ---------------------------------------------------------------------------
    #                         LAUNCH DESCRIPTION
    # ---------------------------------------------------------------------------
    ld = LaunchDescription()

    # Add common arguments
    ld.add_action(world_file_launch_arg)
    ld.add_action(headless_launch_arg)
    ld.add_action(do_fake_localization_arg)
    ld.add_action(publish_tf_odom2baselink_arg)
    ld.add_action(force_publish_vehicle_namespace_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_nav2_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(declare_category_arg)
    ld.add_action(declare_payload_arg)
    ld.add_action(declare_terrain_slope_arg)
    ld.add_action(declare_change_terrain_arg)
    ld.add_action(declare_use_ground_seg_arg)  # [AC-TEB]
    ld.add_action(declare_controller_arg)  # [AC-TEB]
    ld.add_action(declare_ecc_logger_arg)  # [AC-TEB]
    ld.add_action(declare_results_dir_arg)  # [AC-TEB]
    ld.add_action(declare_tag_arg)  # [AC-TEB]
    ld.add_action(validate_action)
    ld.add_action(selected_nodes)

    # Add Nav2 arguments
    ld.add_action(declare_map_launch_arg)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_params_file_teb)
    ld.add_action(declare_params_file_theta)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_slam_launch_arg)

    # ===============================
    # Load PID parameters from YAML
    # ===============================
    with open(PID_MPC_PARAMS, "r") as f:
        params = yaml.safe_load(f)

    # Aplanar diccionario (PID.KP → CTRL_PID_KP)
    def flatten(prefix, d, out):
        for k, v in d.items():
            key = f"{prefix}_{k}".upper()
            if isinstance(v, dict):
                flatten(key, v, out)
            else:
                out[key] = v

    flat_params = {}
    flatten("CTRL", params, flat_params)

    # 1) Declarar argumentos LaunchConfiguration para todos
    for key, val in flat_params.items():
        ld.add_action(DeclareLaunchArgument(key, default_value=str(val)))

    # 2) Crear variables de entorno a partir de esos argumentos
    for key in flat_params.keys():
        env_name = key.replace("CTRL_PID_", "")
        ld.add_action(
            SetEnvironmentVariable(name=env_name, value=LaunchConfiguration(key))
        )

    # Set benchmark arguments as environment variables
    ld.add_action(
        SetEnvironmentVariable(
            name="PAYLOAD_MASS", value=LaunchConfiguration("payload")
        )
    )
    ld.add_action(
        SetEnvironmentVariable(name="MIN_ALT", value=LaunchConfiguration("min_terrain"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="MAX_ALT", value=LaunchConfiguration("max_terrain"))
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="USE_ADAPTATIVE_FRICTION",
            value=LaunchConfiguration("adaptative_friction"),
        )
    )

    # Example of PID parameters usage:
    ld.add_action(
        SetEnvironmentVariable(name="KP", value=LaunchConfiguration("CTRL_PID_KP"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="KI", value=LaunchConfiguration("CTRL_PID_KI"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="KD", value=LaunchConfiguration("CTRL_PID_KD"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="N", value=LaunchConfiguration("CTRL_PID_N"))
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="ENABLE_ANTIWINDUP",
            value=LaunchConfiguration("CTRL_PID_ENABLE_ANTIWINDUP"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="ENABLE_FEEDFORWARD",
            value=LaunchConfiguration("CTRL_FEEDFORWARD_ENABLE_FEEDFORWARD"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="K_FF", value=LaunchConfiguration("CTRL_FEEDFORWARD_K_FF")
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="TAU_FF", value=LaunchConfiguration("CTRL_FEEDFORWARD_TAU_FF")
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="ENABLE_REFERENCEFILTER",
            value=LaunchConfiguration("CTRL_REFERENCEFILTER_ENABLE_REFERENCEFILTER"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="TAU_F", value=LaunchConfiguration("CTRL_REFERENCEFILTER_TAU_F")
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="N_F", value=LaunchConfiguration("CTRL_REFERENCEFILTER_N_F")
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="MAX_TORQUE", value=LaunchConfiguration("CTRL_MOTORSIM_MAX_TORQUE")
        )
    )

    # Add nodes
    ld.add_action(nav2_param_rewrite)
    ld.add_action(mvsim_node)
    ld.add_action(map_odom_tf_node)  # [AC-TEB]
    ld.add_action(init_pose_node)
    ld.add_action(depth_bridge_node)  # [AC-TEB]
    ld.add_action(ground_seg_node)  # [AC-TEB]
    ld.add_action(ecc_logger_node)  # [AC-TEB]
    ld.add_action(rviz2_node)
    ld.add_action(bringup_cmd)

    return ld
