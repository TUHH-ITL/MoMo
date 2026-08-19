import os
from base_utils.launch.launch_utils import (
    find_pkg_share,
    launch_lifecycle_manager,
    launch_map_server,
    launch_mocap_driver,
    launch_nav2,
    launch_node,
    launch_ps4_teleop,
    launch_rviz,
)

from base_utils.node_info import NodeInfo
from localization_switcher.localization_switcher_config import (
    LocalizationSwitcherNodeConfig,
)
from localization_trigger.localization_trigger_node_config import (
    LocalizationTriggerNodeConfig,
)

from log_handling.log_handling_node_config import LogHandlingNodeConfig
from master_control.master_control_node_config import MasterControlNodeConfig
from mission_control.features.node_infos import (
    mission_control_node_info,
    http_server_node_info,
    control_circuit_node_info,
    cargo_storage_manager_node_info,
    delay_node_info,
    gentle_stop_node_info,
    localization_trigger_node_info,
    localization_watcher_node_info,
    log_handling_node_info,
    master_control_node_info,
    master_control_sync_node_info,
    motor_command_switch_node_info,
    move_commander_node_info,
    obstacle_approximation_node_info,
    order_conversion_node_info,
    order_management_node_info,
    rosbag_recorder_node_info,
    status_monitor_node_info,
    offset_reduction_node_info,
    costmap_clearing_node_info,
    node_info_db,
    qualisys_localization_node_info,
    enable_localization_switch_node_info,
)

from mission_control.mission_control_node_config import (
    MissionControlNodeConfig,
)
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python import get_package_share_path
from delay.delay_node_config import DelayNodeConfig
from gentle_stop.gentle_stop_node_config import GentleStopNodeConfig
from localization_watcher.localization_watcher_node_config import (
    LocalizationWatcherNodeConfig,
)
from motor_command_switch.motor_command_switch_node_config import (
    MotorCommandSwitchNodeConfig,
)
from move_commander.move_commander_node_config import MoveCommanderNodeConfig
from order_conversion.order_conversion_node_config import (
    OrderConversionNodeConfig,
)
from order_management.order_management_node_config import (
    OrderManagementNodeConfig,
)
from http_server.http_server_node_config import HttpServerNodeConfig
from master_control_sync.master_control_sync_node_config import (
    MasterControlSyncNodeConfig,
)
from obstacle_approximation.obstacle_approximation_node_config import (
    ObstacleApproximationNodeConfig,
)
from qualisys_localization.qualisys_driver_node_config import (
    QualisysDriverNodeConfig,
)
from qualisys_localization.qualisys_localization_node_config import (
    QualisysLocalizationNodeConfig,
)
from rosbag_recorder.rosbag_recorder_node_config import (
    RosbagRecorderNodeConfig,
)
from status_monitor.status_monitor_node_config import StatusMonitorNodeConfig
from master_control.control_circuit_node_config import ControlCircuitNodeConfig
from costmap_clearing.costmap_clearing_node_config import (
    CostmapClearingNodeConfig,
)


from cargo_storage_manager.cargo_storage_manager_node_config import (
    CargoStorageManagerNodeConfig,
)
from offset_reduction.offset_reduction_node_config import (
    OffsetReductionNodeConfig,
)
from teleop.teleop_node_config import TeleopNodeConfig


def generate_launch_description():
    robot_name = os.getenv("ROBOT_NAME", "MoMo")
    fallback_master_control_address = "1134.28.140.229:50051"
    dns_server_address = "134.28.108.75:8500"
    state_machine_file = os.path.join(
        find_pkg_share(mission_control_node_info),
        "state_machine_definitions",
        "no_master_control_mocap.py",
    )
    recordings_folder = f"{find_pkg_share(http_server_node_info)}/public"
    network_interface = "wlo1"

    map_file = os.path.join(
        get_package_share_path("map_handling"),
        "maps",
        "itl_mocap_aligned_new_clean.yaml",
    )

    # map_file = os.path.join(
    #     get_package_share_path("startup"),
    #     "maps",
    #     "map.yaml",
    # )

    factsheet_file_path = os.path.join(
        get_package_share_path("startup"),
        "config",
        "factsheet.py",
    )
    cargo_storage_manager_config_path = os.path.join(
        get_package_share_path("startup"),
        "config",
        "cargo_storage_manager.yaml",
    )
    logging_whitelist = list(node_info_db.keys())

    DEVICE_PORT_JOYSTICK = "/dev/input/js0"

    ### BEGIN TOPICS
    control_circuit_twist_topic = "control_circuit/cmd_vel"
    cargo_status_topic = "status/cargo"
    gentle_stop_twist_topic = "gentle_stop/cmd_vel"
    master_control_drive_command_topic = "master_control/drive_commands"
    nav_mode_topic = "nav_mode"
    nav_goal_topic = "nav_goal"
    platform_twist_topic = "cmd_vel_switched"
    autonomous_control_topic = "cmd_vel_smoothed"
    teleop_status_topic = "status/teleop"
    teleop_twist_topic = "cmd_vel_teleop"
    odom_topic = "odom"
    mocap_external_pose_topic = "itl_jackal_1/pose"
    ### END TOPICS

    # control_circuit_config = ControlCircuitNodeConfig(
    #     drive_command_output_topic=control_circuit_twist_topic, odometry_input_topic=odom_topic
    # )
    cargo_storage_manager_config = CargoStorageManagerNodeConfig(
        load_status_output_topic=cargo_status_topic,
        config_path=cargo_storage_manager_config_path,
    )
    gentle_stop_config = GentleStopNodeConfig(
        platform_cmd_vel_input_topic=platform_twist_topic,
        gentle_stop_cmd_vel_output_topic=gentle_stop_twist_topic,
    )
    http_server_config = HttpServerNodeConfig(
        network_interface=network_interface,
        path_to_public_folder=recordings_folder,
    )
    # localization_trigger_config = LocalizationTriggerNodeConfig(
    #     twist_input_topic=platform_twist_topic,
    # )
    log_handling_config = LogHandlingNodeConfig(log_gnss_position=False)
    # master_control_config = MasterControlNodeConfig(
    #     server_address=fallback_master_control_address,
    #     logging_whitelist=logging_whitelist,
    #     robot_name=robot_name,
    #     velocity_input_topic=platform_twist_topic,
    #     drive_command_output_topic=master_control_drive_command_topic,
    #     dns_server_address=dns_server_address,
    # )
    mission_control_config = MissionControlNodeConfig(
        state_machine_file=state_machine_file
    )
    motor_command_switch_config = MotorCommandSwitchNodeConfig(
        autonomous_control_input_topic=autonomous_control_topic,
        gentle_stop_input_topic=gentle_stop_twist_topic,
        remote_control_input_topic=teleop_twist_topic,
        cmd_vel_output_topic=platform_twist_topic,
    )
    move_commander_config = MoveCommanderNodeConfig(
        navigation_mode_output_topic=nav_mode_topic,
        navigation_goal_output_topic=nav_goal_topic,
    )
    obstacle_approximation_config = ObstacleApproximationNodeConfig(
        visualize_obstacles=True, obstacle_threshold=99
    )
    # order_management_config = OrderManagementNodeConfig(
    #     scan_profiles_service_name="get_terrestrial_laserscanning_profiles",
    #     factsheet_file_path=factsheet_file_path,
    #     velocity_input_topic=platform_twist_topic,
    #     nav_mode_input_topic=nav_mode_topic,
    #     amcl_pose_input_topic="amcl_pose",
    # )
    offset_reduction_config = OffsetReductionNodeConfig(
        omnidirectional_instead_of_differential=True
    )
    rosbag_recorder_config = RosbagRecorderNodeConfig(
        path_to_recordings=recordings_folder
    )
    rviz_config = os.path.join(
        get_package_share_path("startup"), "rviz", "mocap.rviz"
    )
    qualisys_localization_config = QualisysLocalizationNodeConfig(
        rigid_body_id="MoMo",
        robot_name=robot_name,
        qualisys_localization_input_topic="/mocap/rigid_bodies",
        publish_tf=True,
        parent_tf_frame="odom",
    )
    # teleop_config = TeleopNodeConfig(
    #     device=DEVICE_PORT_JOYSTICK,
    #     twist_output_topic=teleop_twist_topic,
    #     status_output_topic=teleop_status_topic,
    #     master_control_teleop_cmd_input_topic=master_control_drive_command_topic,
    #     key_index_hand_over_control=0,
    #     key_index_cancel=2,
    #     key_button_index_logitech_b_or_ps4_circle=1,
    #     key_button_index_logitech_x_or_ps4_square=3,
    # )
    teleop_config = TeleopNodeConfig(
        device=DEVICE_PORT_JOYSTICK,
        twist_output_topic=teleop_twist_topic,
        status_output_topic=teleop_status_topic,
        master_control_teleop_cmd_input_topic=master_control_drive_command_topic,
        motor_command_input_topic=platform_twist_topic,
        min_message_frequency_in_hz=10,
        joy_index_drive=1,
        joy_index_steer=3,
        joy_index_strafe=0,
        key_index_gear_up=5,
        key_index_gear_down=5,
        key_index_deadman=4,
        key_index_hand_over_control=0,
        key_index_cancel=3,
        key_index_pre_shutdown_1=6,
        key_index_pre_shutdown_2=7,
        key_index_shutdown_1=6,
        key_index_shutdown_2=7,
        max_linear_velocities_in_meters_per_second=[0.30, 0.7, 1.0],
        max_angular_velocities_in_meters_per_second=[0.3, 0.6, 0.8],
    )
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                namespace=robot_name,
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
            launch_map_server(map_file, robot_name),
            launch_mocap_driver(
                QualisysDriverNodeConfig(),
                namespace="mocap",
            ),
            launch_lifecycle_manager(
                ["map_server"],
                robot_name,
            ),
            launch_ps4_teleop(
                teleop_config,
                robot_name,
            ),
            launch_rviz(rviz_config, robot_name),
            launch_node(
                cargo_storage_manager_node_info,
                cargo_storage_manager_config,
                robot_name,
            ),
            # launch_node(
            #     control_circuit_node_info,
            #     control_circuit_config,
            #     robot_name,
            # ),
            launch_node(
                delay_node_info,
                DelayNodeConfig(),
                robot_name,
            ),
            launch_node(
                gentle_stop_node_info,
                gentle_stop_config,
                robot_name,
            ),
            launch_node(
                http_server_node_info,
                http_server_config,
                robot_name,
            ),
            # launch_node(
            #     localization_trigger_node_info,
            #     localization_trigger_config,
            #     robot_name,
            # ),
            # launch_node(
            #     localization_watcher_node_info,
            #     LocalizationWatcherNodeConfig(),
            #     robot_name,
            # ),
            launch_node(
                log_handling_node_info,
                log_handling_config,
                robot_name,
            ),
            # launch_node(
            #     master_control_node_info,
            #     master_control_config,
            #     robot_name,
            # ),
            # launch_node(
            #     master_control_sync_node_info,
            #     MasterControlSyncNodeConfig(),
            #     robot_name,
            # ),
            launch_node(
                mission_control_node_info,
                mission_control_config,
                robot_name,
            ),
            launch_node(
                motor_command_switch_node_info,
                motor_command_switch_config,
                robot_name,
            ),
            launch_node(
                move_commander_node_info,
                move_commander_config,
                robot_name,
            ),
            launch_node(
                obstacle_approximation_node_info,
                obstacle_approximation_config,
                robot_name,
            ),
            # launch_node(
            #     order_conversion_node_info,
            #     OrderConversionNodeConfig(),
            #     robot_name,
            # ),
            # launch_node(
            #     order_management_node_info,
            #     order_management_config,
            #     robot_name,
            # ),
            launch_node(
                qualisys_localization_node_info,
                qualisys_localization_config,
                robot_name,
            ),
            launch_node(
                rosbag_recorder_node_info,
                rosbag_recorder_config,
                robot_name,
            ),
            launch_node(
                status_monitor_node_info,
                StatusMonitorNodeConfig(),
                robot_name,
            ),
            launch_node(
                offset_reduction_node_info,
                offset_reduction_config,
                robot_name,
            ),
            launch_node(
                costmap_clearing_node_info,
                CostmapClearingNodeConfig(),
                robot_name,
            ),
            # launch_node(
            #     enable_localization_switch_node_info,
            #     LocalizationSwitcherNodeConfig(
            #         external_pose_stamped_input_topic=mocap_external_pose_topic
            #     ),
            #     robot_name,
            # ),
        ]
    )
