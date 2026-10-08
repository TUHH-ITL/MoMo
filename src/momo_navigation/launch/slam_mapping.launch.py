"""Build an occupancy grid map in mocap coordinates with slam_toolbox.

Expects the mocap stack (momo_no_master_control_mocap.launch.py) to be
running: it provides odom->base_link from the Qualisys pose and the static
identity map->odom. See config/slam_toolbox_mocap.yaml for why scan matching
is disabled.

The map is published on /<namespace>/slam_map. Save it with:
    ros2 run nav2_map_server map_saver_cli -f <name> -t /MoMo/slam_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    default_robot_name = os.getenv("ROBOT_NAME", "MoMo")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value=default_robot_name,
        description="Top-level namespace",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("momo_navigation"),
            "config",
            "slam_toolbox_mocap.yaml",
        ),
        description="slam_toolbox parameter file",
    )

    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        output="screen",
        parameters=[params_file, {"use_sim_time": False, "use_lifecycle_manager": False}],
        # tf2 uses the literal absolute /tf; the rest of the stack lives on
        # /<namespace>/tf.
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[slam_mapping] activating slam_toolbox"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_params_file_cmd,
            slam_toolbox,
            configure_event,
            activate_event,
        ]
    )
