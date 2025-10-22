"""Equivalent Individual Commands:

1. Run the IMU covariance republisher:
   ros2 run momo_navigation imperfect_odometry_tf_publisher

 Launch localization:
   ros2 launch momo_navigation localization_launch.py map:=/home/workstation/MoMo/momo_isaac_sim/maps/exp_warehouse_map.yaml

 Launch the Nav2 navigation stack:
   ros2 launch momo_navigation navigation_launch.py params_file:=/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml

 Launch RViz2 with a custom view:
   rviz2 -d /home/workstation/ros2_ws/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz

Each of them is launched after some delay from the previous one.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Base path to the package
    momo_pkg = os.path.join(
        get_package_share_directory("momo_navigation"),
    )

    # map_file = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/maps/exp_warehouse_map.yaml"
    map_file = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/maps/symmetric_exp_2.yaml"

    nav2_params_file = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml"
    rviz_config = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz"

    return LaunchDescription(
        [
            Node(
                package="robot_localization_monitor",
                executable="imperfect_odometry_noisy_tf_publisher",
                name="imperfect_odometry_noisy_tf_publisher",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_to_world_tf",
                arguments=["0", "0", "0", "0", "0", "0", "1", "map", "world"],
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                momo_pkg,
                                "launch",
                                "localization_launch.py",
                            ),
                        ),
                        launch_arguments={
                            "map": map_file,
                            "use_sim_time": "true",
                        }.items(),
                    ),
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                momo_pkg,
                                "launch",
                                "navigation_launch.py",
                            ),
                        ),
                        launch_arguments={
                            "params_file": nav2_params_file,
                            "use_sim_time": "true",
                        }.items(),
                    ),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        arguments=["-d", rviz_config],
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                    ),
                ],
            ),
        ],
    )
