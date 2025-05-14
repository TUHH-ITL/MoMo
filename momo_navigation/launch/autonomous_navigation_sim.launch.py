"""
Autonomous Navigation Launch File for momo_navigation Package

This launch file is designed to set up autonomous navigation using the following components:
- KISS ICP for odometry
- Extended Kalman Filter (EKF) for sensor fusion (lidar - default, wheel, or  lidar_wheel)
- Nav2 for localization (amcl) and navigation
- RViz for visualization

This is equivalent to launching the following:

ros2 launch kiss_icp odometry.launch.py topic:=/point_cloud visualize:=false
ros2 launch momo_navigation {only_lidar_ekf.launch.py , only_wheel_ekf.launch.py, lidar_wheel_ekf.launch.py}
ros2 launch momo_navigation localization_launch.py map:=/home/workstation/ros2_ws/src/robot_localization_monitor/maps/map_sim.yaml
ros2 launch momo_navigation navigation_launch.py params_file:=/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml
rviz2 (with a config file nav2_default_view.rviz)


To launch with different EKF configurations:

ros2 launch momo_navigation autonomous_navigation.launch.py ekf_type:=lidar

ros2 launch momo_navigation autonomous_navigation.launch.py ekf_type:=wheel

ros2 launch momo_navigation autonomous_navigation.launch.py ekf_type:= lidar_wheel


Additional Notes:
- The map file is loaded from: `/home/workstation/ros2_ws/src/robot_localization_monitor/maps/map_sim.yaml`
- The navigation parameters are loaded from: `/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml`
- RViz uses the configuration from: `/home/workstation/ros2_ws/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz`

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Declare the EKF type argument
    ekf_type = LaunchConfiguration("ekf_type")
    pkg_momo_navigation = get_package_share_directory("momo_navigation")
    pkg_kiss_icp = get_package_share_directory("kiss_icp")

    # File paths
    map_file = (
        "/home/workstation/MoMo/momo_isaac_sim/maps/warehouse_slamtoolbox.yaml"
    )
    nav2_params_file = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml"
    rviz_config_file = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz"

    return LaunchDescription(
        [
            # Launch KISS ICP Odometry
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_kiss_icp, "launch", "odometry.launch.py")
                ),
                launch_arguments={
                    "topic": "/point_cloud",
                    "visualize": "false",
                    "publish_odom_tf": "true",
                    "base_frame": "base_link",
                    "lidar_odom_frame": "odom",
                    "use_sim_time": "true",
                }.items(),
            ),
            # Launch Localization
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_momo_navigation, "launch", "localization_launch.py"
                    )
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": "true",
                }.items(),
            ),
            # Launch Navigation
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_momo_navigation, "launch", "navigation_launch.py"
                    )
                ),
                launch_arguments={
                    "params_file": nav2_params_file,
                    "use_sim_time": "true",
                }.items(),
            ),
            # Launch RViz
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config_file], output="log"
            ),
        ]
    )
