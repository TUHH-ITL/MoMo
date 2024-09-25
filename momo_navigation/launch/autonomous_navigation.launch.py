"""
Autonomous Navigation Launch File for momo_navigation Package

This launch file is designed to set up autonomous navigation using the following components:
- Velodyne LiDAR
- KISS ICP for odometry
- Extended Kalman Filter (EKF) for sensor fusion (lidar - default, wheel, or  lidar_wheel) 
- Nav2 for localization (amcl) and navigation
- RViz for visualization

This is equivalent to launching the following:

ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
ros2 launch kiss_icp odometry.launch.py topic:=/velodyne_points
ros2 launch momo_navigation {only_lidar_ekf.launch.py , only_wheel_ekf.launch.py, lidar_wheel_ekf.launch.py} 
ros2 launch momo_navigation localization_launch.py map:=/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/maps/map_7.yaml 
ros2 launch momo_navigation navigation_launch.py params_file:=/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/config/nav2_params.yaml
rviz2 (with a config file nav2_default_view.rviz)


To launch with different EKF configurations:

ros2 launch momo_navigation autonomous_navigation.launch.py ekf_type:=lidar 

ros2 launch momo_navigation autonomous_navigation.launch.py ekf_type:=wheel

ros2 launch momo_navigation autonomous_navigation.launch.py ekf_type:= lidar_wheel


Additional Notes:
- The map file is loaded from: `/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/maps/map_7.yaml`
- The navigation parameters are loaded from: `/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/config/nav2_params.yaml`
- RViz uses the configuration from: `/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz`

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the EKF type argument
    ekf_type = LaunchConfiguration('ekf_type')

    # Paths to the packages
    pkg_momo_navigation = get_package_share_directory('momo_navigation')
    pkg_velodyne = get_package_share_directory('velodyne')
    pkg_kiss_icp = get_package_share_directory('kiss_icp')
    
    # File paths
    map_file = '/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/maps/map_7.yaml'
    nav2_params_file = '/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/config/nav2_params.yaml'
    rviz_config_file = '/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz'

    return LaunchDescription([
        # Declare EKF Type Argument
        DeclareLaunchArgument(
            'ekf_type',
            default_value='lidar',
            description='Select EKF type: lidar, wheel, or  lidar_wheel'
        ),
        
        # Launch Velodyne
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_velodyne, 'launch', 'velodyne-all-nodes-VLP16-launch.py'))
        ),

        # Launch KISS ICP Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_kiss_icp, 'launch', 'odometry.launch.py')),
            launch_arguments={'topic': '/velodyne_points'}.items()
        ),

        # Launch EKF based on ekf_type
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_momo_navigation, 'launch', 'only_lidar_ekf.launch.py')),
            condition=IfCondition(PythonExpression(["'", ekf_type, "' == 'lidar'"]))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_momo_navigation, 'launch', 'only_wheel_ekf.launch.py')),
            condition=IfCondition(PythonExpression(["'", ekf_type, "' == 'wheel'"]))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_momo_navigation, 'launch',  'lidar_wheel_ekf.launch.py')),
            condition=IfCondition(PythonExpression(["'", ekf_type, "' == ' lidar_wheel'"]))
        ),

        # Launch Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_momo_navigation, 'launch', 'localization_launch.py')),
            launch_arguments={'map': map_file}.items()
        ),

        # Launch Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_momo_navigation, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params_file}.items()
        ),

        # Launch RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        ),
    ])
