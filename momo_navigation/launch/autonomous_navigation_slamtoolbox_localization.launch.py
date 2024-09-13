"""
Launch file for autonomous navigation.

This launch file integrates the following components:
- Velodyne LIDAR
- Kiss ICP for LIDAR-based odometry
- Extended Kalman Filter (EKF) for state estimation (LIDAR, wheel, or both)
- SLAM Toolbox in localization mode
- Nav2 for navigation
- rviz2 for visualization

This launch file is equivalent of launching the following:

ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py 
ros2 launch kiss_icp odometry.launch.py topic:=/velodyne_points
ros2 launch momo_navigation {ekf.launch.py , only_lidar_ekf.launch.py, only_wheel_ekf.launch.py}
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/config/slam_toolbox_localization.yaml 
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/config/nav2_params.yaml 
rviz2 (with config file)


Usage:
    To launch the system with the default configuration (combined LIDAR and wheel EKF):
    $ ros2 launch momo_navigation autonomous_navigation_slamtoolbox_localization.launch.py

    To specify a particular EKF type, use the `ekf_type` argument:
    - LIDAR-only EKF:
        $ ros2 launch momo_navigation autonomous_navigation_slamtoolbox_localization.launch.py ekf_type:=lidar
    - Wheel-only EKF:
        $ ros2 launch momo_navigation autonomous_navigation_slamtoolbox_localization.launch.py ekf_type:=wheel
    - Combined EKF (default):
        $ ros2 launch momo_navigation autonomous_navigation_slamtoolbox_localization.launch.py ekf_type:=both
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration  

def generate_launch_description():
    # Define paths to your different launch files
    velodyne_launch = os.path.join(
        get_package_share_directory('velodyne'), 'launch', 'velodyne-all-nodes-VLP16-launch.py'
    )
    odometry_launch = os.path.join(
        get_package_share_directory('kiss_icp'), 'launch', 'odometry.launch.py'
    )
    lidar_ekf_launch = os.path.join(
        get_package_share_directory('momo_navigation'), 'launch', 'only_lidar_ekf.launch.py'
    )
    wheel_ekf_launch = os.path.join(
        get_package_share_directory('momo_navigation'), 'launch', 'only_wheel_ekf.launch.py'
    )
    combined_ekf_launch = os.path.join(
        get_package_share_directory('momo_navigation'), 'launch', 'ekf.launch.py'
    )
    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
    )
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    )

    # Path to the RViz config file
    rviz_config_file = '/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz'

    # Launch argument to select which EKF configuration to use, defaulting to 'both'
    declare_ekf_type = DeclareLaunchArgument(
        'ekf_type',
        default_value='both',  # Default is to use both LIDAR and wheel odometry EKF
        description='Choose which EKF to use: lidar, wheel, or both'
    )

    # Create the launch description with all the launch files
    return LaunchDescription([

        # Declare the launch argument
        declare_ekf_type,

        # Launch Velodyne node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_launch)
        ),

        # Launch Kiss ICP Odometry with remapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch),
            launch_arguments={'topic': '/velodyne_points'}.items()
        ),

        # Launch EKF for LIDAR-only if 'lidar' is selected
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_ekf_launch),
            condition=LaunchConfigurationEquals('ekf_type', 'lidar')
        ),

        # Launch EKF for wheel odometry-only if 'wheel' is selected
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wheel_ekf_launch),
            condition=LaunchConfigurationEquals('ekf_type', 'wheel')
        ),

        # Launch EKF for both LIDAR and wheel odometry if 'both' is selected
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(combined_ekf_launch),
            condition=LaunchConfigurationEquals('ekf_type', 'both')
        ),

        # Launch SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': '/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/config/slam_toolbox_localization.yaml'
            }.items()
        ),

        # Launch Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': '/home/itlbot2/ros2_humble/src/MoMo/momo_navigation/config/nav2_params.yaml'
            }.items()
        ),

        # Launch RViz2 with custom config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
