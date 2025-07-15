"""

Equivalent Individual Commands:

1. Run the IMU covariance republisher:
   ros2 run momo_navigation imu_covariance_republisher

2. Launch LiDAR odometry using KissICP:
   ros2 launch kiss_icp odometry.launch.py topic:=/point_cloud visualize:=false lidar_odom_frame:=odom

3. Launch the EKF filter for LiDAR + IMU fusion:
   ros2 launch momo_navigation lidar_imu_ekf.launch.py

4. Launch localization using SLAM toolbox:
   ros2 launch momo_navigation localization_launch.py map:=/home/workstation/MoMo/momo_isaac_sim/maps/warehouse_slamtoolbox.yaml

5. Launch the Nav2 navigation stack:
   ros2 launch momo_navigation navigation_launch.py params_file:=/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml

6. Launch RViz2 with a custom view:
   rviz2 -d /home/workstation/ros2_ws/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz

Each of them is launched after 1 second delay from the previous one.

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
        get_package_share_directory('momo_navigation')
    )

    kiss_icp_pkg = os.path.join(
        get_package_share_directory('kiss_icp')
    )

    # File paths
    map_file = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/maps/warehouse_slamtoolbox.yaml"
    nav2_params_file = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml"
    rviz_config = "/home/workstation/ros2_ws/src/MoMo/momo_navigation/rviz/nav2_default_view.rviz"

    return LaunchDescription([
        # 1. imu_covariance_republisher
        Node(
            package="momo_navigation",
            executable="imu_covariance_republisher",
            name="imu_covariance_republisher",
            parameters=[{'use_sim_time': True}]
        ),
        # 2. launch imperfect_odometry_publisher
        Node(
            package="robot_localization_monitor",
            executable="imperfect_odometry_publisher",
            name="imperfect_odometry_publisher",
            parameters=[{'use_sim_time': True}]
        ),

        # 3. Delay 2s total, launch EKF
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(momo_pkg, 'launch', 'imperfect_odom_ekf.launch.py') # lidar_imu_ekf.launch.py
                    )
                )
            ]
        ),

        # 4. Delay 3s total, launch localization
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(momo_pkg, 'launch', 'localization_launch.py')
                    ),
                    launch_arguments={
                        'map': map_file,
                        'use_sim_time': 'true',
                    }.items()
                )
            ]
        ),

        # 5. Delay 4s total, launch navigation
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(momo_pkg, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'params_file': nav2_params_file,
                        'use_sim_time': 'true',
                    }.items()
                )
            ]
        ),

        # 6. Delay 5s total, launch RViz2
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=["-d", rviz_config],
                    output="screen",
                    parameters=[{'use_sim_time': True}],
                )
            ]
        ),
    ])
