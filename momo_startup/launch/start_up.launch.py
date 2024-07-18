from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                '/home/itl_bot2/ros2_ws/src/momo/momo_description/launch/description.launch.py'
            ]),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
