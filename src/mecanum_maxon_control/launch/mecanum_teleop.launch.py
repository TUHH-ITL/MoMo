import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_maxon_control')
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'mecanum_drive.launch.py'))
    )
    teleop_config = os.path.join(package_share, 'config', 'teleop_joy.yaml')
    return LaunchDescription([
        drive,
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[teleop_config], output='screen'),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node', parameters=[teleop_config], output='screen'),
    ])
