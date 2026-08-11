import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_maxon_control')
    canopen_share = get_package_share_directory('canopen_core')
    config_dir = os.path.join(package_share, 'config', 'maxon_mecanum_bus')
    bus_config = os.path.join(config_dir, 'bus.yml')
    master_bin = os.path.join(config_dir, 'master.bin')
    if not os.path.exists(master_bin):
        master_bin = ''

    can_bus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(canopen_share, 'launch', 'canopen.launch.py')),
        launch_arguments={
            'master_config': os.path.join(config_dir, 'master.dcf'),
            'master_bin': master_bin,
            'bus_config': bus_config,
            'can_interface_name': LaunchConfiguration('can_interface'),
        }.items(),
    )
    controller = TimerAction(
        period=LaunchConfiguration('controller_delay'),
        actions=[Node(
            package='mecanum_maxon_control',
            executable='mecanum_epos4_controller.py',
            parameters=[os.path.join(package_share, 'config', 'controller.yaml')],
            output='screen',
        )],
    )
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('controller_delay', default_value='45.0'),
        can_bus,
        controller,
    ])
