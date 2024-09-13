from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory of the momo_navigation package
    package_share_directory = get_package_share_directory('momo_navigation')

    # Path to the ekf.yaml parameter file
    ekf_param_file_path = os.path.join(package_share_directory, 'config', 'only_wheel_ekf.yaml')

    # Define the ekf_filter_node
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_param_file_path]
    )

    # Create the launch description and add the node
    ld = LaunchDescription()
    ld.add_action(ekf_filter_node)

    return ld
