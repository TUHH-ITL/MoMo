
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=["/home/workstation/ros2_ws/src/MoMo/momo_navigation/config/nav2_params.yaml"],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        )
    ])
