import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()

    cam_1_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="cam_1",
        namespace='left_cam',
        parameters=[os.path.join(get_package_share_directory('momo_startup'),'config','params_1_720p.yaml')]
    )

    cam_2_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name = "cam_2",
        namespace='right_cam',
        parameters=[os.path.join(get_package_share_directory('momo_startup'),'config','params_2_720p.yaml')]
    )

    cam_3_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name = "cam_3",
        namespace='front_cam',
        parameters=[os.path.join(get_package_share_directory('momo_startup'),'config','params_3_720p.yaml')]
    )

    cam_4_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name = "cam_4",
        namespace='back_cam',
        parameters=[os.path.join(get_package_share_directory('momo_startup'),'config','params_4_720p.yaml')]
    )

    ld.add_action(cam_1_node)
    ld.add_action(cam_2_node)
    ld.add_action(cam_3_node)
    ld.add_action(cam_4_node)

    return ld