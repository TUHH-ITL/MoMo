import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sick_scan_share = get_package_share_directory("sick_scan_xd")
    # sick_generic_caller reads its base params from this XML file, then
    # applies any trailing "name:=value" arguments on top of it -- same
    # mechanism sick_tim_5xx.launch.py itself uses via sys.argv.
    launch_file_path = os.path.join(
        sick_scan_share, "launch", "sick_tim_5xx.launch"
    )

    tim_1 = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="sick_tim_5xx_1",
        output="screen",
        arguments=[
            launch_file_path,
            "hostname:=192.168.0.41",
            "frame_id:=tim_1",
            # Measured relative to base_link (x forward, y left, z up):
            # -8.5cm x, -14cm y from back_left_wheel (-0.2735, 0.318),
            # 27cm above the floor (base_link sits ~0cm above the floor).
            # Sensor faces -x, so yaw = pi.
            "tf_base_frame_id:=base_link",
            "tf_base_lidar_xyz_rpy:=-0.3585,0.178,0.27,0,0,3.14159265",
        ],
        # Sensor's own reported time_increment field is inconsistent with
        # its actual angle_increment/scan_time (known TiM5xx firmware
        # quirk) -- override with the driver-computed value. Unlike the
        # args above, this is a real declared ROS2 parameter, not part of
        # the driver's internal file-based param table, so it goes here.
        parameters=[{"time_increment": 6.17222e-05}],
        # Without this, the driver's internal TF broadcaster publishes to
        # the literal absolute "/tf" (ignoring PushRosNamespace), while
        # everything else in this launch stack (robot_state_publisher,
        # rviz2, ...) is remapped the same way onto "/MoMo/tf" -- two
        # disconnected TF trees, so RViz never sees tim_1/tim_2 at all.
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("scan", "scan_1"),
            ("cloud", "cloud_1"),
        ],
    )
    tim_2 = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="sick_tim_5xx_2",
        output="screen",
        arguments=[
            launch_file_path,
            # Re-IP'd off 192.168.0.0/24 onto its own subnet: it's now
            # reached over a separate USB adapter (enxd03745b8ffee), and two
            # interfaces sharing one subnet gives Linux an ambiguous route.
            "hostname:=192.168.1.42",
            "frame_id:=tim_2",
            # Measured relative to base_link (x forward, y left, z up):
            # +14cm x, +8cm y from front_right_wheel (0.26849, -0.238),
            # 28.5cm above the floor (base_link sits ~0cm above the floor).
            # Sensor faces +x, so yaw = 0.
            "tf_base_frame_id:=base_link",
            "tf_base_lidar_xyz_rpy:=0.40849,-0.158,0.285,0,0,0",
        ],
        parameters=[{"time_increment": 6.17222e-05}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("scan", "scan_2"),
            ("cloud", "cloud_2"),
        ],
    )

    return LaunchDescription([tim_1, tim_2])
