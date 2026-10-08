import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_can_interface(context, *args, **kwargs):
    """Fail fast with a clear message if can0-up.service hasn't done its job.

    Bringing the interface up is systemd's responsibility (can0-up.service,
    BindsTo=sys-subsystem-net-devices-can0.device -- fires on Kvaser hotplug,
    auto-recovers bus-off via restart-ms 100). Re-doing that here would only
    duplicate it worse (this runs as the robot user, no sudo). This check
    only verifies that job actually succeeded before we hand the interface
    to canopen_core, instead of finding out 45s into a silent hang.
    """
    interface = LaunchConfiguration("can_interface").perform(context)
    operstate_path = f"/sys/class/net/{interface}/operstate"
    try:
        with open(operstate_path) as f:
            state = f.read().strip()
    except FileNotFoundError:
        state = None
    if state == "up":
        return []
    return [
        LogInfo(
            msg=(
                f"CAN interface '{interface}' is not up (operstate={state!r}). "
                f"Expected can0-up.service to have brought it up already -- "
                f"check 'systemctl status can0-up.service' and the Kvaser "
                f"adapter connection before retrying."
            )
        ),
        Shutdown(reason=f"CAN interface '{interface}' not ready"),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("mecanum_maxon_control")
    canopen_share = get_package_share_directory("canopen_core")
    config_dir = os.path.join(package_share, "config", "maxon_mecanum_bus")
    bus_config = os.path.join(config_dir, "bus.yml")
    master_bin = os.path.join(config_dir, "master.bin")
    if not os.path.exists(master_bin):
        master_bin = ""

    can_check = OpaqueFunction(function=check_can_interface)

    can_bus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(canopen_share, "launch", "canopen.launch.py")
        ),
        launch_arguments={
            "master_config": os.path.join(config_dir, "master.dcf"),
            "master_bin": master_bin,
            "bus_config": bus_config,
            "can_interface_name": LaunchConfiguration("can_interface"),
        }.items(),
    )
    # No outer TimerAction guess anymore: the node's own startup_settle_delay
    # + startup_retries (controller.yaml) already wait out the CANopen bus
    # boot adaptively via real service calls. A fixed outer delay only
    # stacked a second blind guess on top of that. respawn covers the crash
    # case a fixed delay never did.
    controller = Node(
        package="mecanum_maxon_control",
        executable="mecanum_epos4_controller.py",
        parameters=[os.path.join(package_share, "config", "controller.yaml")],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    # Doesn't depend on the controller being ready -- it only reads the
    # CANopen drivers' encoder feedback, which is live as soon as the bus is
    # up. Unlike the controller it fires no CiA402 init calls, so it doesn't
    # need to wait out the bus boot handshake -- give it its own short delay
    # so odom->base_link TF is available well before nav2's lifecycle
    # bringup gives up waiting for it.
    wheel_odometry = TimerAction(
        period=LaunchConfiguration("odometry_delay", default="3.0"),
        actions=[
            Node(
                package="mecanum_maxon_control",
                executable="mecanum_wheel_odometry.py",
                parameters=[os.path.join(package_share, "config", "controller.yaml")],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                # TransformBroadcaster publishes to the literal absolute "/tf",
                # ignoring PushRosNamespace. Without this remap odom->base_link
                # lands on the global /tf while nav2 (and the lidars, and rviz)
                # listen on "/MoMo/tf" -- so controller_server never sees an
                # "odom" frame at all and local_costmap activation times out.
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            )
        ],
    )
    # Waits (latched) for the controller's "motors_ready" event, then tells the
    # LED/speaker Arduino it's safe to announce "Ich bin bereit".
    status_indicator = Node(
        package="mecanum_maxon_control",
        executable="arduino_status_indicator.py",
        parameters=[{
            "connected_port": LaunchConfiguration("status_indicator_port"),
        }],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("odometry_delay", default_value="3.0"),
            DeclareLaunchArgument(
                "status_indicator_port", default_value="/dev/ttyArduinoStatus"
            ),
            can_check,
            can_bus,
            controller,
            wheel_odometry,
            status_indicator,
        ]
    )
