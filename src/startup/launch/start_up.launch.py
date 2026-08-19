import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    robot_name = os.getenv("ROBOT_NAME", "MoMo")
    mecanum_share = get_package_share_directory("mecanum_maxon_control")
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mecanum_share, "launch", "mecanum_drive.launch.py")
        )
    )
    description_share = get_package_share_directory("momo_description")
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    description_share, "launch", "description.launch.py"
                )
            ]
        ),
    )
    startup_share = get_package_share_directory("startup")
    tim_lidars = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(startup_share, "tim_lidars.launch.py")
        )
    )
    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace(robot_name),
                    description,
                    drive,
                    tim_lidars,
                ]
            ),
        ]
    )
