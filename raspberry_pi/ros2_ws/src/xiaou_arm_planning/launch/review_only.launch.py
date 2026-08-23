"""ROS2/MoveIt review-only launch with perception and hardware execution disabled."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("xiaou_arm_planning"))
    pipeline = package_share / "launch" / "pipeline.launch.py"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(pipeline)),
                launch_arguments={
                    "start_perception": "false",
                    "target_class": "",
                    "table_z_m": "nan",
                }.items(),
            )
        ]
    )
