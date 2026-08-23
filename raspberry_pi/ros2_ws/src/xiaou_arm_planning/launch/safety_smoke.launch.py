"""ROS2 safety-layer smoke launch that does not require a MoveIt planner plugin."""

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
                    "start_move_group": "false",
                    "start_planner": "false",
                    "target_class": "",
                    "table_z_m": "nan",
                }.items(),
            )
        ]
    )
