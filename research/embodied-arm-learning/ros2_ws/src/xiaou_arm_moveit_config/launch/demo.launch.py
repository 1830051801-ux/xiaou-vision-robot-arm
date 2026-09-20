from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    description_share = Path(get_package_share_directory("xiaou_arm_description"))
    moveit_config = (
        MoveItConfigsBuilder("xiaou_six_axis_arm", package_name="xiaou_arm_moveit_config")
        .robot_description(
            file_path=str(description_share / "urdf" / "xiaou_arm_display.urdf.xacro")
        )
        .robot_description_semantic(file_path="config/xiaou_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
