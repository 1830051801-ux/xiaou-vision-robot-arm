from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    project_root = LaunchConfiguration("project_root")
    image_topic = LaunchConfiguration("image_topic")
    target_class = LaunchConfiguration("target_class")
    table_z_m = LaunchConfiguration("table_z_m")
    start_perception = LaunchConfiguration("start_perception")
    start_move_group = LaunchConfiguration("start_move_group")
    start_planner = LaunchConfiguration("start_planner")

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
    moveit_parameters = moveit_config.to_dict()

    return LaunchDescription(
        [
            DeclareLaunchArgument("project_root", default_value="/home/pi/raspi_robot_ai"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("target_class", default_value=""),
            DeclareLaunchArgument("table_z_m", default_value="nan"),
            DeclareLaunchArgument(
                "start_perception",
                default_value="true",
                description="Instantiate the camera/YOLO node; keep false for review-only planning.",
            ),
            DeclareLaunchArgument(
                "start_move_group",
                default_value="true",
                description="Start MoveIt move_group; requires a planner plugin such as OMPL.",
            ),
            DeclareLaunchArgument(
                "start_planner",
                default_value="true",
                description="Start the target planner; requires move_group.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[moveit_config.robot_description],
                output="screen",
            ),
            # Review-only zero joint states. Replace this node with measured feedback
            # after the six-axis hardware interface has been calibrated and verified.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[moveit_config.robot_description, {"rate": 30}],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                condition=IfCondition(start_move_group),
                parameters=[
                    moveit_parameters,
                    {
                        "allow_trajectory_execution": False,
                        "publish_robot_description_semantic": True,
                        "publish_planning_scene": True,
                        "publish_geometry_updates": True,
                        "publish_state_updates": True,
                        "publish_transforms_updates": True,
                    },
                ],
                output="screen",
            ),
            Node(
                package="xiaou_arm_hardware",
                executable="hardware_readiness_node",
                parameters=[{"project_root": project_root}],
                output="screen",
            ),
            Node(
                package="xiaou_arm_perception",
                executable="target_pose_node",
                condition=IfCondition(start_perception),
                parameters=[
                    {
                        "project_root": project_root,
                        "image_topic": image_topic,
                        "target_class": target_class,
                        "table_z_m": table_z_m,
                    }
                ],
                output="screen",
            ),
            Node(
                package="xiaou_arm_planning",
                executable="target_planner_node",
                condition=IfCondition(start_planner),
                parameters=[moveit_parameters, {"allow_execution": False}],
                output="screen",
            ),
        ]
    )
