"""Run the taught-grasp Cartesian route in MoveIt/RViz preview mode only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    planning_share = Path(get_package_share_directory("xiaou_arm_planning"))
    pipeline = planning_share / "launch" / "pipeline.launch.py"
    project_root = LaunchConfiguration("project_root")
    waypoint_file = LaunchConfiguration("waypoint_file")
    return LaunchDescription([
        DeclareLaunchArgument("project_root", default_value="/home/pi/raspi_robot_ai"),
        DeclareLaunchArgument(
            "waypoint_file",
            default_value="runtime/teach_poses/cola_taught_grasp_ros2_waypoints.json",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pipeline)),
            launch_arguments={
                "project_root": project_root,
                "start_perception": "false",
                "start_planner": "false",
                "start_joint_state_publisher": "false",
            }.items(),
        ),
        Node(
            package="xiaou_arm_planning",
            executable="cartesian_preview_node",
            parameters=[{
                "avoid_collisions": False,
                "min_fraction": 1.0,
            }],
            output="screen",
        ),
        Node(
            package="xiaou_arm_planning",
            executable="publish_taught_waypoints.py",
            arguments=["--waypoints-file", waypoint_file, "--project-root", project_root],
            output="screen",
        ),
    ])
