"""ROS2 offline scene plus Transformer decision pipeline.

MoveIt and hardware execution are off by default. The scene publishes only
simulation topics and the decision node publishes candidates, never commands.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    planning_share = Path(get_package_share_directory("xiaou_arm_planning"))
    pipeline = planning_share / "launch" / "pipeline.launch.py"
    project_root = LaunchConfiguration("project_root")
    scenario = LaunchConfiguration("scenario")
    checkpoint = LaunchConfiguration("checkpoint")
    device = LaunchConfiguration("device")
    backend = LaunchConfiguration("backend")
    start_simulation = LaunchConfiguration("start_simulation")
    start_decision = LaunchConfiguration("start_decision")
    return LaunchDescription([
        DeclareLaunchArgument("project_root", default_value="/home/pi/raspi_robot_ai_transport_only_20260808"),
        DeclareLaunchArgument("scenario", default_value="bottle"),
        DeclareLaunchArgument("checkpoint", default_value="runtime/decision/transformer_policy_safety_final_20260813.int8.onnx"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("backend", default_value="onnx"),
        DeclareLaunchArgument("start_simulation", default_value="true"),
        DeclareLaunchArgument("start_decision", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pipeline)),
            launch_arguments={
                "project_root": project_root,
                "start_perception": "false",
                "start_move_group": "false",
                "start_planner": "false",
                "target_class": "",
                "table_z_m": "nan",
            }.items(),
        ),
        Node(
            package="xiaou_arm_simulation",
            executable="desktop_scene_node",
            condition=IfCondition(start_simulation),
            parameters=[{"project_root": project_root, "scenario": scenario}],
            output="screen",
        ),
        Node(
            package="xiaou_arm_decision",
            executable="decision_node",
            condition=IfCondition(start_decision),
            parameters=[
                {
                    "project_root": project_root,
                    "checkpoint": checkpoint,
                    "device": device,
                    "backend": backend,
                    "simulation_mode": True,
                    "input_topic": "/xiaou/sim/observation",
                }
            ],
            output="screen",
        ),
    ])
