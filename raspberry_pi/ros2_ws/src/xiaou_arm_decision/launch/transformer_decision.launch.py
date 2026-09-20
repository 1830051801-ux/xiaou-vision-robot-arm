from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("project_root", default_value="/home/pi/raspi_robot_ai_transport_only_20260808"),
        DeclareLaunchArgument("checkpoint", default_value="runtime/decision/transformer_policy_safety_final_20260813.int8.onnx"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("backend", default_value="onnx"),
        DeclareLaunchArgument("input_topic", default_value="/xiaou/decision_observation"),
        DeclareLaunchArgument("simulation_mode", default_value="true"),
        Node(
            package="xiaou_arm_decision",
            executable="decision_node",
            parameters=[{
                "project_root": LaunchConfiguration("project_root"),
                "checkpoint": LaunchConfiguration("checkpoint"),
                "device": LaunchConfiguration("device"),
                "backend": LaunchConfiguration("backend"),
                "input_topic": LaunchConfiguration("input_topic"),
                "simulation_mode": LaunchConfiguration("simulation_mode"),
            }],
            output="screen",
        ),
    ])
