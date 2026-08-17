from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("project_root", default_value="/home/pi/raspi_robot_ai_transport_only_20260808"),
        DeclareLaunchArgument("scenario", default_value="bottle"),
        Node(
            package="xiaou_arm_simulation",
            executable="desktop_scene_node",
            parameters=[{
                "project_root": LaunchConfiguration("project_root"),
                "scenario": LaunchConfiguration("scenario"),
            }],
            output="screen",
        ),
    ])
