from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    project_share = Path(get_package_share_directory("xiaou_arm_description"))
    xacro_path = project_share / "urdf" / "xiaou_arm_display.urdf.xacro"
    robot_description = xacro.process_file(
        str(xacro_path), mappings={"use_can_hardware": "true"}
    ).toxml()
    controller_config = Path(get_package_share_directory("xiaou_arm_can_control")) / "config" / "controllers.yaml"
    project_root = LaunchConfiguration("project_root")

    return LaunchDescription(
        [
            DeclareLaunchArgument("project_root", default_value="/home/pi/raspi_robot_ai"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="xiaou_arm_hardware",
                executable="hardware_readiness_node",
                parameters=[{"project_root": project_root}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{"robot_description": robot_description}, str(controller_config)],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]
    )
