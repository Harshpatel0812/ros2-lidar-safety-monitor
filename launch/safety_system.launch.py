from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("robot_safety_monitor"))
    parameters = str(package_share / "config" / "safety.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_safety_monitor",
                executable="safety_monitor_node",
                name="safety_monitor",
                parameters=[parameters],
                output="screen",
            ),
            Node(
                package="robot_safety_monitor",
                executable="velocity_guard",
                name="velocity_guard",
                output="screen",
            ),
        ]
    )
