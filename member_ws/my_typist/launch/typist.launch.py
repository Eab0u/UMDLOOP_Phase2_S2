"""Launch the autonomous S2 typist node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the configurable launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "episodes",
                default_value="1",
                description="Number of consecutive seeds to attempt",
            ),
            DeclareLaunchArgument(
                "log_path",
                default_value="/ws/control_log.csv",
                description="CSV mission-log output path",
            ),
            DeclareLaunchArgument(
                "vision_log_path",
                default_value="/ws/perception_log.csv",
                description="CSV perception and result log output path",
            ),
            Node(
                package="my_typist",
                executable="typist",
                name="typist",
                output="screen",
                parameters=[
                    {
                        "episodes": LaunchConfiguration("episodes"),
                        "log_path": LaunchConfiguration("log_path"),
                        "vision_log_path": LaunchConfiguration("vision_log_path"),
                    }
                ],
            ),
        ]
    )
