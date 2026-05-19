from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("diablo_dance")
    default_yaml = os.path.join(pkg_share, "config", "dance_demo.yaml")

    choreo_yaml = DeclareLaunchArgument(
        "choreo_yaml", default_value=default_yaml,
        description="Path to choreography YAML (steps or timeline)."
    )
    auto_start = DeclareLaunchArgument(
        "auto_start", default_value="false",
        description="Auto-start the choreography on node startup."
    )

    node = Node(
        package="diablo_dance",
        executable="diablo_dance_orchestrator",
        name="diablo_dance_orchestrator",
        output="log",
        parameters=[{
            "choreo_yaml": LaunchConfiguration("choreo_yaml"),
            "auto_start": LaunchConfiguration("auto_start"),
        }],
    )

    return LaunchDescription([choreo_yaml, auto_start, node])
