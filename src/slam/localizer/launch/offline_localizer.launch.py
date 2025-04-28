import os
import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home_directory = os.path.expanduser("~")

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("localizer"), "rviz", "localizer.rviz"]
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    fast_lio_pkg = get_package_share_directory("fast_lio")
    fast_lio_launch = os.path.join(fast_lio_pkg,'launch')
    fast_lio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                fast_lio_launch,
                'offline_mapping.launch.py'
            )
        )
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )    
    return launch.LaunchDescription(
        [
            declare_use_sim_time_cmd,
            fast_lio_include,
            # launch_ros.actions.Node(
            #     package="fastlio2",
            #     namespace="fastlio2",
            #     executable="lio_node",
            #     name="lio_node",
            #     output="screen",
            #     parameters=[
            #         {"config_path": lio_config_path.perform(launch.LaunchContext())}
            #     ],
            # ),
            launch_ros.actions.Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {"config_path": localizer_config_path.perform(launch.LaunchContext())},
                    {"map_path": os.path.join(home_directory, "pointcloud.pcd")},
                    {'use_sim_time': use_sim_time}
                ],
            ),
            # launch_ros.actions.Node(
            #     package="rviz2",
            #     namespace="localizer",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            # )
        ]
    )
