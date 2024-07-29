from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    viz_config_launch_arg = DeclareLaunchArgument(
        "viz_config", default_value="multi_agent_config.rviz"
    )
    viz_param = PathJoinSubstitution(
        [
            get_package_share_directory("mar_bringup"),
            "config",
            "visualizer",
            LaunchConfiguration("viz_config"),
        ]
    )

    viz_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", viz_param],
    )

    return LaunchDescription([viz_config_launch_arg, viz_node])
