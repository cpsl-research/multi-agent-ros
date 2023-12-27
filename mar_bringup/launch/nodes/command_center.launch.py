import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    cc_pipeline_launch_arg = DeclareLaunchArgument(
        "cc_pipeline", default_value="command_center.py"
    )

    output_folder = LaunchConfiguration("output_folder")

    cc_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "command_center",
        "cc_standard.yml",
    )

    cc_broker_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "command_center",
        "cc_broker_standard.yml",
    )

    cc_broker_node = Node(
        package="mar_agents",
        executable="command_center_broker",
        namespace="command_center",
        name="command_center_broker",
        parameters=[
            cc_broker_config,
            {"namespace": "command_center_broker", "output_folder": output_folder},
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    cc_node = Node(
        package="mar_agents",
        executable="command_center",
        namespace="command_center",
        name="command_center",
        parameters=[
            cc_config,
            {"namespace": "command_center", "output_folder": output_folder},
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription(
        [
            cc_pipeline_launch_arg,
            cc_broker_node,
            cc_node,
        ]
    )
