import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_agents(context):
    """Set up the agents"""
    # HACK: fix the static and mobile agents mappings

    agent_name = str(context.launch_configurations["agent_name"])
    agent_type = str(context.launch_configurations["agent_type"])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("mar_bringup"),
                        "launch",
                        "agent_passive.launch.py",
                    ),
                ]
            ),
            launch_arguments={
                "agent_name": agent_name,
                "agent_type": agent_type,
            }.items(),
        )
    ]


def get_viz(context):
    """Set up the visualizer"""

    viz_config = str(context.launch_configurations["viz_config"])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("mar_bringup"),
                        "launch",
                        "rviz.launch.py",
                    ),
                ]
            ),
            launch_arguments={
                "viz_config": viz_config,
            }.items(),
        )
    ]


def generate_launch_description():
    agent_name_launch_arg = DeclareLaunchArgument("agent_name", default_value="agent0")
    agent_type_launch_arg = DeclareLaunchArgument(
        "agent_type", default_value="mobile.yaml"
    )
    viz_config_launch_arg = DeclareLaunchArgument(
        "viz_config", default_value="agent0_config.rviz"
    )

    agent_nodes = OpaqueFunction(function=get_agents)
    viz_node = OpaqueFunction(function=get_viz)

    return LaunchDescription(
        [
            agent_name_launch_arg,
            agent_type_launch_arg,
            agent_nodes,
            viz_config_launch_arg,
            viz_node,
        ]
    )
