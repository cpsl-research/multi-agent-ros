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

    n_agents = int(context.launch_configurations["n_agents"])

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
                "agent_name": f"agent{i}",
                "agent_type": "mobile.yaml" if i == 0 else "static.yaml",
            }.items(),
        )
        for i in range(0, n_agents)
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
    n_agents_launch_arg = DeclareLaunchArgument("n_agents", default_value="4")
    viz_config_launch_arg = DeclareLaunchArgument(
        "viz_config", default_value="agent0_config.rviz"
    )

    agent_nodes = OpaqueFunction(function=get_agents)
    viz_node = OpaqueFunction(function=get_viz)

    return LaunchDescription(
        [
            n_agents_launch_arg,
            viz_config_launch_arg,
            agent_nodes,
            viz_node,
        ]
    )
