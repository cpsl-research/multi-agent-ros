from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def get_agents(context):
    """Set up the agents"""
    # HACK: fix the static and mobile agents mappings

    agent_id = int(context.launch_configurations["agent_id"])
    agent_name = f"agent{agent_id}"
    agent_type = "mobile.yaml" if agent_id == 0 else "static.yaml"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("mar_bringup"),
                            "launch",
                            "carla_trust_experiments",
                            "nodes",
                            "cte_agent_passive.launch.py",
                        ]
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

    agent_id = int(context.launch_configurations["agent_id"])
    viz_config = f"agent{agent_id}_config.rviz"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("mar_bringup"),
                            "launch",
                            "carla_trust_experiments",
                            "nodes",
                            "cte_rviz.launch.py",
                        ]
                    ),
                ]
            ),
            launch_arguments={
                "viz_config": viz_config,
            }.items(),
        )
    ]


def generate_launch_description():
    agent_id_launch_arg = DeclareLaunchArgument("agent_id")
    agent_nodes = OpaqueFunction(function=get_agents)
    viz_node = OpaqueFunction(function=get_viz)

    return LaunchDescription(
        [
            agent_id_launch_arg,
            agent_nodes,
            viz_node,
        ]
    )
