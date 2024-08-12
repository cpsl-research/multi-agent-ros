import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_agents, get_command_center, get_trust_estimator, get_viz


def generate_launch_description():
    n_agents_launch_arg = DeclareLaunchArgument("n_agents", default_value="4")
    trust_enabled = DeclareLaunchArgument("trust_enabled", default_value="False")
    viz_config_launch_arg = DeclareLaunchArgument(
        "viz_config", default_value="multi_agent_config.rviz"
    )

    agent_nodes = OpaqueFunction(function=get_agents)
    cc_node = OpaqueFunction(function=get_command_center)
    trust_node = OpaqueFunction(function=get_trust_estimator)
    viz_node = OpaqueFunction(function=get_viz)

    return LaunchDescription(
        [
            n_agents_launch_arg,
            trust_enabled,
            viz_config_launch_arg,
            agent_nodes,
            cc_node,
            trust_node,
            viz_node,
        ]
    )
