import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_metrics_visualizer, get_trust_viz, get_viz


def generate_launch_description():
    viz_config_launch_arg = DeclareLaunchArgument(
        "viz_config", default_value="multi_agent_config.rviz"
    )
    n_agents_launch_arg = DeclareLaunchArgument("n_agents", default_value="4")
    viz_node = OpaqueFunction(function=get_viz)
    trust_viz_node = OpaqueFunction(function=get_trust_viz)
    metrics_viz = OpaqueFunction(function=get_metrics_visualizer)

    return LaunchDescription(
        [
            viz_config_launch_arg,
            n_agents_launch_arg,
            viz_node,
            trust_viz_node,
            metrics_viz,
        ]
    )
