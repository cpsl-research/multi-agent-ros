import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_trust_estimator, get_trust_viz


def generate_launch_description():
    n_agents_launch_arg = DeclareLaunchArgument("n_agents", default_value="4")
    trust_node = OpaqueFunction(function=get_trust_estimator)
    trust_viz_node = OpaqueFunction(function=get_trust_viz)

    return LaunchDescription(
        [
            n_agents_launch_arg,
            trust_node,
            trust_viz_node,
        ]
    )
