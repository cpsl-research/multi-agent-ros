import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_adversaries


def generate_launch_description():
    n_advs_launch_arg = DeclareLaunchArgument("n_adversaries", default_value="1")
    attack_coordinated_launch_arg = DeclareLaunchArgument(
        "attack_is_coordinated", default_value="False"
    )
    adv_nodes = OpaqueFunction(function=get_adversaries)

    return LaunchDescription(
        [
            n_advs_launch_arg,
            attack_coordinated_launch_arg,
            adv_nodes,
        ]
    )
