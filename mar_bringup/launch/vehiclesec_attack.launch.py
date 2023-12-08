import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_adversaries


def generate_launch_description():
    attack_is_coordinated = LaunchConfiguration("attack_is_coordinated")

    n_adversaries_launch_arg = DeclareLaunchArgument("n_adversaries")
    is_coordinated_launch_arg = DeclareLaunchArgument("attack_is_coordinated")

    base_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("mar_bringup"), "launch"),
                "/_vehiclesec_base.launch.py",
            ]
        )
    )

    adversaries = OpaqueFunction(function=get_adversaries)

    adversary_coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch", "nodes"
                ),
                "/adversary_coordinator.launch.py",
            ]
        ),
        condition=IfCondition(attack_is_coordinated),  # only on this condition
    )

    return LaunchDescription(
        [
            base_nodes,
            n_adversaries_launch_arg,
            adversaries,
            is_coordinated_launch_arg,
            adversary_coordinator,
        ]
    )