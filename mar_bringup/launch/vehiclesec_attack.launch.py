import os

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


def get_adversaries(context):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("mar_bringup"), "launch", "nodes"
                    ),
                    "/adversary.launch.py",
                ]
            ),
            launch_arguments={
                "adversary_name": f"adversary{i}",
                "attack_agent_name": f"agent{i}",
            }.items(),
        )
        for i in range(int(context.launch_configurations["n_adversaries"]))
    ]


def generate_launch_description():

    # base_nodes = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(get_package_share_directory("mar_bringup"), "launch"),
    #             "/_vehiclesec_base.launch.py",
    #         ]
    #     )
    # )

    n_adversaries_launch_arg = DeclareLaunchArgument("n_adversaries", default_value="4")

    adversaries = OpaqueFunction(function=get_adversaries)

    is_coordinated_launch_arg = DeclareLaunchArgument("is_coordinated")

    is_coordinated = LaunchConfiguration("is_coordinated")  # 2

    adversary_coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch", "nodes"
                ),
                "/adversary_coordinator.launch.py",
            ]
        ),
        condition=IfCondition(is_coordinated),  # only on this condition
    )

    return LaunchDescription(
        [
            # base_nodes,
            n_adversaries_launch_arg,
            adversaries,
            is_coordinated_launch_arg,
            adversary_coordinator,
        ]
    )
