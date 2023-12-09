import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "simulator",
        "carla_point_simulator.yml",
    )

    # HACK: this is a bit hacky.....
    # can we use the launch config variable??
    n_infrastructure_agents = LaunchConfiguration("n_infrastructure_agents")
    n_agents = 10
    agent_remaps_configs = [
        LaunchConfiguration(f"agent_{i}_remap", default=f"/agent{i}/detections")
        for i in range(n_agents)
    ]

    return LaunchDescription(
        [
            Node(
                package="mar_carla_sim",
                executable="point_simulator",
                name="simulator",
                parameters=[sim_config],
                arguments=["--ros-args", "--log-level", "INFO"],
                remappings=[
                    *[
                        (f"/agent{i}/detections", remap)
                        for i, remap in enumerate(agent_remaps_configs)
                    ]
                ],
            ),
        ]
    )
