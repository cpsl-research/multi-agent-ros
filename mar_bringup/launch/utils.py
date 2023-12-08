import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_adversaries(context):
    """Set up the adversaries to attack certain agents

    If the adversary is uncoordinated, remap detections to the adversary...
    If the adversary is coordinated, remap tracks to the adversary
    """
    output_remapping = {}

    for i in range(int(context.launch_configurations["n_adversaries"])):
        if bool(context.launch_configurations["attack_is_coordinated"]):
            output_remapping[i] = {f"/adversary{i}/outputs", f"/agent{i}/tracks"}
        else:
            output_remapping[i] = {f"/adversary{i}/outputs", f"/agent{i}/detections"}

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
                "attack_is_coordinated": context.launch_configurations[
                    "attack_is_coordinated"
                ],
                "output_remapping": output_remapping[i],
            }.items(),
        )
        for i in range(int(context.launch_configurations["n_adversaries"]))
    ]


def get_infra_agents(context):
    """Set up the infrastructure agents

    For the vehiclesec experiments, these could be compromised.
    In the case of compromised agents, the topics will be remapped
    to the adversaries.

    *** ASSUMPTION: "adversary-2" attacks "agent-2" ***
    """
    output_remapping = {}

    if bool(context.launch_configurations["attacked"]):
        if bool(context.launch_configurations["attack_is_coordinated"]):
            for i in range(int(context.launch_configurations["n_adversaries"])):
                output_remapping[i] = {f"/agent{i}/tracks": f"/adversary{i}/tracks"}

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("mar_bringup"), "launch", "nodes"
                    ),
                    "/agent_passive.launch.py",
                ]
            ),
            launch_arguments={
                "agent_name": f"agent{i}",
                "agent_pipeline": "passive_agent.py",
                "output_remapping": output_remapping[i],
            }.items(),
        )
        for i in range(int(context.launch_configurations["n_infrastructure_agents"]))
    ]


def get_simulator(context):
    """Set up the simulator

    This function is here in case we need to do output
    remapping based on the parameters of the experiments.

    *** ASSUMPTION: "adversary-2" attacks "agent-2" ***
    """

    output_remapping = {}

    if bool(context.launch_configuration["attacked"]):
        if not bool(context.launch_configuration["attack_is_coordinated"]):
            for i in range(int(context.launch_configurations["n_adversaries"])):
                output_remapping[f"/agent{i}/detections"] = f"/adversary{i}/detections"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("mar_bringup"), "launch", "nodes"
                    ),
                    "/carla_point_simulator.launch.py",
                ]
            ),
            launch_arguments={
                "output_remapping": output_remapping,
            },
        )
    ]
