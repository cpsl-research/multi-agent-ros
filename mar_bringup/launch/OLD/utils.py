import json
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def log_launch_metadata(output_folder, context):
    """Log the launch metadata to a file"""
    meta_file = "launch_metadata.json"
    os.makedirs(output_folder, exist_ok=True)
    l_config = context.launch_configurations
    metadata = {
        "n_adversaries": int(l_config["n_adversaries"]),
        "attack_is_coordinated": l_config["attack_is_coordinated"] == "True",
        "n_agents": int(l_config["n_agents"]),
    }
    with open(os.path.join(output_folder, meta_file), "w") as f:
        json.dump(metadata, f)


def get_adversaries(output_folder, context):
    """Set up the adversaries to attack certain agents

    If the adversary is uncoordinated, remap detections to the adversary...
    If the adversary is coordinated, remap tracks to the adversary
    """
    n_adv = int(context.launch_configurations["n_adversaries"])
    output_remapping = {}

    for i in range(1, 1 + int(context.launch_configurations["n_adversaries"])):
        if context.launch_configurations["attack_is_coordinated"] == "True":
            output_remapping[i] = {f"/agent{i}/tracks"}
        else:
            output_remapping[i] = {f"/agent{i}/detections"}

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
                "attack_coord_topic": "/adversary_coordinator/attack_directive",
                "attack_is_coordinated": context.launch_configurations[
                    "attack_is_coordinated"
                ],
                "output_new_topic": output_remapping[i],
                "output_folder": output_folder,
            }.items(),
        )
        for i in range(1, 1 + n_adv)
    ]


def attacked(context):
    if ("n_adversaries" in context.launch_configurations) and (
        int(context.launch_configurations["n_adversaries"]) > 0
    ):
        return True
    else:
        return False


def get_agents(output_folder, context):
    """Set up the agents

    *** ASSUMPTION: "adversary-i" attacks "agent-i" ***
    """
    n_agents = int(context.launch_configurations["n_agents"])
    output_remapping = {i: "tracks" for i in range(0, n_agents)}  # default is the same

    if attacked(context):
        if context.launch_configurations["attack_is_coordinated"] == "True":
            for i in range(0, int(context.launch_configurations["n_adversaries"])):
                output_remapping[i] = f"/adversary{i}/tracks"

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
                "track_new_topic": output_remapping[i],
                "output_folder": output_folder,
            }.items(),
        )
        for i in range(0, n_agents)
    ]


def get_simulator(output_folder, context):
    """Set up the simulator

    This function is here in case we need to do output
    remapping based on the parameters of the experiments.

    *** ASSUMPTION: "adversary-2" attacks "agent-2" ***
    """

    output_remapping = {}

    if attacked(context):
        if context.launch_configurations["attack_is_coordinated"] == "False":
            for i in range(1, 1 + int(context.launch_configurations["n_adversaries"])):
                output_remapping[f"agent_{i}_remap"] = f"/adversary{i}/detections"

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
                "n_agents": context.launch_configurations["n_agents"],
                "output_folder": output_folder,
                **output_remapping,
            }.items(),
        )
    ]
