from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def attacked(context):
    if ("n_adversaries" in context.launch_configurations) and (
        int(context.launch_configurations["n_adversaries"]) > 0
    ):
        return True
    else:
        return False


def get_agents(context):
    """Set up the agents

    *** ASSUMPTION: "adversary-i" attacks "agent-i" ***
    """
    n_agents = int(context.launch_configurations["n_agents"])
    det_topic = {i: "detections_3d" for i in range(n_agents)}
    trk_topic = {i: "tracks_3d" for i in range(0, n_agents)}

    # update the topics for detections and tracks
    if attacked(context):
        for i in range(int(context.launch_configurations["n_adversaries"])):
            if context.launch_configurations["attack_is_coordinated"] == "True":
                trk_topic[i] = f"/adversary{i}/tracks_3d"
            else:
                det_topic[i] = f"/adversary{i}/detections_3d"

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
                "agent_name": f"agent{i}",
                "agent_type": "mobile.yaml" if i == 0 else "static.yaml",
                "detection_topic": det_topic[i],
                "track_topic": trk_topic[i],
            }.items(),
        )
        for i in range(0, n_agents)
    ]


def get_command_center(context):
    """Set up the command center"""
    n_agents = int(context.launch_configurations["n_agents"])

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
                            "cte_command_center.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "n_agents": str(n_agents),
            }.items(),
        )
    ]


def get_metrics_evaluator(context):
    """Set up the metrics evaluator"""
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
                            "cte_metrics_evaluator.launch.py",
                        ]
                    )
                ]
            ),
        )
    ]


def get_metrics_visualizer(context):
    """Set up the metrics evaluator"""
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
                            "cte_metrics_visualizer.launch.py",
                        ]
                    )
                ]
            ),
        )
    ]


def get_trust_estimator(context):
    """Set up the trust estimator"""
    n_agents = int(context.launch_configurations["n_agents"])

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
                            "cte_trust_estimator.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "n_agents": str(n_agents),
            }.items(),
        )
    ]


def get_trust_viz(context):
    """Set up the trust visualizer"""
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
                            "cte_trust_viz.launch.py",
                        ]
                    )
                ]
            ),
        )
    ]


def get_viz(context):
    """Set up the visualizer"""

    viz_config = str(context.launch_configurations["viz_config"])

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


def get_adversaries(context):
    """Set up the adversaries

    If the adversary is uncoordinated, remap detections to the adversary...
    If the adversary is coordinated, remap tracks to the adversary
    """
    n_adv = int(context.launch_configurations["n_adversaries"])

    # set up the topic remappings
    input_remapping = {}
    output_remapping = {}
    for i in range(int(context.launch_configurations["n_adversaries"])):
        if context.launch_configurations["attack_is_coordinated"] == "True":
            input_remapping[i] = "tracks_3d"
            output_remapping[i] = f"/agent{i}/tracks_3d"
        else:
            input_remapping[i] = "detections_3d"
            output_remapping[i] = f"/agent{i}/detections_3d"

    # launch the adversaries
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
                            "cte_adversary.launch.py",
                        ]
                    ),
                ]
            ),
            launch_arguments={
                "adversary_name": f"adversary{i}",
                "attack_agent_name": f"agent{i}",
                "attack_coord_topic": "/adversary_coordinator/attack_directive",
                "attack_is_coordinated": context.launch_configurations[
                    "attack_is_coordinated"
                ],
                "input_new_topic": input_remapping[i],
                "output_new_topic": output_remapping[i],
            }.items(),
        )
        for i in range(n_adv)
    ]
