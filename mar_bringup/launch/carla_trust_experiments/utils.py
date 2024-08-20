from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def get_agents(context):
    """Set up the agents"""
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
                            "cte_agent_passive.launch.py",
                        ]
                    ),
                ]
            ),
            launch_arguments={
                "agent_name": f"agent{i}",
                "agent_type": "mobile.yaml" if i == 0 else "static.yaml",
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
