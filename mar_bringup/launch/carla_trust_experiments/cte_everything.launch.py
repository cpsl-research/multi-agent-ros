from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # launch arguments
    n_agents_launch_arg = DeclareLaunchArgument("n_agents", default_value="4")
    n_advs_launch_arg = DeclareLaunchArgument("n_adversaries", default_value="1")
    att_is_coordinated_launch_arg = DeclareLaunchArgument(
        "attack_is_coordinated", default_value="False"
    )
    viz_config_launch_arg = DeclareLaunchArgument(
        "viz_config", default_value="multi_agent_config.rviz"
    )

    # agents
    agent_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("mar_bringup"),
                        "launch",
                        "carla_trust_experiments",
                        "cte_all_agents.launch.py",
                    ]
                )
            ]
        ),
    )

    # trust
    trust_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("mar_bringup"),
                        "launch",
                        "carla_trust_experiments",
                        "cte_trust.launch.py",
                    ]
                )
            ]
        ),
    )

    # metrics
    metric_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("mar_bringup"),
                        "launch",
                        "carla_trust_experiments",
                        "cte_metrics.launch.py",
                    ]
                )
            ]
        ),
    )

    # adversaries
    adv_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("mar_bringup"),
                        "launch",
                        "carla_trust_experiments",
                        "cte_all_adversaries.launch.py",
                    ]
                )
            ]
        ),
    )

    return LaunchDescription(
        [
            n_agents_launch_arg,
            n_advs_launch_arg,
            att_is_coordinated_launch_arg,
            viz_config_launch_arg,
            agent_nodes,
            trust_nodes,
            metric_nodes,
            adv_nodes,
        ]
    )
