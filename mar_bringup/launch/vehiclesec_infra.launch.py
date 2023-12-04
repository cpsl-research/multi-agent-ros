import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def get_infra_agents(context):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("mar_bringup"), "launch"
                    ),
                    "/agent_passive.launch.py",
                ]
            ),
            launch_arguments={
                'agent_name': f'agent{i}',
                'agent_pipeline': 'passive_agent.py'
            }.items(),
        ) for i in range(int(context.launch_configurations['n_infrastructure_agents']))
    ]


def generate_launch_description():
    # n_infrastructure_agents = LaunchConfiguration('n_infrastructure_agents')
    n_infrastructure_agents_launch_arg = DeclareLaunchArgument(
        'n_infrastructure_agents',
        default_value="0"
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch"
                ),
                "/carla_point_simulator.launch.py",
            ]
        )
    )

    ego_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch"
                ),
                "/agent_passive.launch.py",
            ]
        ),
        launch_arguments={
            'agent_name': 'ego',
            'agent_pipeline': 'passive_agent.py'
        }.items()
    )

    inf_agents = OpaqueFunction(function=get_infra_agents)

    command_center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch"
                ),
                "/command_center.launch.py"
            ]
        ),
        launch_arguments={
            'cc_pipeline': 'command_center.py'
        }.items()
    )

    visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mar_bringup"), "launch"
                ),
                "/visualizer.launch.py",
            ]
        )
    )
    

    return LaunchDescription(
        [
            n_infrastructure_agents_launch_arg,
            simulator,
            visualizer,
            ego_agent,
            inf_agents,
            command_center,
        ]
    )