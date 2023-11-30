import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    agent_name = LaunchConfiguration('agent_name')
    agent_pipeline = LaunchConfiguration('agent_pipeline')

    agent_name_launch_arg = DeclareLaunchArgument(
        'agent_name',
        default_value='agent1'
    )

    agent_pipeline_launch_arg = DeclareLaunchArgument(
        'agent_pipeline',
        default_value='passive_agent_pipeline.yml'
    )

    # agent_config = PathJoinSubstitution([
    #     get_package_share_directory("mar_bringup"),
    #     "config",
    #     "agent",
    #     agent_pipeline,
    # ])

    agent_node = Node(
        package="mar_agents",
        executable="agent_passive",
        namespace=agent_name,
        name="agent",
        # parameters=[agent_config],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription(
        [
            agent_name_launch_arg,
            agent_pipeline_launch_arg,
            agent_node
        ]
    )