from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    agent_name = LaunchConfiguration('agent_name')

    agent_name_launch_arg = DeclareLaunchArgument(
        'agent_name',
        default_value='agent1'
    )

    agent_pipeline_launch_arg = DeclareLaunchArgument(
        'agent_pipeline',
        default_value='passive_agent.py'
    )

    agent_node = Node(
        package="mar_agents",
        executable="agent_passive",
        namespace=agent_name,
        name="agent",
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription(
        [
            agent_name_launch_arg,
            agent_pipeline_launch_arg,
            agent_node
        ]
    )