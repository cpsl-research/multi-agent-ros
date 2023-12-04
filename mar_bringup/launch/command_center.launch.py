from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    cc_pipeline_launch_arg = DeclareLaunchArgument(
        'cc_pipeline',
        default_value='command_center.py'
    )

    cc_broker_node = Node(
        package="mar_agents",
        executable="command_center_broker",
        namespace="command_center",
        name="command_center",
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    cc_node = Node(
        package="mar_agents",
        executable="command_center",
        namespace="command_center",
        name="command_center",
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription(
        [
            cc_pipeline_launch_arg,
            cc_broker_node,
            # cc_node,
        ]
    )