import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adversary_name = LaunchConfiguration("adversary_name")
    attack_agent_name = LaunchConfiguration("attack_agent_name")  # the agent to attack

    adversary_name_launch_arg = DeclareLaunchArgument(
        "adversary_name", default_value="adversary1"
    )

    attack_agent_name_launch_arg = DeclareLaunchArgument(
        "attack_agent_name", default_value="agent1"
    )

    adversary_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "adversary",
        "adversary.yml",
    )

    adversary_node = Node(
        package="mar_adv",
        executable="adversary",
        namespace=adversary_name,
        name="adversary",
        parameters=[adversary_config, {"attack_agent_name": attack_agent_name}],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription(
        [adversary_name_launch_arg, attack_agent_name_launch_arg, adversary_node]
    )
