import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adversary_name = LaunchConfiguration("adversary_name")
    attack_agent_name = LaunchConfiguration("attack_agent_name")  # the agent to attack
    attack_is_coordinated = LaunchConfiguration("attack_is_coordinated")
    output_new_topic = LaunchConfiguration("output_new_topic")

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
        parameters=[
            adversary_config,
            {
                "attack_agent_name": attack_agent_name,
                "attack_is_coordinated": attack_is_coordinated,
            },
        ],
        remappings=[("output", output_new_topic)],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([adversary_node])
