from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_name = LaunchConfiguration("agent_name")
    track_new_topic = LaunchConfiguration("track_new_topic", default="tracks")
    output_folder = LaunchConfiguration("output_folder")

    agent_node = Node(
        package="mar_agents",
        executable="agent_passive",
        namespace=agent_name,
        name="agent",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[{"namespace": agent_name, "output_folder": output_folder}],
        remappings=[("tracks", track_new_topic)],
    )

    return LaunchDescription([agent_node])
