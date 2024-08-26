from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    trust_viz_node = Node(
        package="avtrust_bridge",
        namespace="trust",
        executable="visualizer",
        name="trust_visualizer",
    )

    return LaunchDescription([trust_viz_node])
