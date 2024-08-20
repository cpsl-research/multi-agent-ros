from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    metrics_viz_node = Node(
        package="tracking",
        namespace="metrics",
        executable="metrics_visualizer",
        name="metrics_visualizer",
    )

    return LaunchDescription([metrics_viz_node])
