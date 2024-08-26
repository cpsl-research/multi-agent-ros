from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    metrics_viz_node = Node(
        package="avstack_metrics",
        namespace="metrics",
        executable="metrics_visualizer",
        name="visualizer",
    )

    return LaunchDescription([metrics_viz_node])
