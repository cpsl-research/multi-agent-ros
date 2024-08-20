from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    tracking_metrics_node = Node(
        package="tracking",
        namespace="metrics",
        executable="metrics_evaluator",
        name="tracking_metrics",
        remappings=[
            ("/tracks", "/command_center/tracks_3d"),
            ("/truths", "/object_truth"),
        ],
    )

    return LaunchDescription([tracking_metrics_node])
