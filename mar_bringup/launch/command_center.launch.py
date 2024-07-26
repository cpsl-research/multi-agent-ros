from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # trust estimation
    cc_trust_node = Node(
        package="trust",
        executable="multi_agent_trust",
        namespace="mate",
        name="trust_estimator",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # sensor fusion
    cc_fusion_node = Node(
        package="tracking",
        executable="multi_platform_boxtracker3d",
        namespace="command_center",
        name="tracker",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([cc_trust_node, cc_fusion_node])
