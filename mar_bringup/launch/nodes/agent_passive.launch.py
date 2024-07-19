from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_name = LaunchConfiguration("agent_name")
    track_new_topic = LaunchConfiguration("track_new_topic", default="tracks")
    output_folder = LaunchConfiguration("output_folder")

    # ==================================
    # spawn up all agent nodes
    # ==================================

    # perception
    percep_node = Node(
        package="percep_lidar",
        executable="mmdetection3d",
        namespace=agent_name,
        name="perception",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # tracking
    track_node = Node(
        package="mar_agents",
        executable="box_tracker",
        namespace=agent_name,
        name="tracker",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # fov estimation
    fov_node = Node(
        package="mar_agents",
        executable="fov_estimator",
        namespace=agent_name,
        name="fov_estimator",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([percep_node, track_node, fov_node])
