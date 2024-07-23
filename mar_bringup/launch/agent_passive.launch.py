from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_name = LaunchConfiguration("agent_name")

    # ==================================
    # spawn up all agent nodes
    # ==================================

    # perception
    percep_node = Node(
        package="percep_lidar",
        executable="mmdetection3d",
        namespace=agent_name,
        name="perception",
        remappings=[
            ("point_cloud", "lidar0"),
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # tracking
    track_node = Node(
        package="tracking",
        executable="boxtracker3d",
        namespace=agent_name,
        name="tracker",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # fov estimation
    fov_node = Node(
        package="fov_estimator",
        executable="lidar_concave_hull",
        namespace=agent_name,
        name="fov_estimator",
        remappings=[
            ("point_cloud", "lidar0"),
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([percep_node, track_node, fov_node])
