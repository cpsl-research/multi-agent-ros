from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    agent_name = LaunchConfiguration("agent_name")
    agent_type = LaunchConfiguration("agent_type")
    det_topic = LaunchConfiguration("detection_topic")
    trk_topic = LaunchConfiguration("track_topic")

    agent_config = PathJoinSubstitution(
        [
            get_package_share_directory("mar_bringup"),
            "config",
            "agent",
            agent_type,
        ]
    )

    # perception
    percep_node = Node(
        package="avstack_bridge",
        executable="mmdetection3d",
        namespace=agent_name,
        name="perception",
        parameters=[agent_config],
        remappings=[
            ("point_cloud", "lidar0"),
            ("detections_3d", det_topic),
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # tracking
    track_node = Node(
        package="avstack_bridge",
        executable="boxtracker3d",
        namespace=agent_name,
        name="tracking",
        parameters=[
            agent_config,
            {
                "tracking_in_global": True,
            },
        ],
        remappings=[
            ("detections_3d", det_topic),
            ("tracks_3d", trk_topic),
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # fov estimation
    fov_node = Node(
        package="avstack_bridge",
        executable="lidar_concave_hull",
        namespace=agent_name,
        name="fov_estimator",
        parameters=[agent_config],
        remappings=[
            ("point_cloud", "lidar0"),
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([percep_node, track_node, fov_node])
