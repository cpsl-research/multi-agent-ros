import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    flir_camera_parameters = PathJoinSubstitution(
        [
            FindPackageShare("mar_bringup"),
            "config",
            "cameras",
            "north-building-four-cameras",
            "camera-1",
            "parameters.yaml",
        ]
    )

    flir_camera_calibration = "file:///" + os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "cameras",
        "north-building-four-cameras",
        "camera-1",
        "calibration.yaml",
    )

    flir_include = GroupAction(
        actions=[
            # SetRemap(src="", dst=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("mar_drivers"),
                                "launch",
                                "flir_singlecam_driver_node.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "camera_type": "blackfly_s",
                    "serial": "'24172037'",  # "'22395929'",  #"'24131919'",
                    "gev_scps_packet_size": "9000",
                    "parameter_file": flir_camera_parameters,
                    "camerainfo_url": flir_camera_calibration,
                }.items(),
            )
        ]
    )

    return LaunchDescription([flir_include])
