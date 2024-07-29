import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # TODO: script this
    flir_camera_parameters = {
        f"cam_{i}_parameter_file": PathJoinSubstitution(
            [
                FindPackageShare("mar_bringup"),
                "config",
                "cameras",
                "north-building-four-cameras",
                f"camera-{i}",
                "parameters.yaml",
            ]
        )
        for i in range(1, 5)
    }

    flir_camera_calibration = {
        f"cam_{i}_camerainfo_url": "file:///"
        + os.path.join(
            get_package_share_directory("mar_bringup"),
            "config",
            "cameras",
            "north-building-four-cameras",
            f"camera-{i}",
            "calibration.yaml",
        )
        for i in range(1, 5)
    }

    serials = {1: "'22395929'", 2: "'22395953'", 3: "'24131919'", 4: "'24172037'"}
    flir_camera_serials = {f"cam_{i}_serial": serials[i] for i in range(1, 5)}

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
                                "flir_multicam_driver_node.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    **flir_camera_parameters,
                    **flir_camera_calibration,
                    **flir_camera_serials,
                }.items(),
            )
        ]
    )

    return LaunchDescription([flir_include])
