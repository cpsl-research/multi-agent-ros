import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # saving folder
    run_name = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    output_folder = os.path.join("outputs", run_name)
    os.makedirs(output_folder)
    symlink = os.path.join("outputs", "last_run")
    if os.path.exists(symlink):
        os.remove(symlink)
    os.symlink(src=run_name, dst=symlink, target_is_directory=True)

    # run nodes
    base_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("mar_bringup"), "launch"),
                "/_base.launch.py",
            ]
        ),
        launch_arguments={
            "output_folder": output_folder,
        }.items(),
    )

    return LaunchDescription([base_nodes])
