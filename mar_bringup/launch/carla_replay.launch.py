import launch


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch.actions.ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    "-l",
                    "/data/shared/CARLA/rosbags/carla_dataset",
                ],
                output="screen",
            )
        ]
    )
