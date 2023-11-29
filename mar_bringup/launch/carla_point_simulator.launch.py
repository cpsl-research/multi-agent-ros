import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    # mar_ns = LaunchConfiguration('mar_ns')

    sim_config = os.path.join(
        get_package_share_directory("mar_bringup"),
        "config",
        "simulator",
        "carla_point_simulator.yml",
    )

    srv_msg = "{pipeline: 'standard'}"
    spawn_agent = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            # mar_ns,
            '/spawn_agent ',
            'mar_msgs/srv/SpawnAgent ',
            srv_msg,
        ]],
        shell=True
    )

    return LaunchDescription(
        [
            Node(
                package="mar_carla_sim",
                executable="point_simulator",
                name="simulator",
                parameters=[sim_config],
                arguments=['--ros-args', '--log-level', 'INFO'],
            ),
            RegisterEventHandler(
                OnProcessIO(
                    target_action=spawn_agent,
                    on_stdout=lambda event: LogInfo(
                        msg='Spawn request says "{}"'.format(
                            event.text.decode().strip())
                    )
                )
            ),
        ]
    )