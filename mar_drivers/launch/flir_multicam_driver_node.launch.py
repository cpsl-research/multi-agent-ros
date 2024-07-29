# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# Exposure controller parameters
exposure_controller_parameters = {
    'brightness_target': 50,  # from 0..255
    'brightness_tolerance': 20,  # when to update exposure/gain
    'max_exposure_time': 15000,  # usec
    'min_exposure_time': 5000,  # usec
    'max_gain': 29.9,
    'gain_priority': False,
}

# Parameters shared by all cameras
shared_cam_parameters = {
    'debug': False,
    'quiet': True,
    'buffer_queue_size': 1,
    'compute_brightness': True,
    'pixel_format': 'BGR8',
    'exposure_auto': 'Off',
    'exposure_time': 10000,  # not used under auto exposure
    'gain_auto': 'Off',
    'balance_white_auto': 'Continuous',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}

# Parameters for the primary camera
primary_cam_parameters = {
    **shared_cam_parameters,
    'trigger_mode': 'Off',
    'line1_selector': 'Line1',
    'line1_linemode': 'Output',
    'line2_selector': 'Line2',
    'line2_v33enable': True,
}

# Parameters for the secondary camera
secondary_cam_parameters = {
    **shared_cam_parameters,
    'trigger_mode': 'On',
    'trigger_source': 'Line3',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
}

# camera_params = {
#     "blackfly_s": {
#         'buffer_queue_size': 10,
#         'debug': False,
#         'compute_brightness': True,
#         'device_link_throughput_limit': 380000000,
#         'dump_node_map': False,
#         'adjust_timestamp': True,
#         'gain_auto': 'Off',
#         'gain': 0,
#         'gev_scps_packet_size': 9000,
#         'exposure_auto': 'Off',
#         'exposure_time': 9000,
#         'frame_rate_auto': 'Off',
#         'frame_rate': 22.0,
#         'frame_rate_enable': True,
#         'line2_selector': 'Line2',
#         'line2_v33enable': False,
#         'line3_selector': 'Line3',
#         'line3_linemode': 'Input',
#         'trigger_selector': 'FrameStart',
#         'trigger_mode': 'On',
#         'trigger_source': 'Line3',
#         'trigger_delay': 30.0,
#         'trigger_overlap': 'ReadOut',
#         'chunk_mode_active': True,
#         'chunk_selector_frame_id': 'FrameID',
#         'chunk_enable_frame_id': True,
#         'chunk_selector_exposure_time': 'ExposureTime',
#         'chunk_enable_exposure_time': True,
#         'chunk_selector_gain': 'Gain',
#         'chunk_enable_gain': True,
#         'chunk_selector_timestamp': 'Timestamp',
#         'chunk_enable_timestamp': True,
#     }
# }


def make_camera_node(camera_name, camera_type, serial, parameter_file, camerainfo_url):
    # parameter_file = PathJoinSubstitution(
    #     [FindPackageShare('spinnaker_camera_driver'), 'config', camera_type + '.yaml']
    # )

    # set camera params
    camera_params = {
        **exposure_controller_parameters,
        **shared_cam_parameters,
    }
    if camera_name == "camera_1":
        camera_params = {**camera_params, **primary_cam_parameters}
    else:
        camera_params = {**camera_params, **primary_cam_parameters}

    # compose nodes
    node = ComposableNode(
        package='spinnaker_camera_driver',
        plugin='spinnaker_camera_driver::CameraDriver',
        name=camera_name,
        parameters=[
            camera_params,
            {
                'parameter_file': parameter_file,
                'serial_number': serial,
                'camerainfo_url': camerainfo_url,
            }
        ],
        remappings=[
            ('~/control', '/exposure_control/control'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    return node


def launch_setup(context, n_cameras=4, *args, **kwargs):
    """Create multiple camera."""
    container = ComposableNodeContainer(
        name='stereo_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            #
            # These camera nodes run independently from each other,
            # but in the same address space
            #
            make_camera_node(
                LaunchConfig(f'cam_{i}_name'),
                LaunchConfig(f'cam_{i}_type').perform(context),
                LaunchConfig(f'cam_{i}_serial'),
                LaunchConfig(f'cam_{i}_parameter_file'),
                LaunchConfig(f'cam_{i}_camerainfo_url'),
            )
            for i in range(1, n_cameras+1)
        ],
        output='screen',
    )  # end of container
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    n_cameras = 4
    return LaunchDescription(
        [
            *[
                v for i in range(1, n_cameras+1) for v in [
                    LaunchArg(
                    f'cam_{i}_name',
                    default_value=[f'camera_{i}'],
                    description='camera name (ros node name) of camera ',
                    ),
                    LaunchArg(f'cam_{i}_type', default_value='blackfly_s', description='type of camera'),
                    LaunchArg(
                        f'cam_{i}_serial',
                        default_value='',
                        description='FLIR serial number of camera (in quotes!!)',
                    ),
                    LaunchArg(
                        f'cam_{i}_parameter_file',
                        default_value='',
                        description='parameter file for camera',
                    ),
                    LaunchArg(
                        f'cam_{i}_camerainfo_url',
                        default_value='',
                        description='path to camera calibration file',
                    )
                ]
            ],
            OpaqueFunction(function=launch_setup),
        ]
    )