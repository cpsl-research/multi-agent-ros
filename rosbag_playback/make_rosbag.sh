#!/usr/bin/env zsh

set -e

rm -rf /data/shared/CARLA/rosbags/carla_dataset

ros2 launch avstack_rosbag carla_dataset_writer.launch.py