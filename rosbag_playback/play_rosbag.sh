#!/usr/bin/env zsh

set -e

ros2 bag play \
    --loop \
    --delay=1 \
    --rate=0.5 \
    --clock=100 \
    --qos-profile-overrides-path qos_override.yaml \
    '/data/shared/CARLA/rosbags/carla_dataset'