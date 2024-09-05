#!/usr/bin/env zsh

BAGPATH=${1:?"Missing input argument for rosbag path"}

set -e

ros2 bag play \
    --loop \
    --delay=1 \
    --rate=1.0 \
    --clock=100 \
    --qos-profile-overrides-path qos_override.yaml \
    $BAGPATH
