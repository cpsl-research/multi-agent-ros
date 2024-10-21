#!/usr/bin/env zsh

set -e

HOOK=${1:?"Missing input filepath to the hook config"}
N_MCS=${2:?"Missing input for the number of MC trials"}
ROSBAG_DIR=${3:?"Missing input for rosbag dir (e.g., /data/shared/CARLA/rosbags/cte_monte_carlo)"}

SEP="======================================================"

echo "$SEP\nRunning $HOOK\n$SEP"
python run_trust_as_hook_on_mc_cases.py $ROSBAG_DIR $HOOK $N_MCS