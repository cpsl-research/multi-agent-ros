#!/usr/bin/env zsh

set -e

N_MCS=${1:?"missing arg 1 for N_MCS"}

SEP="======================================================"

filename="configs/mc_tuning/mc_adv_datasets.py"

echo "$SEP\nRunning $filename\n$SEP"
python write_carla_dataset_mc.py $filename $N_MCS