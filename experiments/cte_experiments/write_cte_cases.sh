#!/usr/bin/env zsh

set -e

CASEDIR="configs/cte_cases"

SEP="======================================================"

for filename in $CASEDIR/*.py; do
    echo "$SEP\nRunning $filename\n$SEP"
    python write_carla_dataset.py $filename
done