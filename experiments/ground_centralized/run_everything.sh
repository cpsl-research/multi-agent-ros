#!/usr/bin/env zsh

set -e

./write_cte_baselines.sh
./write_cte_cases.sh
./write_cte_mc.sh 50
./run_cte_trust_tuning.sh configs/mc_tuning/mc_trust_tune_neg_bias.py 30 /data/shared/CARLA/rosbags/cte_monte_carlo
./run_cte_trust_tuning.sh configs/mc_tuning/mc_trust_tune_neg_bias_threshold.py 30 /data/shared/CARLA/rosbags/cte_monte_carlo
