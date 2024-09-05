_base_ = ["../baseline_trust_a0_prior.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/baseline_intersection",
    "bag_name": "baseline_intersection_2",
    "loader": {"scene_idx": 2},
}
