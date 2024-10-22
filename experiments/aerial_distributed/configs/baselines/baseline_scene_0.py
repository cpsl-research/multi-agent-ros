_base_ = ["../baseline_trust.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/baseline_aerial",
    "bag_name": "baseline_aerial_0",
    "loader": {"scene_idx": 0},
}
