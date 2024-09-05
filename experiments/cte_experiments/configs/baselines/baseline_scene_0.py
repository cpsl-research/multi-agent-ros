_base_ = ["../baseline_trust.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/baseline_intersection",
    "bag_name": "baseline_intersection_0",
    "loader": {"scene_idx": 0},
}
