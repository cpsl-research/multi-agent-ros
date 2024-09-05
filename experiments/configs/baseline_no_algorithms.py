rosbag_writer = {
    "type": "DatasetRosWriter",
    "bag_folder": "/data/shared/CARLA/rosbags/",
    "bag_name": "baseline_no_algorithms",
    "loader": {
        "type": "DatasetLoader",
        "scene_manager": {
            "type": "CarlaScenesManager",
            "data_dir": "/data/shared/CARLA/multi-agent-intersection",
        },
        "scene_idx": 0,
        "i_frame_start": 4,
        "n_frames_trim": 10,
        "n_frames_max": 700,
    },
    "algorithms": {
        "type": "DatasetAlgorithms",
        "run_perception": False,
        "run_tracking": False,
        "run_fusion": False,
        "perception_hooks": [],
        "tracking_hooks": [],
        "fusion_hooks": [],
        "other_hooks": [],
    },
    "rosconverter": {"type": "DatasetRosConverter"},
}
