"""
A scenario demonstrating a false positive attack on agent 1's
and agent 2's lidar-based detections with markov propagation
"""

_base_ = ["../baselines/baseline_scene_1.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/cte_case",
    "bag_name": "cte_case_1_attacked",
    "algorithms": {
        "perception_hooks": [
            {
                "type": "AdversaryRosbagHook",
                "hook": {
                    "type": "AdversaryHook",
                    "verbose": False,
                    "models": {
                        ("agent0", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": 3,
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fp": {
                                "type": "FalsePositiveManifest",
                                "exact_select": 2,
                                "x_sigma": [10, 3, 0],
                            },
                        },
                        ("agent1", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": 4,
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fp": {
                                "type": "FalsePositiveManifest",
                                "exact_select": 2,
                                "x_sigma": [14, 3, 0],
                                "x_bias": [30, 0, 0],
                            },
                        },
                    },
                },
            }
        ],
    },
}
