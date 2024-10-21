"""
A scenario demonstrating a false positive attack on agent 0's
lidar-based detections with markov propagation
"""

_base_ = ["../baselines/baseline_scene_0.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/cte_case",
    "bag_name": "cte_case_0_attacked",
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
                            "seed": 5,
                            "propagator": {"type": "StaticPropagator"},
                            "manifest_fp": {
                                "type": "FalsePositiveManifest",
                                "exact_select": 4,
                                "x_sigma": [14, 3, 0],
                            },
                        },
                    },
                },
            }
        ],
    },
}
