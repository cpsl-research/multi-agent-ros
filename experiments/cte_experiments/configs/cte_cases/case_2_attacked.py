"""
A scenario demonstrating a false negative attack on agent 3's
lidar-based detections
"""

_base_ = ["../baselines/baseline_scene_1.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/cte_case",
    "bag_name": "cte_case_2_attacked",
    "algorithms": {
        "perception_hooks": [
            {
                "type": "AdversaryRosbagHook",
                "hook": {
                    "type": "AdversaryHook",
                    "verbose": False,
                    "models": {
                        ("agent3", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": 4,
                            # "manifest_fn": {
                            #     "type": "FalseNegativeManifest",
                            #     "exact_select": 3,
                            # },
                            "manifest_tr": {
                                "type": "TranslationManifest",
                                "exact_select": 2,
                            },
                            "propagator": {"type": "MarkovPropagator"},
                        },
                    },
                },
            }
        ],
    },
}
