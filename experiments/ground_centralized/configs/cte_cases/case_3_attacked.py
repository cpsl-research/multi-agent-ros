"""
A scenario demonstrating a translation attack on agent 0's
lidar-based detections. Unfortunately this situation is
challenging due to the lack of suitable objects in the
field of view for agent 1
"""

_base_ = ["../baselines/baseline_scene_2.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/cte_case",
    "bag_name": "cte_case_3_attacked",
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
                            "seed": 4,
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_tr": {
                                "type": "TranslationManifest",
                                "exact_select": 2,
                            },
                        },
                    },
                },
            }
        ],
    },
}
