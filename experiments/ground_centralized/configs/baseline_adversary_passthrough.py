"""
A configuration with all the adversary parameters in long form
but with the parameters set such that the adversary is deactivated
"""

_base_ = ["../../configs/baseline_algorithms.py"]

rosbag_writer = {
    "bag_name": "adversary_passthrough",
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
                            "seed": 1,
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fp": {
                                "type": "FalsePositiveManifest",
                                "fp_poisson": None,
                                "min_select": 0,
                                "max_select": 0,
                            },
                        },
                    },
                },
            },
            {
                "type": "AdversaryRosbagHook",
                "hook": {
                    "type": "AdversaryHook",
                    "verbose": False,
                    "models": {
                        ("agent0", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": 1,
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fp": {
                                "type": "FalseNegativeManifest",
                                "fp_poisson": None,
                                "min_select": 0,
                                "max_select": 0,
                            },
                        },
                    },
                },
            },
            {
                "type": "AdversaryRosbagHook",
                "hook": {
                    "type": "AdversaryHook",
                    "verbose": False,
                    "models": {
                        ("agent0", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": 1,
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fp": {
                                "type": "TranslationManifest",
                                "fp_poisson": None,
                                "min_select": 0,
                                "max_select": 0,
                            },
                        },
                    },
                },
            },
        ],
    },
}
