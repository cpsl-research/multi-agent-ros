"""
A configuration with all the adversary parameters in long form
and monte carlo substitutions added for random testing
"""

_base_ = ["../../../configs/baseline_algorithms.py"]

rosbag_writer = {
    "bag_folder": "/data/shared/CARLA/rosbags/cte_monte_carlo/",
    "bag_name": "mc-trial-__MCPARAMETER__DATETIME__",
    "loader": {"scene_idx": "__MCPARAMETER__RANDINT_0_3__"},
    "algorithms": {
        "perception_hooks": [
            {
                "type": "AdversaryRosbagHook",
                "hook": {
                    "type": "AdversaryHook",
                    "verbose": False,
                    "models": {
                        ("agent__MCPARAMETER__RANDINT_0_4__", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": "__MCPARAMETER__RANDINT_0_100__",
                            "enabled": "__MCPARAMETER__RANDCUT_0.30__",
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fp": {
                                "type": "FalsePositiveManifest",
                                "exact_select": "__MCPARAMETER__RANDINT_0_5__",
                                "x_sigma": [15, 3, 0],
                                "x_bias": [15, 0, 0],
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
                        ("agent__MCPARAMETER__RANDINT_0_3__", "lidar0"): {
                            "type": "AdversaryModel",
                            "seed": "__MCPARAMETER__RANDINT_0_100__",
                            "enabled": "__MCPARAMETER__RANDCUT_0.30__",
                            "propagator": {"type": "MarkovPropagator"},
                            "manifest_fn": {
                                "type": "FalseNegativeManifest",
                                "exact_select": "__MCPARAMETER__RANDINT_0_5__",
                            },
                        },
                    },
                },
            },
        ],
    },
}
