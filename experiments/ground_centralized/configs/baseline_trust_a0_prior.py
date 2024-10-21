"""
A baseline scenario with trust estimation but without any adversaries
"""

_base_ = ["./baseline_algorithms.py"]

rosbag_writer = {
    "bag_name": "baseline_trust",
    "algorithms": {
        "other_hooks": [
            {
                "type": "TrustFusionRosbagHook",
                "hook": {
                    "type": "TrustFusionHook",
                    "estimation_hook": {
                        "type": "TrustEstimationHook",
                        "n_calls_burnin": 2,
                        "model": {
                            "type": "TrustEstimator",
                            "measurement": {
                                "type": "ViewBasedPsm",
                                "assign_radius": 1.0,
                            },
                            "updater": {
                                "type": "TrustUpdater",
                                "agent_negativity_bias": 8.0,
                                "track_negativity_bias": 1.0,
                                "agent_negativity_threshold": 0.2,
                                "track_negativity_threshold": 0.4,
                                "prior_agents": {
                                    0: {"type": "trusted", "strength": 10.0}
                                },
                                "agent_propagator": {
                                    "type": "PriorInterpolationPropagator",
                                    "prior": {
                                        "type": "TrustBetaDistribution",
                                        "timestamp": 0.0,
                                        "identifier": "prior",
                                        "alpha": 0.5,
                                        "beta": 0.5,
                                    },
                                    "dt_return": 1,
                                },
                                "track_propagator": {
                                    "type": "PriorInterpolationPropagator",
                                    "prior": {
                                        "type": "TrustBetaDistribution",
                                        "timestamp": 0.0,
                                        "identifier": "prior",
                                        "alpha": 0.5,
                                        "beta": 0.5,
                                    },
                                    "dt_return": 1,
                                },
                            },
                        },
                    },
                    "fusion": {
                        "type": "TrackThresholdingTrustFusion",
                        "threshold_track_ignore": 0.40,
                    },
                    "verbose": False,
                },
            },
        ],
    },
}
