"""
A baseline scenario with trust estimation but without any adversaries
"""

_base_ = ["./baseline_algorithms_distributed.py"]

rosbag_writer = {
    "bag_name": "baseline_trust_distributed",
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
                                "assign_radius": 3.0,
                                "n_frames_viewable_bias": 3,
                                "n_frames_viewable_scaling": 2,
                            },
                            "updater": {
                                "type": "TrustUpdater",
                                "agent_negativity_bias": 6.0,
                                "track_negativity_bias": 1.0,
                                "agent_negativity_threshold": 0.3,
                                "track_negativity_threshold": 0.4,
                                "prior_agents": {
                                    0: {"type": "untrusted", "strength": 1.0}
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