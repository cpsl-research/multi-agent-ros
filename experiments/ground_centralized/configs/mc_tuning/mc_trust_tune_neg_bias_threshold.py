other_hooks = [
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
                        "type": "TrustUpdatcer",
                        "agent_negativity_bias": "__MCPARAMETER__RANDFLOAT_1_20__",
                        "track_negativity_bias": 1.0,
                        "agent_negativity_threshold": "__MCPARAMETER__RANDFLOAT_0.2_0.7__",
                        "track_negativity_threshold": 0.4,
                        "prior_agents": {0: {"type": "untrusted", "strength": 1.0}},
                        "agent_propagator": {
                            "type": "PriorInterpolationPropagator",
                            "prior": {
                                "type": "TrustBetaDistribution",
                                "timestamp": 0.0,
                                "identifier": "prior",
                                "alpha": 0.5,
                                "beta": 0.5,
                            },
                            "dt_return": 10,
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
                            "dt_return": 10,
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
]
