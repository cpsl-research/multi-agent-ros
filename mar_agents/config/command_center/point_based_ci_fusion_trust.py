_base_ = "../_base_/base_command_center.py"

commandcenter = dict(
    type="CommandCenter",
    pipeline=dict(
        type="CommandCenterPipeline",
        clustering=dict(type="SampledAssignmentClusterer", assign_radius=2.0),
        group_tracking=dict(
            type="GroupTrackerWrapper",
            fusion=dict(type="CovarianceIntersectionFusion"),
            tracker=dict(type="BasicXyTracker"),
        ),
        trust=dict(
            type="PointBasedTrustPipeline",
            cluster_scorer=dict(
                type="ClusterScorer", connective=dict(type="StandardFuzzy")
            ),
            agent_scorer=dict(type="AgentScorer"),
            trust_estimator=dict(
                type="MaximumLikelihoodTrustEstimator",
                distribution=dict(type="Beta", alpha=None, beta=None, phi=0.5, lam=0.1),
                update_rate=10,
                time_window=None,
                forgetting=0.0,
            ),
        ),
    ),
)
