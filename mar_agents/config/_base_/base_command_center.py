# model for the command center

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
        trust=dict(type="NoTrustPipeline"),
    ),
)
