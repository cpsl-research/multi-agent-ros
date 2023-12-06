# model for command center pipeline
    
pipeline = dict(
    type="CommandCenterPipeline",
    clustering=dict(
        type="SampledAssignmentClusterer",
        assign_radius=2.0
    ),
    group_tracking=dict(
        type="GroupTrackerWrapper",
        fusion=dict(type="CovarianceIntersectionFusion"),
        tracker=dict(type="BasicXyzTracker"),
    ),
    # trust=dict(type="NoTrustPipeline"),
)