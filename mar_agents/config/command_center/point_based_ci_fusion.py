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
        trust=dict(type="NoTrustPipeline"),
    ),
)
