# model for command center pipeline
    
pipeline = dict(
    type="CommandCenterPipeline",
    clustering=dict(
        type="SampledAssignmentClusterer",
        assign_radius=2.0
    ),
    tracking=dict(
        type="TrackBasedMultiTracker",
        fusion=dict(type="CovarianceIntersectionFusionToBox"),
        tracker=dict(type="BasicBoxTracker3D", check_reference=False),
        post_hooks=[dict(type="TracksLogger")],
    ),
    # trust=dict(type="NoTrustPipeline"),
)