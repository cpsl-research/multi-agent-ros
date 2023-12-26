# model for a pipeline

pipeline = dict(
    type="PassiveAgentPipeline",
    perception=[
        dict(
            type="PerceptionWrapper",
            ID=0,
            ID_input=[0],
            algorithm=dict(
                type="Passthrough3DObjectDetector",
                post_hooks=[dict(type="DetectionsLogger")]
            ),
        )
    ],
    tracking=[
        dict(
            type="TrackingWrapper",
            ID=0,
            ID_input=[0],
            algorithm=dict(
                type="BasicBoxTracker3D",
                threshold_confirmed=5,
                threshold_coast=20,
                check_reference=False,
                post_hooks=[dict(type="TracksLogger")]
            ),
        )
    ],
)