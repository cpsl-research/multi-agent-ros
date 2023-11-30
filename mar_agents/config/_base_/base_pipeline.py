# model for a pipeline

pipeline = dict(
    type="PassiveAgentPipeline",
    perception=[
        dict(
            type="PerceptionWrapper",
            ID=0,
            ID_input=[0],
            algorithm=dict(type="Passthrough3DObjectDetector"),
        )
    ],
    tracking=[
        dict(
            type="TrackingWrapper",
            ID=0,
            ID_input=[0],
            algorithm=dict(
                type="BasicRazTracker",
                threshold_confirmed=5,
                threshold_coast=20,
            ),
        )
    ],
)