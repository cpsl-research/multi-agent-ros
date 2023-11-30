# model for an agent

agent = dict(
    type="Agent",
    trusted=False,
    spawn=dict(type="RandomPoseTwist"),
    motion=dict(type="Stationary"),
    communication=dict(
        type="Omnidirectional",
        max_range=None,
        rate="continuous",
        send=True,
        receive=False,
        always_to_central=False,
    ),
    sensing=[
        dict(
            type="SensorWrapper",
            ID=0,
            model=dict(
                type="PositionSensor",
                ID=0,
                x=[0, 0, 0],
                q=[1, 0, 0, 0],
                noise=[0, 0, 0],
                fov=[30, 0.785398, 3.1415926],  # [30, pi/4, pi]
                Pd=1.0,
                Dfa=0.0,
            ),
        )
    ],
    pipeline=None,
)
