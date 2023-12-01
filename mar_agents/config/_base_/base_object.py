# model for an object in the scene

object = dict(
    type="Object",
    spawn=dict(type="RandomPoseTwist"),
    motion=dict(
        type="ConstantSpeedMarkovTurn",
        sigma_roll=0,
        sigma_pitch=0,
        sigma_yaw=1,
    ),
)
