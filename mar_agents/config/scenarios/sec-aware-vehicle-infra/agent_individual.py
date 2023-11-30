_base_ = "../../_base_/base_agent.py"

agent = dict(
    motion=dict(
        type="ConstantSpeedMarkovTurn", sigma_roll=0, sigma_pitch=0, sigma_yaw=1.0
    ),
    communication=dict(max_range=200),
)
