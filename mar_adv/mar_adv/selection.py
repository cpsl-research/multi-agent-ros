from copy import deepcopy
from typing import Dict, List

import numpy as np
from avstack.environment.objects import ObjectState
from avstack.geometry import (
    Attitude,
    Box3D,
    PassiveReferenceFrame,
    Position,
    ReferenceFrame,
    Velocity,
    transform_orientation,
)
from avstack.modules.perception.detections import BoxDetection
from avstack.modules.tracking.tracks import BasicBoxTrack3D
from avstack_bridge.base import Bridge
from geometry_msgs.msg import TransformStamped


class TargetObject:
    def __init__(self, obj_state, ID=None):
        self.obj_state = obj_state
        self.target_state = deepcopy(obj_state)
        self.ID = ID
        self.last_position = obj_state.position
        self.t = 0.0

    def as_track(self):
        """Format the target state as a track state"""
        return BasicBoxTrack3D(
            t0=self.obj_state.t,
            box3d=self.obj_state.box,
            reference=self.obj_state.reference,
            obj_type=self.obj_state.obj_type,
            v=self.obj_state.velocity.x,
        )

    def as_detection(self):
        """Format the target state as a detection"""
        return BoxDetection(
            source_identifier=0,
            box=self.obj_state.box,
            reference=self.obj_state.reference,
            obj_type=self.obj_state.obj_type,
        )

    def propagate(self, dt: float):
        """Updates target state with kinematics"""
        self.target_state.predict(dt)
        self.t += dt


def select_false_positives(
    fp_poisson,
    reference,
    tf_world_to_agent: TransformStamped,
    x_sigma: float = 30,
    v_sigma: float = 10,
    hwl: List[float] = [2, 2, 4],
) -> List[Dict]:
    """Create some false positives for attacks.

    Keep in mind the reference frame may not be parallel to the group plane.
    Therefore, the generation of false positive targets should be assumed
    to be made relative to a projected sensor plane that is co-planar with ground.
    """
    if reference is None:
        # this is a dummy reference and doesn't need to be "correct"
        reference = PassiveReferenceFrame(frame_id="world", timestamp=0.0)
    n_fp = int(np.random.poisson(fp_poisson))
    targets = []
    for i in range(n_fp):
        obj = ObjectState(ID=i, obj_type="")

        # position is random in x-y
        x_vec = x_sigma * np.array([np.random.randn(), np.random.randn(), 0])

        # velocity is random in x-y
        v_vec = v_sigma * np.array([np.random.randn(), np.random.randn(), 0])

        # attitude is in direction of velocity
        yaw = np.arctan2(v_vec[2], v_vec[1])
        euler = [yaw, 0, 0]
        q_obj = transform_orientation(euler, "euler", "quat")

        # adjust for false positive selection that is coplanar with ground
        reference_agent = Bridge.tf2_to_reference(tf_world_to_agent)
        x_gp = deepcopy(reference_agent.x)
        x_gp[2] = 0.0
        e_gp = transform_orientation(reference_agent.q, "quat", "euler")
        q_gp = transform_orientation([0, 0, e_gp[2]], "euler", "quat")
        reference_gp = ReferenceFrame(
            x=x_gp, q=q_gp, reference=reference_agent.reference
        )

        # convert to avstack objects
        position = Position(x_vec, reference=reference_gp).change_reference(
            reference_agent, inplace=False
        )
        velocity = Velocity(v_vec, reference=reference_gp).change_reference(
            reference_agent, inplace=False
        )
        attitude = Attitude(q_obj, reference=reference_gp).change_reference(
            reference_agent, inplace=False
        )
        position.reference = reference
        velocity.reference = reference
        attitude.reference = reference

        # set object attributes
        obj.set(
            t=0.0,  # initialization of object
            position=position,
            box=Box3D(position, attitude, hwl),
            velocity=velocity,
            attitude=attitude,
        )
        targets.append(TargetObject(obj, ID=i))
    return targets


def select_false_negatives(existing_objects, fn_fraction: float) -> List[Dict]:
    n_objects = len(existing_objects)
    n_fn = max(min(1, n_objects), int(fn_fraction * n_objects))
    idx_targets = np.random.choice(
        list(range(len(existing_objects))), size=n_fn, replace=False
    )
    targets = [TargetObject(existing_objects[idx]) for idx in idx_targets]
    return targets
