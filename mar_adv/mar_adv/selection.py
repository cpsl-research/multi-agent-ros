from copy import deepcopy
from typing import Dict, List

import numpy as np
from avstack.environment.objects import ObjectState
from avstack.geometry import Attitude, Box3D, Position, Velocity, transform_orientation
from avstack.geometry.coordinates import cross_product_coord
from avstack.modules.perception.detections import BoxDetection
from avstack.modules.tracking.tracks import BasicBoxTrack3D


class TargetObject:
    def __init__(self, obj_state, ID=None):
        self.obj_state = obj_state
        self.target_state = deepcopy(obj_state)
        self.ID = ID
        self.last_position = obj_state.position
        self.t = 0.0

    def as_track(self):
        """Format the target state as a track state"""
        raise

    def as_detection(self):
        """Format the target state as a detection"""
        raise

    def propagate(self, dt: float):
        """Updates target state with kinematics"""
        self.target_state.predict(dt)
        self.t += dt


def select_false_positives(
    fp_poisson, reference, x_sigma=30, v_sigma=10, hwl=[2, 2, 4]
) -> List[Dict]:
    n_fp = int(np.random.poisson(fp_poisson))
    targets = []
    for i in range(n_fp):
        obj = ObjectState(ID=i)

        # position is random in x-y
        x_vec = x_sigma * np.array([np.random.randn(), np.random.randn(), 0])
        position = Position(x_vec, reference=reference)

        # velocity is random in x-y
        v_vec = v_sigma * np.array([np.random.randn(), np.random.randn(), 0])
        velocity = Velocity(v_vec, reference=reference)

        # attitude is in direction of velocity
        c1 = velocity.unit()
        c2 = np.array([0, 0, 1])
        c3 = cross_product_coord(c1, c2)
        R = np.array([c1, c2, c3])  # is this the correct order?
        attitude = Attitude(
            transform_orientation(R, "DCM", "quat"), reference=reference
        )

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
    targets_selected = np.random.choice(existing_objects, size=n_fn, replace=False)
    targets = []
    for obj in targets_selected:
        if isinstance(obj, BoxDetection):
            pass
        elif isinstance(obj, BasicBoxTrack3D):
            pass
        else:
            raise NotImplementedError(type(obj))
        targets.append(TargetObject(obj_as_state))
    return targets
