import random
from typing import Dict

import numpy as np
from avapi.carla import CarlaScenesManager
from avstack.datastructs import DataContainer
from avstack.environment import ObjectState
from avstack.geometry import GlobalOrigin3D, ReferenceFrame
from avstack.modules.clustering import SampledAssignmentClusterer
from avstack.modules.fusion import CovarianceIntersectionFusionToBox
from avstack.modules.perception.detections import BoxDetection
from avstack.modules.perception.object2dfv import MMDetObjectDetector2D
from avstack.modules.tracking import BasicBoxTracker3D


data_dir = "/data/shared/CARLA/multi-agent-aerial"
CSM = CarlaScenesManager(data_dir=data_dir)
CDM = CSM.get_scene_dataset_by_index(0)


def convert_2d_to_3d_detections(dets_2d: DataContainer, reference: ReferenceFrame):
    """Uses constraint that objects are on the ground"""
    ref_to_global = reference.integrate(start_at=GlobalOrigin3D)

    # each corner of the box has an angle off boresight
    dets_3d = DataContainer(
        frame=dets_2d.frame,
        timestamp=dets_2d.timestamp,
        data=[],
        source_identifier=dets_2d.source_identifier,
    )
    z = ref_to_global.x[0]  # assume x is direction to the ground
    for det in dets_2d:
        box_3d = det.box.upscale_to_3d(z_to_box=z, height=2.0)
        dets_3d.append(
            BoxDetection(
                data=box_3d,
                noise=np.array([1, 1, 1, 1, 1, 1]) ** 2,
                source_identifier=det.source_identifier,
                reference=det.reference,
            )
        )
    return dets_3d


def communicate(
    agent_receive: ObjectState,
    agent_send: ObjectState,
    d_max: float = 30.0,
    model: str = "quadratic",
) -> bool:
    """Probabilistic, distance-based communication model"""
    d = agent_receive.position.distance(agent_send.position)

    # run the communications model
    if model == "quadratic":
        p = 1 - 1 / d_max * 2 * d**2
        comm = random.rand <= p
    elif model == "absolute":
        comm = d <= d_max
    elif model == "always":
        comm = True
    else:
        raise NotImplementedError(model)

    return comm


def fusion(
    id_self: int,
    tracks_self: DataContainer,
    tracks_received: Dict[int, DataContainer],
    clustering: str = "assignment",
    assign_radius: float = 3.0,
    fusion: str = "ci",
):
    """Perform fusion of own tracks with neighbors"""

    # perform assignment/clustering
    if clustering == "assignment":
        clusters = SampledAssignmentClusterer.cluster(
            objects={id_self: tracks_self, **tracks_received},
            frame=tracks_self.frame,
            timestamp=tracks_self.timestamp,
            assign_radius=assign_radius,
            check_reference=True,
        )
    else:
        raise NotImplementedError

    # perform fusion on output clusters
    if fusion == "ci":
        tracks_out = [
            CovarianceIntersectionFusionToBox.fuse(cluster) for cluster in clusters
        ]
    else:
        raise NotImplementedError

    return tracks_out


def main():
    # set up the sensor/agents
    n_frames = 50
    sensor = "camera-0"
    agents = CDM.get_agents(frame=None)

    # set up the algorithms
    perception = MMDetObjectDetector2D(model="fasterrcnn", dataset="carla-joint")
    trackers = {agent.ID: BasicBoxTracker3D() for agent in agents}

    # set up data structures
    tracks_3d = {agent.ID: None for agent in agents}
    fused_3d = {agent.ID: None for agent in agents}
    last_imgs = {agent.ID: None for agent in agents}

    # flags
    det_type = "3d"  # "2d_perception", "2d_conversion", "3d"

    # loop over frames and replay some data
    for frame in CDM.get_frames(sensor=sensor, agent=0)[3:n_frames]:

        # loop over agents at this frame for local processing
        for agent in agents:

            # get data
            img = CDM.get_image(frame=frame, sensor=sensor, agent=agent.ID)
            objs_tru = CDM.get_objects(frame=frame, sensor=sensor, agent=agent.ID)
            last_imgs[agent.ID] = img

            # run perception
            if det_type == "2d_perception":
                objs_det = perception(img)
            elif det_type in ["2d_conversion", "3d"]:

                # get bounding boxes
                objs_det = objs_tru.apply_and_return("getattr", "box3d")
                if det_type == "2d_conversion":
                    objs_det = objs_det.apply_and_return(
                        "project_to_2d_bbox", img.calibration
                    )
                    noise = np.array([5, 5, 5, 5]) ** 2
                elif det_type == "3d":
                    noise = np.array([1, 1, 1, 1, 1, 1]) ** 2

                # build box detections
                objs_det = objs_det.apply_and_return(
                    BoxDetection,
                    noise=noise,
                    source_identifier=0,
                    reference=img.calibration.reference,
                )

            else:
                raise NotImplementedError(det_type)

            # convert 2d detections to 3d bounding boxes
            if det_type in ["2d_perception", "2d_conversion"]:
                objs_det_3d = convert_2d_to_3d_detections(objs_det, img.reference)
            else:
                objs_det_3d = objs_det

            # convert to global reference frame
            objs_det_3d.apply("change_reference", GlobalOrigin3D, inplace=True)

            # run tracking locally and save the history
            tracks_3d[agent.ID] = trackers[agent.ID](
                detections=objs_det_3d, check_reference=False, platform=GlobalOrigin3D
            )

        # loop over receiving agents
        for agent_receive in agents:
            receive_data = {}

            # loop over sending agents
            for agent_send in agents:

                # get info from all agents nearby
                if agent_send.ID == agent_receive.ID:
                    continue
                else:
                    # run communications model
                    if communicate(
                        agent_receive, agent_send, model="absolute", d_max=30
                    ):
                        receive_data[agent_send.ID] = tracks_3d[agent_send.ID]
                    else:
                        pass

            # perform fusion on all the received data
            fused_3d[agent_receive.ID] = fusion(
                agent_send.ID, tracks_3d[agent_send.ID], receive_data
            )


if __name__ == "__main__":
    main()
