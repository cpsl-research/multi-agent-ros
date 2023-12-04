from avapi.carla.dataset import CarlaScenesManager
from avstack.geometry import PassiveReferenceFrame
from avstack_bridge.base import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.objects import ObjectStateBridge
from avstack_msgs.msg import ObjectStateArray


class CarlaDatasetLoader:
    def __init__(
        self,
        dataset_path: str,
        scene_idx: int,
        i_frame_start: int = 0,
    ) -> None:
        self.scene_dataset = CarlaScenesManager(
            data_dir=dataset_path
        ).get_scene_dataset_by_index(scene_idx=scene_idx)
        self.i_frame = i_frame_start  # start after initialization

    def load_next(self) -> ObjectStateArray:
        """Loads the next set of data from the dataset

        'next' is defined in this case as the next available frame of NPC data
        """
        objs = None
        while True:
            try:
                objs = self.scene_dataset.get_objects_global(frame=self.i_frame)
            except:
                self.i_frame += 1
                if self.i_frame > self.scene_dataset.frames[-1]:
                    break
            else:
                break
        timestamp = self.scene_dataset.get_timestamp(frame=self.i_frame, sensor="npcs")
        self.i_frame += 1

        # Process objects once loaded
        if objs is None:
            # we must be done??
            # TODO -- how to exit?
            raise
        else:
            # convert object messages
            obj_header = Bridge.reference_to_header(
                PassiveReferenceFrame(
                    frame_id="world",
                    timestamp=timestamp,
                )
            )
            objs_msgs = ObjectStateBridge.avstack_to_objecstatearray(
                obj_states=objs, header=obj_header
            )
            # HACK: ensure that timestamp is correct
            objs_msgs.header.stamp = Bridge.time_to_rostime(timestamp)

            # get agent information
            agent_poses = {}
            agent_detections = {}
            if self.i_frame in self.scene_dataset.get_frames(sensor="main_lidar"):

                ####################################
                # Agent information
                # HACK: This is soooooo hacky
                ####################################

                # get agent names:
                agent_names = {}
                ID = 0
                for agent_lidar_sensor in self.scene_dataset.sensor_IDs.keys():
                    if ("lidar" in agent_lidar_sensor.lower()) and "infra" in (
                        agent_lidar_sensor.lower()
                    ):
                        agent_names[agent_lidar_sensor] = f"agent{ID}"
                        ID += 1
                agent_names["ego"] = "ego"

                # loop over agents
                for sensor_name, agent_name in agent_names.items():
                    # agent pose
                    if agent_name == "ego":
                        ego = self.scene_dataset.get_ego(frame=self.i_frame)
                        agent_ref = ego.as_reference()
                    else:
                        lidar_calib = self.scene_dataset.get_calibration(
                            frame=self.i_frame, sensor=sensor_name
                        )
                        agent_ref = lidar_calib.reference
                    agent_ref.to_frame = agent_name
                    agent_ref.from_frame = "world"
                    agent_ref.timestamp = timestamp
                    agent_poses[agent_name] = Bridge.reference_to_tf2_stamped(agent_ref)

                    # agent detections
                    agent_dets = self.scene_dataset.get_objects(
                        frame=self.i_frame, sensor=sensor_name
                    )
                    passive_ref = PassiveReferenceFrame(
                        frame_id=agent_name, timestamp=timestamp
                    )
                    agent_obj_header = Bridge.reference_to_header(passive_ref)
                    agent_detections[
                        agent_name
                    ] = DetectionBridge.avstack_to_detections(
                        agent_dets,
                        header=agent_obj_header,
                    )

        return objs_msgs, agent_poses, agent_detections, self.i_frame

        # # ego pose
        # try:
        #     ego = self.scene_dataset.get_ego(frame=self.i_frame)
        # except FileNotFoundError:
        #     agent_poses["ego"] = None
        # else:
        #     # HACK for ego reference
        #     ego_ref = ego.as_reference()
        #     ego_ref.to_frame = "ego"
        #     ego_ref.from_frame = "world"
        #     ego_ref.timestamp = timestamp
        #     agent_poses["ego"] = Bridge.reference_to_tf2_stamped(ego_ref)

        # # ego detections
        # try:
        #     ego_detections = self.scene_dataset.get_objects(
        #         frame=self.i_frame, sensor="main_lidar"
        #     )
        # except FileNotFoundError:
        #     agent_detections["ego"] = None
        # else:
        #     ego_obj_header = Bridge.reference_to_header(
        #         PassiveReferenceFrame(
        #             frame_id="ego",
        #             timestamp=timestamp,
        #         )
        #     )
        #     agent_detections["ego"] = DetectionBridge.avstack_to_detections(
        #         ego_detections,
        #         header=ego_obj_header,
        #     )

        # # agent detections
        # ID = 0

        #         agent_ref = self.scene_dataset.get_agent(frame=self.i_frame)
        #         ID += 1

        #     agent_name = "agent"
        #     objs_in_view = self.scene_dataset.get_objects(frame=self.i_frame, sensor="")
