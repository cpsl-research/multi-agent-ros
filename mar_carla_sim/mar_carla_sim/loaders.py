from avapi.carla.dataset import CarlaScenesManager
from avstack.geometry import PassiveReferenceFrame
from avstack_bridge.base import Bridge
from avstack_bridge.objects import ObjectStateBridge
from avstack_bridge.sensors import CameraSensorBridge, LidarSensorBridge
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
        self.i_frame = i_frame_start - 1  # start after initialization

    def load_next(self) -> ObjectStateArray:
        """Loads the next set of data from the dataset

        'next' is defined in this case as the next available frame of NPC data
        """
        self.i_frame += 1
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
        timestamp = self.scene_dataset.get_timestamp(frame=self.i_frame)

        # Process objects once loaded
        if objs is None:
            # we must be done??
            raise SystemExit
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

            agent_poses = {}
            agent_data = {}
            agent_objects = {}
            if self.i_frame in self.scene_dataset.frames:
                agents = self.scene_dataset.get_agents(self.i_frame)
                for agent in agents:
                    if agent.ID is None:
                        raise RuntimeError("Agent must have an ID")
                    agent_name = f"agent{agent.ID}"

                    # agent poses
                    agent_ref = agent.as_reference()
                    agent_ref.from_frame = "world"
                    agent_ref.to_frame = agent_name
                    agent_ref.timestamp = timestamp
                    agent_poses[agent_name] = Bridge.reference_to_tf2_stamped(agent_ref)

                    # object states in agent view
                    try:
                        agent_objs = self.scene_dataset.get_objects(
                            frame=self.i_frame,
                            sensor="lidar-0",
                            agent=agent.ID,
                            max_dist=60,
                        )
                    except FileNotFoundError:
                        continue
                    passive_ref = PassiveReferenceFrame(
                        frame_id=agent_name, timestamp=timestamp
                    )
                    agent_obj_header = Bridge.reference_to_header(passive_ref)
                    agent_objects[
                        agent_name
                    ] = ObjectStateBridge.avstack_to_objecstatearray(
                        agent_objs,
                        header=agent_obj_header,
                    )

                    # agent sensor data
                    data = {}
                    for sensor in self.scene_dataset.sensor_IDs[agent.ID]:
                        sensor_ref = PassiveReferenceFrame(
                            frame_id=sensor,
                            timestamp=timestamp,
                        )
                        sensor_header = Bridge.reference_to_header(sensor_ref)
                        if "camera" in sensor:
                            img = self.scene_dataset.get_image(
                                frame=self.i_frame,
                                sensor=sensor,
                                agent=agent.ID,
                            )
                            sensor_data = CameraSensorBridge.avstack_to_imgmsg(
                                img, header=sensor_header
                            )
                        elif "lidar" in sensor:
                            pc = self.scene_dataset.get_lidar(
                                frame=self.i_frame,
                                sensor=sensor,
                                agent=agent.ID,
                            )
                            sensor_data = LidarSensorBridge.avstack_to_pc2(
                                pc, header=sensor_header
                            )
                        else:
                            sensor_data = None
                        if sensor_data is not None:
                            sensor = sensor.replace("-", "")
                            data[sensor] = sensor_data
                    agent_data[agent_name] = data

        return objs_msgs, agent_poses, agent_data, agent_objects, self.i_frame
