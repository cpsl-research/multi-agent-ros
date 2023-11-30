from avstack_msgs.msg import ObjectStateArray
from avstack_bridge.objects import ObjectStateBridge
from avstack_bridge.detections import DetectionBridge
from tf2_ros.buffer import Buffer

from avstack.environment.objects import ObjectState
from avapi.carla.dataset import CarlaScenesManager


class CarlaDatasetLoader:
    def __init__(self, dataset_path: str, scene_idx: int, tf_buffer: Buffer, i_frame_start: int=0) -> None:
        self.scene_dataset = CarlaScenesManager(data_dir=dataset_path).get_scene_dataset_by_index(
            scene_idx=scene_idx
        )
        self.i_frame = i_frame_start  # start after initialization
        self.obj_bridge = ObjectStateBridge()
        self.det_bridge = DetectionBridge()
        self.tf_buffer = tf_buffer

    def load_next(self) -> ObjectStateArray:
        """Loads the next set of data from the dataset
        
        'next' is defined in this case as the next available frame of NPC data
        """
        objs = None
        while True:
            try:
                objs = self.scene_dataset.get_objects_global(frame=self.i_frame)
                self.i_frame += 1
            except:
                self.i_frame += 1
                if self.i_frame > self.scene_dataset.frames[-1]:
                    break
            else:
                break
        
        # Process objects once loaded
        if objs is None:
            # we must be done??
            # TODO -- how to exit?
            raise
        else:
            objs_msgs = self.obj_bridge.avstack_to_objecstatearray(obj_states=objs, tf_buffer=self.tf_buffer)


            ####################################
            # Agent information
            # HACK: This is soooooo hacky
            ####################################

            agent_poses = {}
            agent_detections = {}

            # ego pose
            try:
                ego = self.scene_dataset.get_ego(frame=self.i_frame)
            except FileNotFoundError:
                agent_poses["ego"] = None
            else:
                agent_poses["ego"] = self.obj_bridge.avstack_to_objectstatestamped(ego, tf_buffer=self.tf_buffer)

            # ego detections
            try:
                ego_detections = self.scene_dataset.get_objects(frame=self.i_frame, sensor="main_lidar")
            except FileNotFoundError:
                agent_detections["ego"] = None
            else:
                agent_detections["ego"] = self.det_bridge.avstack_to_detections(
                    ego_detections, tf_buffer=self.tf_buffer, frame_override="ego",
                )

            # for each infrastructure agent:
            # ID = 0
            # for agent_lidar_sensor in self.scene_dataset.sensor_IDs.values():
            #     if ('lidar' in agent_lidar_sensor.lower()) and 'infra' in (agent_lidar_sensor.lower()):
            #         agent_ref = self.scene_dataset.get_agent(frame=self.i_frame)
            #         ID += 1

            #     agent_name = "agent"
            #     objs_in_view = self.scene_dataset.get_objects(frame=self.i_frame, sensor="")

        return objs_msgs, agent_poses, agent_detections, self.i_frame
        