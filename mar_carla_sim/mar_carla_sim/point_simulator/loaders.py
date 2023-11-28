from avstack_msgs.msg import ObjectStateArray
from avstack_bridge.objects import ObjectStateBridge
from tf2_ros.buffer import Buffer

from avapi.carla.dataset import CarlaScenesManager


class CarlaDatasetLoader:
    def __init__(self, dataset_path: str, scene_idx: int, tf_buffer: Buffer) -> None:
        self.scene_dataset = CarlaScenesManager(data_dir=dataset_path).get_scene_dataset_by_index(
            scene_idx=scene_idx
        )
        self.i_frame = 0
        self.obj_bridge = ObjectStateBridge()
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

        return objs_msgs, self.i_frame
        