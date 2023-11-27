from avstack_bridge import 

from avapi.carla.dataset import CarlaScenesManager


class CarlaDatasetLoader:
    def __init__(self, dataset_path: str, scene_idx: int) -> None:
        self.dataset_path = dataset_path
        self.scene_dataset = CarlaScenesManager.get_scene_dataset_by_index(scene_idx)
        self.i_frame = 0

    def load_next(self):
        """Loads the next set of data from the dataset
        
        'next' is defined in this case as the next available frame of NPC data
        """
        while True:
            try:
                objs = self.scene_dataset.get_objects_global(frame=self.i_frame)
            except:
                self.i_frame += 1
            else:
                break