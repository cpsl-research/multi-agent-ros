import json
import os

import rclpy
from avstack_msgs.msg import ObjectStateArray
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros.buffer import Buffer
from vision_msgs.msg import BoundingBox3DArray

from .loaders import CarlaDatasetLoader


class PointSimulator(Node):
    def __init__(self, name):
        super().__init__(name)

        # transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


class CarlaPointSimulator(PointSimulator):
    def __init__(self):
        super().__init__("point_simulator")

        # parameters
        self.declare_parameter("real_time_framerate", 10)
        self.declare_parameter("dataset_path", "/data/shared/CARLA/multi-agent-v1")
        self.declare_parameter("scene_idx", 0)
        self.declare_parameter("i_frame_start", 4)
        self.declare_parameter(name="output_folder", value="outputs")
        self.index = 0

        # set things based on params
        rt_framerate = self.get_parameter("real_time_framerate").value
        self.loader = CarlaDatasetLoader(
            dataset_path=self.get_parameter("dataset_path").value,
            scene_idx=self.get_parameter("scene_idx").value,
            i_frame_start=self.get_parameter("i_frame_start").value,
        )

        # all the publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_object_gt = self.create_publisher(
            ObjectStateArray, "object_truth", 10
        )
        self.publisher_agent_dets = {}

        # callback timers
        timer_period = 1.0 / rt_framerate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # log metadata
        self.output_folder = os.path.join(
            self.get_parameter("output_folder").value, "simulator"
        )
        os.makedirs(self.output_folder, exist_ok=False)
        metadata = {
            "scene_idx": self.get_parameter("scene_idx").value,
            "i_frame_start": self.get_parameter("i_frame_start").value,
            "dataset_path": self.get_parameter("dataset_path").value,
        }
        with open(os.path.join(self.output_folder, "metadata.json"), "w") as f:
            json.dump(metadata, f)

    def timer_callback(self):
        (
            obj_state_array,
            agent_poses,
            agent_detections,
            i_frame,
        ) = self.loader.load_next()

        # publish object ground truth object states
        self.publisher_object_gt.publish(obj_state_array)

        # publish agent pose information
        for agent in agent_poses:
            if agent_poses[agent] is not None:
                self.tf_broadcaster.sendTransform(agent_poses[agent])

        # publish detection information
        for agent in agent_detections:
            if agent not in self.publisher_agent_dets:
                self.publisher_agent_dets[agent] = self.create_publisher(
                    BoundingBox3DArray, f"{agent}/detections", 10
                )
            if agent_detections[agent] is not None:
                self.publisher_agent_dets[agent].publish(agent_detections[agent])

        # save index-to-frame map
        with open(os.path.join(self.output_folder, "idx_to_frame_map.txt"), "a") as f:
            f.write("{} {}\n".format(self.index, i_frame))
        self.index += 1


def main(args=None):
    rclpy.init(args=args)

    sim = CarlaPointSimulator()

    rclpy.spin(sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
