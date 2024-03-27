import json
import os

import rclpy
from avstack_msgs.msg import ObjectStateArray
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2 as LidarMsg
from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros.buffer import Buffer

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
        self.declare_parameter("max_frames", 200)
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
        self.publisher_agent_object_gt = {}
        self.publisher_agent_data = {}

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
            agent_data,
            agent_objects,
            i_frame,
        ) = self.loader.load_next()

        # publish object ground truth object states
        self.publisher_object_gt.publish(obj_state_array)

        # publish agent pose information
        for agent in agent_poses:
            if agent_poses[agent] is not None:
                self.tf_broadcaster.sendTransform(agent_poses[agent])

        # publish agent sensor data
        for agent in agent_data:
            if agent not in self.publisher_agent_data:
                self.publisher_agent_data[agent] = {}
            for sensor in agent_data[agent]:
                if sensor not in self.publisher_agent_data[agent]:
                    if "camera" in sensor:
                        msg_type = ImageMsg
                    elif "lidar" in sensor:
                        msg_type = LidarMsg
                    else:
                        raise ValueError(sensor)
                    self.publisher_agent_data[agent][sensor] = self.create_publisher(
                        msg_type, f"{agent}/{sensor}", 10
                    )
                self.publisher_agent_data[agent][sensor].publish(
                    agent_data[agent][sensor]
                )

        # publish object information in gent view
        for agent in agent_objects:
            if agent not in self.publisher_agent_object_gt:
                self.publisher_agent_object_gt[agent] = self.create_publisher(
                    ObjectStateArray, f"{agent}/gt_objects", 10
                )
            if agent_objects[agent] is not None:
                self.publisher_agent_object_gt[agent].publish(agent_objects[agent])

        # save index-to-frame map
        with open(os.path.join(self.output_folder, "idx_to_frame_map.txt"), "a") as f:
            f.write("{} {}\n".format(self.index, i_frame))
        self.index += 1
        if self.index >= self.get_parameter("max_frames").value:
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    sim = CarlaPointSimulator()

    try:
        rclpy.spin(sim)
    except SystemExit:  # <--- process the exception
        rclpy.logging.get_logger("Quitting").info("Done")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
