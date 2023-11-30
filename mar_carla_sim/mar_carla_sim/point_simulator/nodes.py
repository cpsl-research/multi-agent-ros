import rclpy
from rclpy.node import Node

from vision_msgs.msg import BoundingBox3DArray
from avstack_msgs.msg import ObjectStateStamped, ObjectStateArray

from mar_msgs.srv import SpawnAgent

from tf2_ros import TransformListener
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
        self.declare_parameter("n_agents", 0)

        # set things based on params
        rt_framerate = self.get_parameter("real_time_framerate").value
        self.loader = CarlaDatasetLoader(
            dataset_path=self.get_parameter("dataset_path").value,
            scene_idx=self.get_parameter("scene_idx").value,
            tf_buffer=self.tf_buffer,
            i_frame_start=self.get_parameter("i_frame_start").value,
        )

        # all the publishers
        self.publisher_object_gt = self.create_publisher(ObjectStateArray, "object_truth", 10)
        self.publisher_agent_gt  = {
            f"agent{n}" : self.create_publisher(ObjectStateStamped, f"agent{n}/pose") 
            for n in range(self.get_parameter("n_agents").value)
        }
        self.publisher_agent_gt["ego"] = self.create_publisher(ObjectStateStamped, "ego/pose", 10)
        self.publisher_agent_dets = {
            f"agent{n}" : self.create_publisher(BoundingBox3DArray, f"agent{n}/detections", 10)
            for n in range(self.get_parameter("n_agents").value)
        }
        self.publisher_agent_dets["ego"] = self.create_publisher(BoundingBox3DArray, "ego/detections", 10)

        # callback timers
        timer_period = 1.0 / rt_framerate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        obj_state_array, agent_poses, agent_detections, i_frame = self.loader.load_next()

        # publish object ground truth states
        self.publisher_object_gt.publish(obj_state_array)
        for agent in agent_poses:
            if agent_poses[agent] is not None:
                self.publisher_agent_gt[agent].publish(agent_poses[agent])
            if agent_detections[agent] is not None:
                self.publisher_agent_dets[agent].publish(agent_detections[agent])

        # self.get_logger().info(f"Publishing {len(obj_state_array.states)} objects at frame {i_frame}")
        # publish agent ground truth states


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