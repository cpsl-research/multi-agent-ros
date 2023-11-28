import rclpy
from rclpy.node import Node

from avstack_msgs.msg import ObjectStateArray
from tf2_ros import TransformException, TransformListener
from tf2_ros.buffer import Buffer

from .loaders import CarlaDatasetLoader


class PointSimulator(Node):
    def __init__(self):
        super().__init__("point_simulator")

        # transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # parameters
        self.declare_parameter("real_time_framerate", 10)
        self.declare_parameter("dataset_path", "/data/shared/CARLA/object-v1")
        self.declare_parameter("scene_idx", 0)

        # set things based on params
        rt_framerate = self.get_parameter("real_time_framerate").value
        self.loader = CarlaDatasetLoader(
            dataset_path=self.get_parameter("dataset_path").value,
            scene_idx=self.get_parameter("scene_idx").value,
            tf_buffer=self.tf_buffer
        )

        # set ros actions
        self.publisher_ = self.create_publisher(ObjectStateArray, "object_truth", 10)
        timer_period = 1.0 / rt_framerate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        obj_state_array, i_frame = self.loader.load_next()
        self.publisher_.publish(obj_state_array)
        self.get_logger().info(f"Publishing {len(obj_state_array.states)} objects at frame {i_frame}")


def main(args=None):
    rclpy.init(args=args)

    sim = PointSimulator()

    rclpy.spin(sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()