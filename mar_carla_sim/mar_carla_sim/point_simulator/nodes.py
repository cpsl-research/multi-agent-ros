import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray

# from .loaders 


class PointSimulator(Node):
    def __init__(self):
        super().__init__("point_simulator")

        # parameters
        self.declare_parameter("real_time_framerate", 10)

        # set things based on params
        rt_framerate = self.get_parameter("real_time_framerate").get_parameter_value().string_value

        self.loader = None

        # set ros actions
        self.publisher_ = self.create_publisher(PoseArray, "object_poses", 10)
        timer_period = 1.0 / rt_framerate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i_frame = 0

    def timer_callback(self):
        # load in ground truth information

        # publish
        pass



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