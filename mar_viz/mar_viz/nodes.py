import rclpy
from avstack_bridge.objects import ObjectStateBridge
from avstack_msgs.msg import ObjectStateArray
from rclpy.node import Node


class AvstackBridgedVisualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.object_bridge = ObjectStateBridge()

        # subscribe to ground truth object information
        self.subscription = self.create_subscription(
            ObjectStateArray,
            "object_truth",
            self.listener_callback,
            10,  # 10 is the queue size
        )

    def listener_callback(self, msg):
        obj_state_array = self.object_bridge.objectstatearray_to_avstack(msg)
        # self.get_logger().info(f"Received {len(obj_state_array)} objects")


def main(args=None):
    rclpy.init(args=args)

    visualizer = AvstackBridgedVisualizer()

    rclpy.spin(visualizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
