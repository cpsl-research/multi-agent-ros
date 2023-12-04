"""
The command center is a receiver of information from
other agents. The CC runs a pipeline across the 
information from all of the agents.
"""

import rclpy

from avstack_msgs.msg import ObjectStateArray, ObjectStateArrayWithSenderArray
from .base import BaseAgent


class CommandCenter(BaseAgent):
    def __init__(self):
        super().__init__(name="command_center", default_pipeline="command_center.py")

        # subscribe to collated tracks from the broker
        self.subscriber_tracks = self.create_subscription(
            ObjectStateArrayWithSenderArray, "collated", self.pipeline_callback, 10
        )

        # publish fused results
        self.pubsliher_fused = self.create_publisher(
            ObjectStateArray, "fused", 10
        )

    def pipeline_callback(self, msg: ObjectStateArrayWithSenderArray):
        # obj_states = None
        # fused_out = self.pipeline()
        # msg_fused = self.
        pass


def main(args=None):
    rclpy.init(args=args)

    cc = CommandCenter()

    rclpy.spin(cc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
