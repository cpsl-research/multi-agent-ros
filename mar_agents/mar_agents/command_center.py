"""
The command center is a receiver of information from
other agents. The CC runs a pipeline across the 
information from all of the agents.
"""

import rclpy
from avstack_bridge.base import Bridge
from avstack_bridge.tracks import TrackBridge
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
        self.pubsliher_fused = self.create_publisher(ObjectStateArray, "fused", 10)
        self.i_frame = 0

    def pipeline_callback(self, msg: ObjectStateArrayWithSenderArray):
        obj_tracks = {}
        for state_array in msg.state_arrays:
            tracks = TrackBridge.tracks_to_avstack(state_array)
            obj_tracks[state_array.sender_id] = tracks

        timestamp = Bridge.rostime_to_time(msg.state_arrays[0].header.stamp)
        fused_out = self.pipeline(
            tracks_in=obj_tracks,
            platform=None,
            frame=self.i_frame,
            timestamp=timestamp,
        )
        self.i_frame += 1
        msg_fused = []
        self.get_logger().info("Maintaining {} fused tracks".format(len(fused_out)))


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
