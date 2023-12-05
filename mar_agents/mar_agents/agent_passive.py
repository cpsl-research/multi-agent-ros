"""
Passive agents already have a route planned and do no use their
situational awareness to impact the route.
"""

import rclpy
from avstack_msgs.msg import ObjectStateArray, ObjectStateStamped
from vision_msgs.msg import BoundingBox3DArray

from .base import BaseAgent


class PassiveAgent(BaseAgent):
    def __init__(self):
        super().__init__(name="agent", default_pipeline="passive_agent.py")

        # passive agents subscribe to sensor/detection messages
        self.subscriber_dets = self.create_subscription(
            BoundingBox3DArray,
            "detections",
            self.dets_callback,
            10,
        )

        # passive agents publish track information
        self.publisher_tracks = self.create_publisher(ObjectStateArray, "tracks", 10)

    def dets_callback(self, msg):
        dets = self.dets_bridge.detections_to_avstack(msg)
        tracks_out = self.pipeline(
            sensing={0: dets},
            platform=None,
            frame=dets.frame,
            timestamp=dets.timestamp,
        )
        msg_track = self.track_bridge.avstack_to_tracks(
            tracks_out[0],
            header=msg.header,
        )
        self.publisher_tracks.publish(msg_track)


def main(args=None):
    rclpy.init(args=args)

    agent = PassiveAgent()

    rclpy.spin(agent)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
