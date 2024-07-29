"""
Passive agents already have a route planned and do no use their
situational awareness to impact the route.
"""

import rclpy
from avstack_msgs.msg import BoxTrackArray, ObjectStateArray
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2 as LidarMsg

from .base import BaseAgent


class PassiveAgent(BaseAgent):
    def __init__(self):
        super().__init__(name="agent", default_pipeline="passive_agent.py")

        # agents subscribe to gt object messages
        self.subscriber_objects = self.create_subscription(
            ObjectStateArray,
            "gt_objects",
            self.gt_object_callback,
            10,
        )

        # agents subscribe to sensor data
        self.subscriber_sensors = {
            "camera0": self.create_subscription(
                ImageMsg,
                "camera0",
                self.img_callback,
                10,
            ),
            "lidar0": self.create_subscription(
                LidarMsg,
                "lidar0",
                self.pc_callback,
                10,
            ),
        }

        # agents publish track information
        self.publisher_tracks = self.create_publisher(BoxTrackArray, "tracks", 10)

    def gt_object_callback(self, msg):
        gt_objs = self.objs_bridge.objectstatearray_to_avstack(msg)
        tracks_out = self.pipeline(
            sensing={0: gt_objs},
            platform=None,
            frame=gt_objs.frame,
            timestamp=gt_objs.timestamp,
        )
        msg_track = self.track_bridge.avstack_to_tracks(
            tracks_out[0],
            header=msg.header,
            default_type=BoxTrackArray,
        )
        msg_track_2 = self.track_bridge.avstack_to_tracks(gt_objs, header=msg.header)
        self.publisher_tracks.publish(msg_track)

    def img_callback(self, msg):
        self.get_logger().info("received image")

    def pc_callback(self, msg):
        self.get_logger().info("received pc")


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
