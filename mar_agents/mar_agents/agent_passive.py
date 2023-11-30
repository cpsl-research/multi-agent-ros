"""
Passive agents already have a route planned and do no use their
situational awareness to impact the route.
"""

import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from avstack.config import Config, PIPELINE

from avstack_bridge.detections import DetectionBridge
from avstack_bridge.tracks import TrackBridge

import mar_algs  # to set the pipeline
from avstack_msgs.msg import ObjectStateStamped, ObjectStateArray
from vision_msgs.msg import BoundingBox3DArray


class PassiveAgent(Node):
    def __init__(self):
        super().__init__("agent")

        # transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # bridge converters
        self.dets_bridge = DetectionBridge()
        self.track_bridge = TrackBridge()

        self.declare_parameters(
            namespace='',
            parameters=[
                ("agent_name", "agent"),
                ("pipeline", "passive_agent_pipeline.py")
            ]
        )

        # -- set up pipeline
        self.pipeline_path = os.path.join(
            get_package_share_directory("mar_agents"),
            "config", "pipeline", self.get_parameter("pipeline").value,
        )
        if not os.path.exists(self.pipeline_path):
            raise FileNotFoundError(f"Cannot find pipeline path at {self.pipeline_path}")

        self.pipeline_cfg = Config.fromfile(self.pipeline_path).pipeline
        self.pipeline = PIPELINE.build(self.pipeline_cfg)

        # -- passive agents subscribe to their own pose information
        self.subscriber_pose = self.create_subscription(
            ObjectStateStamped,
            "pose",
            self.pose_callback,
            10,
        )

        # -- passive agents subscribe to sensor/detection messages
        self.subscriber_dets = self.create_subscription(
            BoundingBox3DArray,
            "detections",
            self.dets_callback,
            10,
        )

        # -- passive agents publish track information
        self.publisher_tracks = self.create_publisher(ObjectStateArray, "tracks", 10)

    def pose_callback(self, msg):
        pass  # don't really need to do anything here

    def dets_callback(self, msg):
        dets = self.dets_bridge.detections_to_avstack(msg, tf_buffer=self.tf_buffer)
        tracks_out = self.pipeline(dets)
        msg_track = self.track_bridge.avstack_to_tracks(tracks_out, tf_buffer=self.tf_buffer)
        self.publisher_tracks.publish(msg_track)


def main(args=None):
    rclpy.init(args=args)

    sim = PassiveAgent()

    rclpy.spin(sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()