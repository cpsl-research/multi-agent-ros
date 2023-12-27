"""
The command center is a receiver of information from
other agents. The CC runs a pipeline across the 
information from all of the agents.
"""

import rclpy
from avstack_bridge.base import Bridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import BoxTrackArray, BoxTrackArrayWithSenderArray

from .base import BaseAgent


class CommandCenter(BaseAgent):
    def __init__(self):
        super().__init__(name="command_center", default_pipeline="command_center.py")

        # set to True for more detailed logging for debugging
        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value

        # subscribe to collated tracks from the broker
        self.subscriber_tracks = self.create_subscription(
            BoxTrackArrayWithSenderArray, "collated", self.pipeline_callback, 10
        )

        # publish fused results
        self.publisher_tracks = self.create_publisher(BoxTrackArray, "tracks", 10)
        self.i_frame = 0

    def pipeline_callback(self, msg: BoxTrackArrayWithSenderArray) -> BoxTrackArray:
        obj_tracks = {}
        for track_array in msg.track_arrays:
            tracks = TrackBridge.tracks_to_avstack(track_array)
            obj_tracks[track_array.sender_id] = tracks

        # run the cc pipeline
        timestamp = Bridge.rostime_to_time(msg.header.stamp)
        group_tracks = self.pipeline(
            tracks_in=obj_tracks,
            platform=None,
            frame=self.i_frame,
            timestamp=timestamp,
        )
        self.i_frame += 1

        # convert group tracks to output messages
        msg_tracks = TrackBridge.avstack_to_tracks(
            [g_track.state for g_track in group_tracks],
            header=msg.header,
            default_type=BoxTrackArray,
        )
        self.publisher_tracks.publish(msg_tracks)
        if self.debug:
            self.get_logger().info(
                "Maintaining {} group tracks".format(len(group_tracks))
            )


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
