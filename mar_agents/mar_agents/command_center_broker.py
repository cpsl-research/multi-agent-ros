from functools import partial

import rclpy
from avstack.datastructs import DataManager
from avstack_bridge.base import Bridge
from avstack_msgs.msg import (
    BoxTrackArray,
    BoxTrackArrayWithSender,
    BoxTrackArrayWithSenderArray,
    BoxTrackStamped,
)
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class CommandCenterBroker(Node):
    """The broker is responsible for collating the
    information from the agents before sending it to the command
    center MODELS to be processed
    """

    def __init__(self):
        super().__init__("command_center_broker")

        # set to True for more detailed logging for debugging
        self.declare_parameter("debug", False)
        self.declare_parameter("output_folder", "outputs")
        self.debug = self.get_parameter("debug").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.agent_subscribers = {}

        # this defaults as a min-heap, so the top item has "lowest" time
        self.data_manager = DataManager(
            max_size=10
        )  # manages collating info from agents

        # discovery timer
        discovery_timer_period = 1.0  # run at 1 Hz
        self.discovery_timer = self.create_timer(
            discovery_timer_period, self.discovery_callback
        )

        # command center publishes collated group tracks
        publish_timer_period = 1.0 / 20  # run at 20 Hz
        self.datamanager_timer = self.create_timer(
            publish_timer_period, self.check_datamanager_callback
        )
        self.publisher_collated = self.create_publisher(
            BoxTrackArrayWithSenderArray, "collated", 10
        )

    def discovery_callback(self):
        """Set up a subscriber to this agent's track information"""
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        for _, namespace in node_names_and_namespaces:
            if (
                ("ego" in namespace)
                or ("agent" in namespace)
                and ("command_center" not in namespace)
            ):
                if namespace not in self.agent_subscribers:
                    self.agent_subscribers[namespace] = self.create_subscription(
                        BoxTrackArray,
                        f"{namespace}/tracks",
                        partial(self.tracks_callback, agent=namespace),
                        10,
                    )

    def tracks_callback(self, msg, agent):
        """Add new track information to the data manager"""
        data = (Bridge.rostime_to_time(msg.header.stamp), msg)
        self.data_manager.push(data, ID=agent)

    async def check_datamanager_callback(self):
        """Check if we are ready to send out the collated message

        NOTE: there are MANY ways that we could do this. One of which
        is just to check if all of the slots have at least 1 item and
        then send out when they do. This is vulnerable to when one of
        the agents goes offline, so it would be good to build in a
        latency threshold at some point for real physical systems.
        """
        if self.data_manager.n_buckets > 0:
            for ID in self.data_manager:
                if not self.data_manager.has_data(ID):
                    break
            else:  # all must have data if we made it here
                # take the oldest data from all and check the synchronicity
                data_arrays = self.data_manager.pop(s_ID=None, with_priority=False)
                to_frame = "world"
                track_arrays = []
                for ID, data in data_arrays.items():
                    # transform to the world
                    new_tracks = [
                        self._tf_buffer.transform(
                            object_stamped=BoxTrackStamped(
                                header=data.header, track=track
                            ),
                            target_frame=to_frame,
                        ).track
                        for track in data.tracks
                    ]
                    when = data.header.stamp
                    new_header = Header(frame_id=to_frame, stamp=when)

                    # save transformed track array
                    track_arrays.append(
                        BoxTrackArrayWithSender(
                            header=new_header, tracks=new_tracks, sender_id=ID
                        )
                    )
                header = Header(frame_id=to_frame, stamp=when)
                trk_arrarr_msg = BoxTrackArrayWithSenderArray(
                    header=header, track_arrays=track_arrays
                )
                self.publisher_collated.publish(trk_arrarr_msg)


def main(args=None):
    rclpy.init(args=args)

    cc = CommandCenterBroker()

    rclpy.spin(cc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
