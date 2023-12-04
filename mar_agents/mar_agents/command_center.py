"""
The command center is a receiver of information from
other agents. The CC runs a pipeline across the 
information from all of the agents.
"""

import rclpy
from avstack_msgs.msg import ObjectStateArray
from rclpy.node import Node

from .base import BaseAgent


class CommandCenterBroker(Node):
    """The broker is responsible for collating the
    information from the agents before sending it to the command
    center algorithms to be processed
    """

    def __init__(self):
        super().__init__(name="command_center", default_pipeline="command_center.py")
        self.agent_subscribers = {}

        # discovery timer
        timer_period = 1.0  # run every 1 second
        self.timer = self.create_timer(timer_period, self.discovery_callback)

        # command center publishes group tracks
        self.publisher_tracks = self.create_publisher(ObjectStateArray, "tracks", 10)

    def discovery_callback(self):
        """Set up a subscriber to this agent's track information"""
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        for name, namespace in node_names_and_namespaces:
            if (
                ("ego" in namespace)
                or ("agent" in namespace)
                and ("command_center" not in namespace)
            ):
                if namespace not in self.agent_subscribers:
                    self.agent_subscribers[namespace] = self.create_subscription(
                        ObjectStateArray,
                        f"{namespace}/tracks",
                        self.tracks_callback,
                        10,
                    )

    def tracks_callback(self, msg):
        self.get_logger().info(
            f"Received {len(msg.states)} tracks from agent {msg.header.frame_id}"
        )


class CommandCenter(BaseAgent):
    pass

    # def tracks_on_timer(self)

    #     group_tracks = self.pipeline(
    #         tracks_in =
    #     )
    #     msg_track = self.track_bridge.avstack_to_tracks(
    #         group_tracks, tf_buffer=self.tf_buffer
    #     )
    #     self.publisher_tracks.publish(msg_track)


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
