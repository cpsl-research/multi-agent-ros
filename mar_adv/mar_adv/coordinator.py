"""
Runs the coordinator for the adversaries
"""

from functools import partial

import rclpy
from avstack.datastructs import DataManager
from avstack.modules.clustering import SampledAssignmentClusterer
from avstack_bridge.base import Bridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import (
    BoxTrackArray,
    BoxTrackArrayWithSender,
    BoxTrackArrayWithSenderArray,
    BoxTrackStamped,
)
from rclpy.node import Node
from ros2node.api import get_node_names
from std_msgs.msg import Header
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from .selection import select_false_negatives, select_false_positives


class AdversaryCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("adversary_coordinator")

        self.declare_parameter(name="debug", value=True)

        # parameters for a coordinated attack
        self.declare_parameter("fp_poisson_coord", 10)
        self.declare_parameter("fn_fraction_coord", 0.10)
        self.declare_parameter("dt_init_adv_coord", 5.0)
        self.declare_parameter("attack_coord_topic", value="attack_directive")

        # transform listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # get the init time
        self.adv_init_timer = self.create_timer(
            self.get_parameter("dt_init_adv_coord").value, self.adv_init
        )

        # set up subscriber to the adversaries
        self.adv_subscribers = {}
        discovery_timer_period = 1.0  # run at 1 Hz
        self.discovery_timer = self.create_timer(
            discovery_timer_period, self.discovery_callback
        )

        # this defaults as a max-heap, so the top item has "highest" time
        self.data_manager = DataManager(
            max_size=10,
            max_heap=True,
            circular=True,
        )  # manages collating info from agents
        self.clusterer = SampledAssignmentClusterer(assign_radius=5)

        # set up publisher on a timer that sends unified attack objectives
        self.adv_publisher = self.create_publisher(
            BoxTrackArrayWithSenderArray,
            self.get_parameter("attack_coord_topic").value,
            10,
        )
        self.targets = {"false_positive": [], "false_negative": []}

    @property
    def name(self):
        return get_node_names(node=self)

    @property
    def debug(self):
        return self.get_parameter("debug").value

    @property
    def n_adversaries(self):
        return len(self.adv_subscribers)

    async def adv_init(self):
        """On the init, create the callback function"""
        if self.debug:
            self.get_logger().info("Attack coordinator is ready!")

        # cancel the initialization timer
        self.adv_init_timer.cancel()

        # timer for future calls
        adv_timer_period = 20  # run again every 10 seconds
        self.adv_timer = self.create_timer(adv_timer_period, self.adv_callback)

        # run the callback once now
        await self.adv_callback()

    def discovery_callback(self):
        """Set up subscribers to any adversaries that exist"""
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        for _, namespace in node_names_and_namespaces:
            if "adversary" in namespace:
                if namespace not in self.adv_subscribers:
                    self.adv_subscribers[namespace] = self.create_subscription(
                        BoxTrackArray,
                        f"{namespace}/tracks",
                        partial(self.tracks_callback, agent=namespace),
                        10,
                    )

    def tracks_callback(self, msg: BoxTrackArray, agent: str):
        """Add new track information to the data manager"""
        data = (Bridge.rostime_to_time(msg.header.stamp), msg)
        self.data_manager.push(data, ID=agent)
        # self.get_logger().info("ID: {}, top is {}".format(agent, self.data_manager.top(s_ID=agent)[0]))

    async def adv_callback(self):
        """Determines attack objectives in the world frame and publishes

        every time you call this function, it resets the attack objectives
        """

        if self.debug:
            self.get_logger().info("Running adv callback!")

        # select false positive objects randomly in space -- in world frame
        self.targets["false_positive"] = select_false_positives(
            fp_poisson=self.get_parameter("fp_poisson_coord").value,
            reference=None,
        )

        # select false negative targets from existing objects
        objects = self.data_manager.top(with_priority=False)
        world_tracks = {}
        for ID, data in objects.items():
            # Suspends callback until transform becomes available
            to_frame = "world"
            when = data.header.stamp
            world_tracks[ID] = [
                TrackBridge.boxtrack_to_avstack(
                    self._tf_buffer.transform(
                        object_stamped=BoxTrackStamped(header=data.header, track=track),
                        target_frame=to_frame,
                    )
                )
                for track in data.tracks
            ]
        if len(world_tracks) == 0:
            raise RuntimeError

        clusters = self.clusterer(
            frame=0,
            timestamp=Bridge.rostime_to_time(when),
            objects=world_tracks,
            check_reference=False,
        )
        existing_objects = [cluster.sample() for cluster in clusters]
        self.targets["false_negative"] = select_false_negatives(
            existing_objects=existing_objects,
            fn_fraction=self.get_parameter("fn_fraction_coord").value,
        )

        # send out coordinating adversary messages
        header = Header(frame_id="world", stamp=when)  # stamp doesn't really matter...
        fp_tracks = BoxTrackArrayWithSender(
            header=header,
            tracks=[
                TrackBridge.avstack_to_boxtrack(target.as_track())
                for target in self.targets["false_positive"]
            ],
            sender_id="false_positive",
        )
        fn_tracks = BoxTrackArrayWithSender(
            header=header,
            tracks=[
                TrackBridge.avstack_to_boxtrack(target.as_track())
                for target in self.targets["false_negative"]
            ],
            sender_id="false_negative",
        )
        msg_out = BoxTrackArrayWithSenderArray(
            header=header, track_arrays=[fp_tracks, fn_tracks]
        )
        self.adv_publisher.publish(msg_out)

        if self.debug:
            self.get_logger().info("Sending out coordination message")


def main(args=None):
    rclpy.init(args=args)

    coord = AdversaryCoordinator()

    rclpy.spin(coord)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    coord.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
