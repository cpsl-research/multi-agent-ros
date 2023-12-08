"""
Runs the coordinator for the adversaries
"""

from functools import partial

import rclpy
from avstack.datastructs import DataManager
from avstack_bridge.base import Bridge
from avstack_msgs.msg import BoxTrackArray, BoxTrackArrayWithSenderArray
from rclpy.node import Node
from ros2node.api import get_node_names


class AdversaryCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("adversary_coordinator")

        self.declare_parameter(name="debug", value=True)

        # parameters for a coordinated attack
        self.declare_parameter("fp_poisson_coord", 10)
        self.declare_parameter("fn_fraction_coord", 0.10)
        self.declare_parameter("dt_init_adv_coord", 5.0)

        # get the init time
        self.adv_init_timer = self.create_timer(
            self.get_parameter("dt_init_adv").value, self.adv_init
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
        )  # manages collating info from agents

        # set up publisher on a timer that sends unified attack objectives
        self.adv_publisher = self.create_publisher(
            BoxTrackArrayWithSenderArray, "attack_coordination", 10
        )

    @property
    def name(self):
        return get_node_names(node=self)

    @property
    def debug(self):
        return self.get_parameter("debug").value

    @property
    def n_adversaries(self):
        return len(self.adv_subscribers)

    def adv_init(self):
        """On the init, create the callback function"""
        if self.debug:
            self.get_logger().info("Attack coordinator is ready!")

        # cancel the initialization timer
        self.adv_init_timer.cancel()

        # timer for future calls
        adv_timer_period = 10  # run again every 10 seconds
        self.adv_timer = self.create_timer(adv_timer_period, self.adv_callback)

        # run the callback once now
        self.adv_callback()

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

    def tracks_callback(self, msg, agent):
        """Add new track information to the data manager"""
        data = (Bridge.rostime_to_time(msg.header.stamp), msg)
        self.data_manager.push(data, ID=agent)

    def adv_callback(self):
        """Determines attack objectives in the world frame and publishes

        every time you call this function, it resets the attack objectives
        """

        if self.debug:
            self.get_logger().info("Running adv callback!")

        # first: false positives -- random tracks in world frame
        # n_fps = np.random.poisson(self.get_parameter("fp_poisson").value)

        # # second: false negatives -- random tracks from the data manager
        # for agent in self.adv_subscribers:
        #     tracks = self.data_manager.top(s_ID=agent)
        #     tracks_fp


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
