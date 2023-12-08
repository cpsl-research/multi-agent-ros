"""
The nodes to run adversary agents
"""

import rclpy
from avstack_msgs.msg import BoxTrackArray, BoxTrackArrayWithSenderArray
from rclpy.node import Node
from ros2node.api import get_node_names
from vision_msgs.msg import BoundingBox3DArray


class AdversaryNode(Node):
    def __init__(self):
        super().__init__("adversary")

        self.declare_parameter(name="debug", value=True)
        self.declare_parameter(name="attack_is_coordinated", value=False)
        self.declare_parameter(name="attack_agent_name", value="agent0")

        # parameters for an uncoordinated attack
        self.declare_parameter(name="dt_init_uncoord", value=5.0)
        self.declare_parameter(name="fp_poisson_uncoord", value=6.0)
        self.declare_parameter(name="fn_fraction_uncoord", value=0.20)
        self.ready = False

        # set up a publisher for the output - type depends if coordinated
        if not self.get_parameter("attack_is_coordinated").value:
            if self.debug:
                self.get_logger().info("setting up an uncoordinated adversary!")

            # set a burnin timer to wait
            self.adv_init_timer = self.create_timer(
                self.get_parameter("dt_init_uncoord").value, self.adv_init_uncoord
            )

            # set up a subscriber to detections
            self.in_subscriber = self.create_subscription(
                BoundingBox3DArray, "detections", self.coordinated_output_callback, 10
            )

            # set up a publisher of detections
            self.out_publisher = self.create_publisher(BoundingBox3DArray, "output", 10)

        else:
            if self.debug:
                self.get_logger().info("setting up a coordinated adversary!")

            # set up a subscriber to the coordination adversary
            self.coord_subscriber = self.create_subscription(
                BoxTrackArrayWithSenderArray,
                "attack_coordination",
                self.coordinated_input_callback,
                10,
            )

            # set up a subscriber to the object tracks
            self.in_subscriber = self.create_subscription(
                BoxTrackArray, "tracks", self.uncoordinated_output_callback, 10
            )

            # set up a publisher of tracks
            self.out_publisher = self.create_publisher(BoxTrackArray, "output", 10)

    @property
    def name(self):
        return get_node_names(node=self)

    @property
    def debug(self):
        return self.get_parameter("debug").value

    def adv_init_uncoord(self):
        """On the init, we are ready"""
        if self.debug:
            self.get_logger().info("Uncoordinated adversary is ready!")
        self.ready = True
        self.adv_init_timer.cancel()

    def uncoordinated_output_callback(self, msg: BoundingBox3DArray):
        """Called when intercepting detections from the compromised agent/simulator"""
        if self.debug:
            self.get_logger().info(
                "Received {} detections at the adversary".format(len(msg.boxes))
            )

        if self.ready:
            # process inputs
            raise NotImplementedError
        else:
            msg_out = msg

        # send new outputs
        self.out_publisher.publish(msg_out)

    def coordinated_input_callback(self, msg: BoxTrackArrayWithSenderArray):
        """Called when receiving a directive from the coordinating adversary"""
        if self.debug:
            self.get_logger().info("Received directive from coordinating adversary")
        # TODO: set up directive...
        self.ready = True

    def coordinated_output_callback(self, msg: BoxTrackArray):
        """Called when intercepting tracks from the compromised agent"""
        if self.debug:
            self.get_logger().info(
                "Received {} tracks at the adversary".format(len(msg.tracks))
            )

        if self.ready:
            # process inputs
            pass
        else:
            msg_out = msg

        # send new outputs
        self.out_publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)

    adversary = AdversaryNode()

    rclpy.spin(adversary)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    adversary.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
