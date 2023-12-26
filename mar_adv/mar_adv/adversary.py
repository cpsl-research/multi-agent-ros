"""
The nodes to run adversary agents
"""

import numpy as np
import rclpy
from avstack.datastructs import PriorityQueue
from avstack_bridge.base import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import (
    BoxTrackArray,
    BoxTrackArrayWithSenderArray,
    BoxTrackStamped,
)
from rclpy.node import Node
from ros2node.api import get_node_names
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from vision_msgs.msg import BoundingBox3DArray

from mar_adv.selection import (
    TargetObject,
    select_false_negatives,
    select_false_positives,
)


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

        # parameters for a coordinated attack
        self.declare_parameter(
            name="attack_coord_topic", value="/adversary_coordinator/attack_directive"
        )

        self.ready = False
        self.init_targets = False
        self.targets = {"false_positive": [], "false_negative": []}

        # transform listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # allow for a data buffer to store the last data from the agent
        self.data_buffer = PriorityQueue(max_size=5, max_heap=True)

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
                BoundingBox3DArray, "detections", self.uncoordinated_output_callback, 10
            )

            # set up a publisher of detections
            self.out_publisher = self.create_publisher(BoundingBox3DArray, "output", 10)

        else:
            if self.debug:
                self.get_logger().info("setting up a coordinated adversary!")

            # set up a subscriber to the coordination adversary
            self.coord_subscriber = self.create_subscription(
                BoxTrackArrayWithSenderArray,
                self.get_parameter("attack_coord_topic").value,
                self.coordinated_input_callback,
                10,
            )

            # set up a subscriber to the object tracks
            self.in_subscriber = self.create_subscription(
                BoxTrackArray, "tracks", self.coordinated_output_callback, 10
            )

            # set up a publisher of tracks
            self.out_publisher = self.create_publisher(BoxTrackArray, "output", 10)

    @property
    def name(self):
        return get_node_names(node=self)

    @property
    def debug(self):
        return self.get_parameter("debug").value

    def reset_targets(self):
        self.targets = {"false_positive": [], "false_negative": []}

    def adv_init_uncoord(self):
        """On the init, we are ready"""
        if self.debug:
            self.get_logger().info("Uncoordinated adversary is ready!")
        self.reset_targets()
        self.ready = True
        self.adv_init_timer.cancel()

    def process_targets(self, objects, coordinated: bool, time: float, fn_threshold=6):
        """Processing the targets for the attacks

        False positives:
        if we have already created a list of false positives, just propagate
        those in time and append to the list of existing objects

        False negatives:
        find if there is a track with the same ID as identified before or one
        that is sufficiently close in space to the target and eliminate
        it from the outgoing message.
        """

        # process false positives
        for obj_fp in self.targets["false_positive"]:
            obj_fp.propagate(dt=(time - obj_fp.t))
            if coordinated:
                obj_fp_convert = obj_fp.as_track()
            else:
                obj_fp_convert = obj_fp.as_detection()
            objects.append(obj_fp_convert)

        # process false negatives
        for obj_fn in self.targets["false_negative"]:
            # perform assignment of existing detections/tracks to targets
            dists = [
                obj.position.distance(obj_fn.last_position, check_reference=False)
                for obj in objects
            ]
            idx_select = np.argmin(dists)
            if dists[idx_select] <= fn_threshold:
                # remove the ones that were assigned
                obj_fn.last_position = objects[idx_select].position
                del objects[idx_select]

        return objects

    def uncoordinated_output_callback(self, msg: BoundingBox3DArray):
        """Called when intercepting detections from the compromised agent/simulator"""
        if self.debug:
            self.get_logger().info(
                "Received {} detections at the adversary".format(len(msg.boxes))
            )
        self.data_buffer.push(
            priority=Bridge.rostime_to_time(msg.header.stamp),
            item=DetectionBridge.detections_to_avstack(msg),
        )

        # only select targets if we are in an uncoordinated attack
        if self.ready:
            # first is target selection
            if not self.init_targets:
                # select false positive objects randomly in space
                self.targets["false_positive"] = select_false_positives(
                    fp_poisson=self.get_parameter("fp_poisson_uncoord").value,
                    reference=Bridge.header_to_reference(msg.header),
                )

                # select false negative targets from existing objects
                self.targets["false_negative"] = select_false_negatives(
                    existing_objects=self.data_buffer.top(with_priority=False),
                    fn_fraction=self.get_parameter("fn_fraction_uncoord").value,
                )

                self.init_targets = True
                if self.debug:
                    self.get_logger().info(
                        "Initialized {} fp and {} fn targets for uncoordinated".format(
                            len(self.targets["false_positive"]),
                            len(self.targets["false_negative"]),
                        )
                    )

            # process inputs
            objects = self.process_targets(
                objects=DetectionBridge.detections_to_avstack(msg),
                coordinated=False,
                time=Bridge.rostime_to_time(msg.header.stamp),
            )
            msg_out = DetectionBridge.avstack_to_detections(objects, header=msg.header)
        else:
            msg_out = msg

        # send new outputs
        self.out_publisher.publish(msg_out)

    async def coordinated_input_callback(self, msg: BoxTrackArrayWithSenderArray):
        """Called when receiving a directive from the coordinating adversary"""
        if self.debug:
            self.get_logger().info("Received directive from coordinating adversary")

        self.ready = True
        self.init_targets = True
        self.reset_targets()

        if not len(msg.track_arrays) == 2:
            raise ValueError("Input must be of length 2 -- FP and FN")
        else:
            to_frame = self.get_parameter("attack_agent_name").value
            for obj_fp in msg.track_arrays[0].tracks:
                obj_fp_stamped = BoxTrackStamped(header=msg.header, track=obj_fp)
                obj_fp_in_agent_frame = self._tf_buffer.transform(
                    object_stamped=obj_fp_stamped,
                    target_frame=to_frame,
                )
                obj_fp_avstack = TrackBridge.boxtrack_to_avstack(obj_fp_in_agent_frame)
                self.targets["false_positive"].append(
                    TargetObject(obj_state=obj_fp_avstack)
                )

            # fn targets are in the world coordinate frame -- convert to agent
            # for obj_fn in msg.track_arrays[1].tracks:
            #     obj_fn_in_agent_frame = do_transform_boxtrack(obj_fn, tf_to_agent)
            #     obj_fn_avstack = TrackBridge.boxtrack_to_avstack(
            #         obj_fn_in_agent_frame, header=tf_to_agent.header
            #     )
            #     self.targets["false_negative"].append(TargetObject(obj_state=obj_fn_avstack))

        if self.debug:
            self.get_logger().info(
                "Initialized {} fp and {} fn targets for coordinated".format(
                    len(self.targets["false_positive"]),
                    len(self.targets["false_negative"]),
                )
            )

    def coordinated_output_callback(
        self, msg: BoxTrackArray, threshold_obj_dist: float = 70.0
    ):
        """Called when intercepting tracks from the compromised agent"""
        self.data_buffer.push(
            priority=Bridge.rostime_to_time(msg.header.stamp),
            item=TrackBridge.tracks_to_avstack(msg),
        )

        if self.ready:
            # process inputs
            objects = self.process_targets(
                objects=TrackBridge.tracks_to_avstack(msg),
                coordinated=True,
                time=Bridge.rostime_to_time(msg.header.stamp),
            )

            # filter objects greater than a threshold
            objects = [
                obj for obj in objects if obj.position.norm() < threshold_obj_dist
            ]
            msg_out = TrackBridge.avstack_to_tracks(
                objects, header=msg.header, default_type=BoxTrackArray
            )
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
