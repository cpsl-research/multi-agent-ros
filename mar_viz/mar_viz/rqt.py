import rclpy
from avstack_msgs.msg import BoxTrack, BoxTrackArray, ObjectState, ObjectStateArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray
from visualization_msgs.msg import Marker, MarkerArray


class AvstackBridgedVisualizer(Node):
    """Visualization Node that converts AvstackBridge messages
    and topics into MarkerArray messages that can easily be
    visualized in rviz2. The node automatically detects new agent
    namespaces with either "ego" or "agent" in the namespace and
    also publishes ground truth detections from the
    "object_truth" topic as a MarkerArray. Additional logging
    can be enabled by setting the "-debug" parameter to true.
    """

    def __init__(self):
        """Initialize a new visualizer node"""
        super().__init__("visualizer")

        # set to True for more detailed logging for debugging
        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value

        # initialize different colors
        self.colors = {}
        self._init_colors()

        # subscribe to ground truth object information
        self.gt_subscription = None
        self.gt_publisher = None
        self.gt_init_pub_sub()

        # agent namespaces/frame IDs
        self._agent_namespaces = []

        # pub/sub for agent detections
        self._detections_pubs = {}
        self._detections_subs = {}

        # pub/sub for agent tracks
        self._tracks_pubs = {}
        self._tracks_subs = {}

        # pub for individual agent positions
        self._agent_pose_pubs = {}

        # pub for all agent positions
        self._all_agent_poses_pub = None
        self._all_agent_poses_pub_init()

        # start new agent discovery
        self.new_agent_discovery_timer = None
        self.new_agent_discovery_init_timer()

    """
    ##########################################################
    Ground Truth Initializations, Functions, and Callbacks
    ##########################################################
    """

    def gt_init_pub_sub(self):
        """Initialize a publisher and subscriber for ground truth positions"""
        # subscribe to ground truth object information
        self.gt_subscription = self.create_subscription(
            ObjectStateArray, "object_truth", self._gt_sub_callback, 10
        )

        # publish the visualized markers
        self.gt_publisher = self.create_publisher(
            MarkerArray, "object_truth/markers", 10
        )

    def _gt_sub_callback(self, msg: ObjectStateArray):
        """Callback function for the ground truth position subscriber.
        If there are agents, additionally publishes the pose of each agent.

        Args:
            msg (ObjectStateArray): Array of ObjectState objects containing ground truth positions
        """
        # publish all of the ground truth object locations
        points = self._objectStateArray_to_markerArray(
            msg=msg, namespace="object_truth", color=self.colors["blue"]
        )
        self.gt_publisher.publish(points)

        # publish all current agent positions
        if len(self._agent_namespaces) > 0:
            agent_poses = self._all_agent_poses_generate_markerArray(
                stamp=msg.header.stamp, color=self.colors["yellow"]
            )

            self._all_agent_poses_pub.publish(agent_poses)

        if self.debug:
            self.get_logger().info("Received {} objects".format(len(msg.states)))

    """
    ##########################################################
    Finding new agents
    ##########################################################
    """

    def new_agent_discovery_init_timer(self):
        """Initializes a timer to check for new agents every 1.0 second"""
        self.new_agent_discovery_timer = self.create_timer(
            timer_period_sec=1.0, callback=self.new_agent_discovery_callback
        )

    def new_agent_discovery_callback(self):
        """Callback function to check for new agents.
        When a new agent is detected, publisher/subscribers
        for the agent's tracking/detections are initialized
        so that they can be visualized
        """
        # get the node names and namespaces
        node_names_and_namespaces = self.get_node_names_and_namespaces()

        # check for new agents
        for name, namespace in node_names_and_namespaces:
            if ("agent" in namespace) or ("command_center" in namespace):
                if namespace not in self._agent_namespaces:
                    self._add_agent(agent_namespace=namespace)

    """
    ##########################################################
    Initialize new agent
    ##########################################################
    """

    def _add_agent(self, agent_namespace: str):
        """Add a publisher/subsciber for an new agent's
        detections and tracking

        Args:
            agent_namespace (str): the agent's namespace (ex:"ego" or "agent1" or "command_center")
        """

        if agent_namespace not in self._agent_namespaces:

            # add namespace to existing list of namespaces
            self._agent_namespaces.append(agent_namespace)
            self.get_logger().info("Visualizer added {}".format(agent_namespace))

            # create a publisher/subscriber for the agent's detections
            self._detections_init_pub_sub(agent_namespace)

            # create a publisher/subscriber for the agent's tracks
            self._tracks_init_pub_sub(agent_namespace)

            # create a publisher/subscriber for the agent's pose
            self._agent_pose_init_pub(agent_namespace)

        else:
            self.get_logger().info(
                "Visualizer attempted to add {}, but it already existed".format(
                    agent_namespace
                )
            )

    """
    ##########################################################
    Detections Initializations, Functions, and Callbacks
    ##########################################################
    """

    def _detections_init_pub_sub(self, agent_namespace: str):
        """Initialize the publisher and subscriber for an
        agent's detections

        Args:
            agent_namespace (str): the agent's namespace
                (ex: "ego" or "agent1" or "command_center)
        """
        # initialize the subscriber
        self._detections_subs[agent_namespace] = self.create_subscription(
            BoundingBox3DArray,
            "{}/detections".format(agent_namespace),
            lambda msg: self._detections_sub_callback(msg, agent_namespace),
            10,
        )

        self._detections_pubs[agent_namespace] = self.create_publisher(
            MarkerArray, "{}/markers/detections".format(agent_namespace), 10
        )

    def _detections_sub_callback(self, msg: BoundingBox3DArray, agent_namespace: str):
        """Callback function for a given agent's detections
        subscriber.

        Args:
            msg (BoundingBox3DArray): array of bounding
                BoundingBox3D object's representing the
                agent's detections
            agent_namespace (str): the agent's namespace
                (ex: "ego" or "agent1" or "command_center")
        """

        markers = self._boundingBox3DArray_to_markerArray(
            msg=msg, namespace=agent_namespace, color=self.colors["red"]
        )
        self._detections_pubs[agent_namespace].publish(markers)
        if self.debug:
            self.get_logger().info(
                "Received {} detections from {}".format(len(msg.boxes), agent_namespace)
            )

    """
    ##########################################################
    Track Initializations, Functions, and Callbacks
    ##########################################################
    """

    def _tracks_init_pub_sub(self, agent_namespace: str):
        """Initialize the publisher and subscriber for an
        agent's tracks

        Args:
            agent_namespace (str): the agent's namespace
                (ex: "ego" or "agent1" or "command_center")
        """
        # initialize the subscriber
        self._tracks_subs[agent_namespace] = self.create_subscription(
            BoxTrackArray,
            "{}/tracks".format(agent_namespace),
            lambda msg: self._tracks_sub_callback(msg, agent_namespace),
            10,
        )

        self._tracks_pubs[agent_namespace] = self.create_publisher(
            MarkerArray, "{}/markers/tracks".format(agent_namespace), 10
        )

    def _tracks_sub_callback(self, msg: BoxTrackArray, agent_namespace: str):
        """Callback function for a given agent's tracks
        subscriber.

        Args:
            msg (ObjectStateArray): array of
                ObjectState objects representing the
                agent's detections
            agent_namespace (str): the agent's namespace
                (ex: "ego" or "agent1" or "command_center)
        """

        # publish the agent's tracks
        track_markers = self._boxTrackArray_to_markerArray(
            msg=msg, namespace=agent_namespace, color=self.colors["green"]
        )
        self._tracks_pubs[agent_namespace].publish(track_markers)
        if self.debug:
            self.get_logger().info(
                "Received {} tracks from {}".format(len(msg.states), agent_namespace)
            )

        # publish the agent's pose
        pose_marker = self._agent_pose_generate_marker(
            header=msg.header, namespace=agent_namespace, color=self.colors["yellow"]
        )

        self._agent_pose_pubs[agent_namespace].publish(pose_marker)
        if self.debug:
            self.get_logger().info("Published pose for {}".format(agent_namespace))

    """
    ##########################################################
    Agent Pose Initializations and Functions
    ##########################################################
    """

    def _agent_pose_init_pub(self, agent_namespace: str):
        """Initialize the publisher and subscriber for an
        agent's pose

        Args:
            agent_namespace (str): the agent's namespace
                (ex: "ego" or "agent1" or "command_center")
        """
        # initialize the publisher

        self._agent_pose_pubs[agent_namespace] = self.create_publisher(
            Marker, "{}/markers/agent_pose".format(agent_namespace), 10
        )

    def _all_agent_poses_pub_init(self):
        """Initialize a publisher that publishes a MarkerStateArray object
        of all current agent poses
        """

        # initialize the publisher

        self._all_agent_poses_pub = self.create_publisher(
            MarkerArray, "object_truth/agent_poses", 10
        )

        return

    def _all_agent_poses_generate_markerArray(
        self, stamp: Time, color: ColorRGBA
    ) -> MarkerArray:
        """Generate a marker array containing Marker objects at the position of
        each agent

        Args:
            stamp (Time): current time stamp in ROS2 time
            color (ColorRGBA): the color of the marker to use

        Returns:
            MarkerArray: MarkerArray object containing a marker for each agent
        """
        # define a new marker array
        points = MarkerArray()
        points.markers = []

        for i in range(len(self._agent_namespaces)):

            # initialize a new header object with the correct frame_id
            header = Header()
            header.stamp = stamp
            header.frame_id = self._agent_namespaces[i]

            # for each agent_namespace, generate a marker for the agent's pose
            points.markers.append(
                self._agent_pose_generate_marker(
                    header=header, namespace=self._agent_namespaces[i], color=color
                )
            )

        return points

    def _agent_pose_generate_marker(
        self,
        header: Header,
        namespace: str,
        color: ColorRGBA,
    ) -> Marker:
        """Helper Function to generate a marker for an agent's
        pose


        Args:
            header: the header to use for each Marker object
                (use the header from the ObjectStateArray
                object)
            id (int): a unique integer id for the given marker
            namespace (str): the namespace of the agent
                (ex: "ego" or "agent1" or "command_center")
            color (ColorRGBA): the desired color of each Marker
                expressed as a ROS2 ColorRGBA message type

        Returns:
            Marker: ROS2 Marker object
        """
        # define a new marker
        marker = Marker()

        # initialize the header
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp

        # define the marker attributes
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.CUBE

        # define the scale
        marker.scale.x = 4.0
        marker.scale.y = 2.5
        marker.scale.z = 2.0

        # set the behavior to ADD for now
        marker.action = Marker.ADD

        # define the pose and point
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # define the color
        marker.color = color

        return marker

    """
    ##########################################################
    Helper Fn's to convert to visualization messages
    ##########################################################
    """

    def _boundingBox3DArray_to_markerArray(
        self, msg: BoundingBox3DArray, namespace: str, color: ColorRGBA
    ) -> MarkerArray:
        """Helper Function to convert from a BoundingBox3DArray
        to a MarkerArray ROS2 message

        Args:
            msg (BoundingBox3DArray): the input
                BoundingBox3DArray message
            namespace (str): the namespace of the agent
                (ex: "ego" or "agent1" or "command_center")
            color (ColorRGBA): the desired color of each Marker
                expressed as a ROS2 ColorRGBA message type

        Returns:
            MarkerArray: Array of ROS2 Marker objects
        """
        # define a new marker array
        detections = MarkerArray()
        detections.markers = []

        header = msg.header

        for i in range(len(msg.boxes)):

            detections.markers.append(
                self._boundingBox3D_to_marker(
                    bounding_box_3D=msg.boxes[i],
                    header=header,
                    id=i + 1,
                    namespace=namespace,
                    color=color,
                )
            )

        return detections

    def _boundingBox3D_to_marker(
        self,
        bounding_box_3D: BoundingBox3D,
        header: Header,
        id: int,
        namespace: str,
        color: ColorRGBA,
    ) -> Marker:
        """Helper Function to convert from a BoundingBox3D
        to a Marker ROS2 message

        Args:
            bounding_box_3D (BoundingBox3D): the input
                BoundingBox3D message
            header: the header to use for each Marker object
                (use the header from the BoundingBox3DArray
                object)
            id (int): a unique integer id for the given marker
            namespace (str): the namespace of the agent
                (ex: "ego" or "agent1" or "command_center")
            color (ColorRGBA): the desired color of each Marker
                expressed as a ROS2 ColorRGBA message type

        Returns:
            Marker: ROS2 Marker object
        """
        # define a new marker
        marker = Marker()

        # initialize the header
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp

        # define the marker attributes
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.CUBE

        # define the scale
        marker.scale = bounding_box_3D.size

        # set the behavior to ADD for now
        marker.action = Marker.ADD

        # define the pose and point
        marker.pose.orientation = bounding_box_3D.center.orientation
        marker.pose.position = bounding_box_3D.center.position

        # define the color
        marker.color = color

        return marker

    def _objectStateArray_to_markerArray(
        self, msg: ObjectStateArray, namespace: str, color: ColorRGBA
    ) -> MarkerArray:
        """Helper Function to convert from a BoundingBox3DArray
        to a MarkerArray ROS2 message

        Args:
            msg (ObjectStateArray): the input
                ObjectStateArray message
            namespace (str): the namespace of the agent
                (ex: "ego" or "agent1" or "command_center")
            color (ColorRGBA): the desired color of each Marker
                expressed as a ROS2 ColorRGBA message type

        Returns:
            MarkerArray: Array of ROS2 Marker objects
        """
        # define a new marker array
        points = MarkerArray()
        points.markers = []

        # get the header from the object_state_array
        header = msg.header

        for i in range(len(msg.states)):

            # for each objectState in states, generate a new marker
            points.markers.append(
                self._objectState_to_marker(
                    object_state=msg.states[i],
                    header=header,
                    id=i + 1,
                    namespace=namespace,
                    color=color,
                )
            )

        return points

    def _boxTrackArray_to_markerArray(
        self,
        msg: BoxTrackArray,
        **kwargs,
    ) -> MarkerArray:
        """Wrapper between box track and object state array"""
        states = [self._boxTrack_to_objectState(track) for track in msg.tracks]
        objs_msg = ObjectStateArray(header=msg.header, states=states)
        return self._objectStateArray_to_markerArray(objs_msg, **kwargs)

    @staticmethod
    def _boxTrack_to_objectState(msg: BoxTrack) -> ObjectState:
        return ObjectState(
            obj_type=msg.obj_type,
            pose=msg.box.center,
            twist=Twist(linear=msg.velocity),
            box=msg.box,
        )

    def _objectState_to_marker(
        self,
        object_state: ObjectState,
        header: Header,
        id: int,
        namespace: str,
        color: ColorRGBA,
    ) -> Marker:
        """Helper Function to convert from a ObjectState
        object to a Marker ROS2 message

        Args:
            object_state (ObjectState): the input
                ObjectState message
            header: the header to use for each Marker object
                (use the header from the ObjectStateArray
                object)
            id (int): a unique integer id for the given marker
            namespace (str): the namespace of the agent
                (ex: "ego" or "agent1" or "command_center")
            color (ColorRGBA): the desired color of each Marker
                expressed as a ROS2 ColorRGBA message type

        Returns:
            Marker: ROS2 Marker object
        """
        # define a new marker
        marker = Marker()

        # initialize the header
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp

        # define the marker attributes
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.CUBE

        # define the scale
        marker.scale = object_state.box.size

        # set the behavior to ADD for now
        marker.action = Marker.ADD

        # define the pose and point
        marker.pose.orientation = object_state.box.center.orientation
        marker.pose.position = object_state.box.center.position

        # define the color
        marker.color = color

        return marker

    """
    ##########################################################
    Defining Custom Colors for Visualization
    ##########################################################
    """

    def _init_colors(self):
        """initializes a dictionary of common colors as a
        dictionary of ColorRGBA objects (key is color's name)
        NOTE: supported colors - "red","green","blue","yellow","purple"
        """

        # primary colors
        self.colors["red"] = ColorRGBA(a=1.0, r=1.0, g=0.0, b=0.0)
        self.colors["green"] = ColorRGBA(a=1.0, r=0.0, g=1.0, b=0.0)
        self.colors["blue"] = ColorRGBA(a=1.0, r=0.0, g=0.0, b=1.0)

        self.colors["yellow"] = ColorRGBA(a=1.0, r=1.0, g=1.0, b=0.0)
        self.colors["purple"] = ColorRGBA(a=1.0, r=1.0, g=0.0, b=1.0)


def main(args=None):
    rclpy.init(args=args)

    visualizer = AvstackBridgedVisualizer()

    rclpy.spin(visualizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
