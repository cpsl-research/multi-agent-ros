import rclpy
from avstack_bridge.objects import ObjectStateBridge
from avstack_msgs.msg import ObjectStateArray,ObjectState
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from vision_msgs.msg import BoundingBox3DArray,BoundingBox3D
from std_msgs.msg import Header,ColorRGBA
from avstack_msgs.msg import ObjectStateArray
from rclpy.node import Node


class AvstackBridgedVisualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.debug_logging = False
        
        self.object_bridge = ObjectStateBridge()

        #initialize different colors
        self.colors = {}
        self._init_colors()

        # subscribe to ground truth object information
        self.gt_subscription = None
        self.gt_publisher = None
        self.gt_init_pub_sub()

        #agent namespaces
        self._agent_namespaces = []

        #pub/sub for agent detections
        self._detections_pubs = {}
        self._detections_subs = {}

        #pub/sub for agent tracks
        self._tracks_pubs = {}
        self._tracks_subs = {}

        #start new agent discovery
        self.new_agent_discovery_timer = None
        self.new_agent_discovery_init_timer()

        return

    '''
    ##########################################################
    Ground Truth Initializations, Functions, and Callbacks
    ##########################################################
    '''
    def gt_init_pub_sub(self):
        # subscribe to ground truth object information
        self.gt_subscription = self.create_subscription(
            ObjectStateArray,
            "object_truth",
            self._gt_sub_callback,
            10
        )

        # publish the visualized markers
        self.gt_publisher = self.create_publisher(
            MarkerArray, "object_truth/markers", 10
        )

    def _gt_sub_callback(self, msg:ObjectStateArray):

        points = self._objectStateArray_to_markerArray(
            msg = msg,
            namespace="object_truth",
            color = self.colors["blue"]
        )
        self.gt_publisher.publish(points)
        # self.get_logger().info("Received {} objects".format(len(msg.states)))

    '''
    ##########################################################
    Finding new agents
    ##########################################################
    '''
    def new_agent_discovery_init_timer(self):
        self.new_agent_discovery_timer = self.create_timer(
            timer_period_sec = 1.0,
            callback = self.new_agent_discovery_callback
        )
    
    def new_agent_discovery_callback(self):
        
        #get the node names and namespaces
        node_names_and_namespaces = self.get_node_names_and_namespaces()

        #check for new agents
        for name,namespace in node_names_and_namespaces:
            if (
                ("ego" in namespace)
                or ("agent" in namespace)
                and ("command_center" not in namespace)
            ):
                if namespace not in self._agent_namespaces:
                    self.add_agent(
                        agent_namespace=namespace
                    )

    '''
    ##########################################################
    Initialize new agent
    ##########################################################
    '''
    def add_agent(self,agent_namespace:str):

        if agent_namespace not in self._agent_namespaces:

            #add namespace to existing list of namespaces
            self._agent_namespaces.append(agent_namespace)
            self.get_logger().info("Visualizer added {}".format(agent_namespace))

            #create a publisher/subscriber for the agent's detections
            self._detections_init_pub_sub(agent_namespace)

            #create a publisher/subscriber for the agent's tracks
            self._tracks_init_pub_sub(agent_namespace)


        else:
            self.get_logger().info("Visualizer attempted to add {}, but it already existed".format(agent_namespace))
    
    
    '''
    ##########################################################
    Detections Initializations, Functions, and Callbacks
    ##########################################################
    '''
    def _detections_init_pub_sub(self, agent_namespace:str):
        
        #initialize the subscriber
        self._detections_subs[agent_namespace] = self.create_subscription(
            BoundingBox3DArray,
            "{}/detections".format(agent_namespace),
            lambda msg: self._detections_sub_callback(msg,agent_namespace),
            10
        )

        self._detections_pubs[agent_namespace] = self.create_publisher(
            MarkerArray,
            "{}/markers/detections".format(agent_namespace),
            10
        )

    def _detections_sub_callback(self,msg:BoundingBox3DArray,agent_namespace:str):

        markers = self._boundingBox3DArray_to_markerArray(
            msg=msg,
            namespace=agent_namespace,
            color=self.colors["red"])
        self._detections_pubs[agent_namespace].publish(markers)
        if self.debug_logging:
            self.get_logger().info("Received {} detections from {}".format(len(msg.boxes),agent_namespace))

    '''
    ##########################################################
    Track Initializations, Functions, and Callbacks
    ##########################################################
    '''

    def _tracks_init_pub_sub(self, agent_namespace:str):
        
        #initialize the subscriber
        self._tracks_subs[agent_namespace] = self.create_subscription(
            ObjectStateArray,
            "{}/tracks".format(agent_namespace),
            lambda msg: self._tracks_sub_callback(msg,agent_namespace),
            10
        )

        self._tracks_pubs[agent_namespace] = self.create_publisher(
            MarkerArray,
            "{}/markers/tracks".format(agent_namespace),
            10
        )

    def _tracks_sub_callback(self,msg:ObjectStateArray,agent_namespace:str):

        markers = self._objectStateArray_to_markerArray(
            msg=msg,
            namespace=agent_namespace,
            color=self.colors["green"])
        self._tracks_pubs[agent_namespace].publish(markers)
        if self.debug_logging:
            self.get_logger().info("Received {} tracks from {}".format(len(msg.states),agent_namespace))

    
    '''
    ##########################################################
    Helper Fn's to convert to visualization messages
    ##########################################################
    '''

    def _boundingBox3DArray_to_markerArray(self,msg:BoundingBox3DArray,namespace,color:ColorRGBA) -> MarkerArray:

        # define a new marker array
        detections = MarkerArray()
        detections.markers = []

        header = msg.header

        for i in range(len(msg.boxes)):

            detections.markers.append(
                self._boundingBox3D_to_marker(
                    bounding_box_3D=msg.boxes[i],
                    header=header,
                    id = i,
                    namespace=namespace,
                    color=color
                )
            )

        return detections
    
    def _boundingBox3D_to_marker(self,
                            bounding_box_3D:BoundingBox3D,
                            header:Header,
                            id:int,
                            namespace:str,
                            color:ColorRGBA) -> Marker:

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
        marker.pose.orientation.w = -1.0 * marker.pose.orientation.w
        marker.pose.position = bounding_box_3D.center.position

        # define the color
        marker.color = color

        return marker

    def _objectStateArray_to_markerArray(self,msg:ObjectStateArray,namespace,color:ColorRGBA) -> MarkerArray:

        #define a new marker array
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
                    id = i,
                    namespace=namespace,
                    color=color
                )
            )
        
        return points
   

    def _objectState_to_marker(self,
                            object_state:ObjectState,
                            header:Header,
                            id:int,
                            namespace:str,
                            color:ColorRGBA) -> Marker:

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
        marker.pose.orientation.w = -1.0 * marker.pose.orientation.w
        marker.pose.position = object_state.box.center.position

        # define the color
        marker.color = color

        return marker

    '''
    ##########################################################
    Defining Custom Colors for Visualization
    ##########################################################
    '''
    def _init_colors(self):

        #primary colors
        self.colors["red"] = ColorRGBA(a=1.0,r=1.0,g=0.0,b=0.0)
        self.colors["green"] = ColorRGBA(a=1.0,r=0.0,g=1.0,b=0.0)
        self.colors["blue"] = ColorRGBA(a=1.0,r=0.0,g=0.0,b=1.0)

        self.colors["yellow"] = ColorRGBA(a=1.0,r=1.0,g=1.0,b=0.0)
        self.colors["purple"] = ColorRGBA(a=1.0,r=1.0,g=0.0,b=1.0)



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
