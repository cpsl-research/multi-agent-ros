import rclpy
from rclpy.node import Node

from avstack_bridge.objects import ObjectStateBridge
from avstack_msgs.msg import ObjectStateArray,ObjectState
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from vision_msgs.msg import BoundingBox3DArray,BoundingBox3D
from std_msgs.msg import Header
from avstack_msgs.msg import ObjectStateArray
from rclpy.node import Node


class AvstackBridgedVisualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.object_bridge = ObjectStateBridge()

        self.gt_visualization_method = "bb" #"arrow" or "bb" (bounding boxes)
        
        # subscribe to ground truth object information
        self.gt_subscription = None
        self.gt_publisher = None
        self.gt_init_pub_sub()
        #pub/sub for the ego detections
        self.ego_subscription = self.create_subscription(
            BoundingBox3DArray,
            "ego/detections",
            lambda msg: self.detections_sub_callback(msg,"ego"),
            10
        )

        self.ego_publisher = self.create_publisher(
            MarkerArray,
            "ego",
            10
        )
    
    def gt_init_pub_sub(self):
        # subscribe to ground truth object information
        self.gt_subscription = self.create_subscription(
            ObjectStateArray,
            "object_truth",
            self.gt_sub_callback,
            10
        )

        #publish the visualized markers
        self.gt_publisher = self.create_publisher(
            MarkerArray,
            "object_truth/markers",
            10
        )

    def gt_sub_callback(self, msg:ObjectStateArray):

        points = self.objectStateArray_to_markerArray(msg)
        self.gt_publisher.publish(points)
        self.get_logger().info("Received {} objects".format(len(msg.states)))

    def detections_sub_callback(self,msg:BoundingBox3DArray,agent:str):

        markers = self.boundingBox3DArray_to_markerArray(msg)
        self.ego_publisher.publish(markers)
        self.get_logger().info("Received {} detections from {}".format(len(msg.boxes),agent))

    def boundingBox3DArray_to_markerArray(self,msg:BoundingBox3DArray) -> MarkerArray:

        #define a new marker array
        detections = MarkerArray()
        detections.markers = []

        header = msg.header

        for i in range(len(msg.boxes)):

            detections.markers.append(
                self.boundingBox3D_to_marker(
                    bounding_box_3D=msg.boxes[i],
                    header=header,
                    id = i,
                    name_space="ego"
                )
            )
        
        return detections
    
    def boundingBox3D_to_marker(self,
                            bounding_box_3D:BoundingBox3D,
                            header:Header,
                            id:int,
                            name_space:str="ego") -> Marker:

        #define a new marker
        marker = Marker()

        #initialize the header
        marker.header.frame_id = "ego"
        marker.header.stamp = header.stamp

        #define the marker attributes
        marker.ns = name_space
        marker.id = id
        marker.type = Marker.CUBE
        
        #define the scale
        marker.scale = bounding_box_3D.size


        #set the behavior to ADD for now
        marker.action = Marker.ADD

        #define the pose and point
        marker.pose.orientation = bounding_box_3D.center.orientation
        marker.pose.orientation.w = -1.0 * marker.pose.orientation.w
        marker.pose.position = bounding_box_3D.center.position

        #define the color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        return marker


    def objectStateArray_to_markerArray(self,msg:ObjectStateArray) -> MarkerArray:

        #define a new marker array
        points = MarkerArray()
        points.markers = []

        #get the header from the object_state_array
        header = msg.header

        for i in range(len(msg.states)):

            # for each objectState in states, generate a new marker
            if self.gt_visualization_method == "arrow":
                points.markers.append(
                    self.objectState_to_marker_arrows(
                        object_state=msg.states[i],
                        header=header,
                        id = i,
                        name_space="gt"
                    )
                )
            elif self.gt_visualization_method == "bb":

                points.markers.append(
                    self.objectState_to_marker_bb(
                        object_state=msg.states[i],
                        header=header,
                        id = i,
                        name_space="gt"
                    )
                )
        
        return points


    def objectState_to_marker_arrows(self,
                            object_state:ObjectState,
                            header:Header,
                            id:int,
                            name_space:str="gt") -> Marker:

        #define a new marker
        marker = Marker()

        #initialize the header
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp

        #define the marker attributes
        marker.ns = name_space
        marker.id = id
        marker.type = Marker.ARROW
        
        #define the scale
        marker.scale.x = 5.0
        marker.scale.y = 2.5
        marker.scale.z = 3.0


        #set the behavior to ADD for now
        marker.action = Marker.ADD

        #define the pose and point
        marker.pose.orientation = object_state.attitude
        marker.pose.orientation.w = -1.0 * marker.pose.orientation.w
        marker.pose.position = object_state.position

        #define the color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        return marker

    def objectState_to_marker_bb(self,
                            object_state:ObjectState,
                            header:Header,
                            id:int,
                            name_space:str="gt") -> Marker:

        #define a new marker
        marker = Marker()

        #initialize the header
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp

        #define the marker attributes
        marker.ns = name_space
        marker.id = id
        marker.type = Marker.CUBE
        
        #define the scale
        marker.scale = object_state.box.size


        #set the behavior to ADD for now
        marker.action = Marker.ADD

        #define the pose and point
        marker.pose.orientation = object_state.box.center.orientation
        marker.pose.orientation.w = -1.0 * marker.pose.orientation.w
        marker.pose.position = object_state.box.center.position

        #define the color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        return marker

    


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
