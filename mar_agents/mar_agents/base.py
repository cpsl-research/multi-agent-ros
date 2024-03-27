import os

from ament_index_python.packages import get_package_share_directory
from avstack.config import PIPELINE, Config
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.objects import ObjectStateBridge
from avstack_bridge.tracks import TrackBridge
from rclpy.node import Node
from ros2node.api import get_node_names
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer


class BaseAgent(Node):
    def __init__(self, name, default_pipeline):
        super().__init__(name)
        self.declare_parameter("namespace", "")
        self.declare_parameter("output_folder", "last_run")
        output_folder = os.path.join(
            self.get_parameter("output_folder").value,
            self.get_parameter("namespace").value,
        )

        # transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # bridge converters
        self.objs_bridge = ObjectStateBridge()
        self.dets_bridge = DetectionBridge()
        self.track_bridge = TrackBridge()

        # common parameters of all agents
        self.declare_parameter(name="pipeline", value=default_pipeline)
        self.pipeline_path = os.path.join(
            get_package_share_directory("mar_agents"),
            "config",
            "pipeline",
            self.get_parameter("pipeline").value,
        )
        if not os.path.exists(self.pipeline_path):
            raise FileNotFoundError(
                f"Cannot find pipeline path at {self.pipeline_path}"
            )

        self.pipeline_cfg = Config.fromfile(self.pipeline_path).pipeline
        self.pipeline = PIPELINE.build(
            self.pipeline_cfg,
            default_args={
                "name": self.get_parameter("namespace").value,
                "output_folder": output_folder,
            },
        )

    @property
    def name(self):
        return get_node_names(node=self)
