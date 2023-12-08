"""
The nodes to run adversary agents
"""

import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names



class AdversaryNode(Node):
    def __init__(self):
        super().__init__("adversary")

        self.declare_parameter(name="attack_agent_name", value="agent1")

        self.get_logger().info("attacking {}".format(self.get_parameter("attack_agent_name").value))

    @property
    def name(self):
        return get_node_names(node=self)


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
