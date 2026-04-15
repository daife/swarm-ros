from functools import partial
from typing import Dict, List, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool


class SimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_bridge_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("pursuer_name_prefix", "pursuer")
        self.declare_parameter("evader_name_prefix", "evader")

        num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value
        pursuer_prefix = self.get_parameter("pursuer_name_prefix").get_parameter_value().string_value
        evader_prefix = self.get_parameter("evader_name_prefix").get_parameter_value().string_value

        self.agent_names: List[Tuple[int, str]] = []
        for i in range(num_pursuers):
            self.agent_names.append((i, f"{pursuer_prefix}_{i}"))
        for i in range(num_evaders):
            self.agent_names.append((num_pursuers + i, f"{evader_prefix}_{i}"))

        self.odom_pubs: Dict[int, object] = {}
        self.collision_pubs: Dict[int, object] = {}

        for agent_id, model_name in self.agent_names:
            model_odom_topic = f"/model/{model_name}/odometry"
            model_contacts_topic = f"/model/{model_name}/contacts"
            odom_topic = f"/agent_{agent_id}/odom"
            collision_topic = f"/agent_{agent_id}/collision"

            self.odom_pubs[agent_id] = self.create_publisher(Odometry, odom_topic, 10)
            self.collision_pubs[agent_id] = self.create_publisher(Bool, collision_topic, 10)

            self.create_subscription(
                Odometry,
                model_odom_topic,
                partial(self.on_model_odom, agent_id),
                10,
            )
            self.create_subscription(
                Contacts,
                model_contacts_topic,
                partial(self.on_model_contacts, agent_id),
                10,
            )

        self.get_logger().info(
            f"Sim bridge online for {len(self.agent_names)} agents; odom+contacts forwarding enabled"
        )

    def on_model_odom(self, agent_id: int, msg: Odometry) -> None:
        self.odom_pubs[agent_id].publish(msg)

    def on_model_contacts(self, agent_id: int, msg: Contacts) -> None:
        out = Bool()
        out.data = len(msg.contacts) > 0
        self.collision_pubs[agent_id].publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
