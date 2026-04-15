import random
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray


class WorldManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("world_manager_node")
        self.declare_parameter("world_mode", "random")
        self.declare_parameter("seed_mode", "none")
        self.declare_parameter("seed_value", 42)
        self.declare_parameter("seed_sequence", [42, 43, 44, 45])
        self.declare_parameter("obstacle_density", 0.3)
        self.declare_parameter("world_half_extent", 10.0)

        obstacle_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.obstacles_pub = self.create_publisher(
            MarkerArray,
            "/world_manager/obstacles",
            obstacle_qos,
        )
        self.reset_sub = self.create_subscription(
            Empty,
            "/scheduler/reset_world",
            self.on_reset_world,
            10,
        )

        self._world_reset_count = 0

        self._obstacles = self._generate_obstacles()
        self._publish_obstacles()

    def _generate_obstacles(self) -> List[Tuple[float, float, float, float]]:
        world_mode = self.get_parameter("world_mode").get_parameter_value().string_value
        seed_mode = self.get_parameter("seed_mode").get_parameter_value().string_value
        seed_value = self.get_parameter("seed_value").get_parameter_value().integer_value
        seed_sequence = list(self.get_parameter("seed_sequence").get_parameter_value().integer_array_value)
        density = self.get_parameter("obstacle_density").get_parameter_value().double_value
        half_extent = self.get_parameter("world_half_extent").get_parameter_value().double_value

        if seed_mode == "single":
            random.seed(seed_value)
        elif seed_mode == "sequence":
            if seed_sequence:
                idx = self._world_reset_count % len(seed_sequence)
                random.seed(seed_sequence[idx])
            else:
                random.seed(seed_value)
        elif seed_mode == "none":
            random.seed()

        if world_mode == "fixed":
            obstacles = [
                (-5.0, -1.5, 2.0, 2.0),
                (-0.5, 3.0, 1.5, 2.4),
                (3.2, -2.3, 2.2, 1.8),
                (5.5, 4.2, 1.8, 2.2),
                (0.0, -5.5, 3.0, 1.4),
            ]
        else:
            count = max(1, int(12 * density))
            obstacles = []
            for _ in range(count):
                x = random.uniform(-half_extent + 1.0, half_extent - 1.0)
                y = random.uniform(-half_extent + 1.0, half_extent - 1.0)
                sx = random.uniform(0.8, 2.8)
                sy = random.uniform(0.8, 2.8)
                obstacles.append((x, y, sx, sy))
        return obstacles

    def _publish_obstacles(self) -> None:
        arr = MarkerArray()
        for i, (x, y, sx, sy) in enumerate(self._obstacles):
            mk = Marker()
            mk.header.frame_id = "world"
            mk.ns = "obstacles"
            mk.id = i
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position = Point(x=x, y=y, z=0.5)
            mk.scale.x = sx
            mk.scale.y = sy
            mk.scale.z = 1.0
            mk.color.a = 0.8
            mk.color.r = 0.2
            mk.color.g = 0.6
            mk.color.b = 0.8
            arr.markers.append(mk)
        self.obstacles_pub.publish(arr)
        self.get_logger().info(f"Published {len(arr.markers)} obstacles")

    def on_reset_world(self, _msg: Empty) -> None:
        self._world_reset_count += 1
        self._obstacles = self._generate_obstacles()
        self._publish_obstacles()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
