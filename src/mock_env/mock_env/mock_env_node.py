import math
import random
from functools import partial
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Accel
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray


class MockEnvNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_env_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("world_half_extent", 10.0)
        self.declare_parameter("max_speed", 3.0)
        self.declare_parameter("linear_drag", 0.15)
        self.declare_parameter("spawn_half_extent", 8.0)

        self.num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        self.num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value
        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.world_half_extent = self.get_parameter("world_half_extent").get_parameter_value().double_value
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        self.linear_drag = self.get_parameter("linear_drag").get_parameter_value().double_value
        spawn_half_extent = self.get_parameter("spawn_half_extent").get_parameter_value().double_value

        self.agent_ids = list(range(self.num_pursuers + self.num_evaders))
        self.positions: Dict[int, np.ndarray] = {}
        self.velocities: Dict[int, np.ndarray] = {}
        self.accels: Dict[int, np.ndarray] = {}
        self.obstacles: List[Tuple[float, float, float, float]] = []
        self.tick_count = 0

        for agent_id in self.agent_ids:
            self.positions[agent_id] = np.array(
                [
                    random.uniform(-spawn_half_extent, spawn_half_extent),
                    random.uniform(-spawn_half_extent, spawn_half_extent),
                ],
                dtype=float,
            )
            self.velocities[agent_id] = np.zeros(2, dtype=float)
            self.accels[agent_id] = np.zeros(2, dtype=float)

        self.odom_pubs = {}
        self.collision_pubs = {}
        for agent_id in self.agent_ids:
            self.create_subscription(Accel, f"/agent_{agent_id}/cmd_accel", partial(self.on_cmd, agent_id), 10)
            self.odom_pubs[agent_id] = self.create_publisher(Odometry, f"/agent_{agent_id}/odom", 10)
            self.collision_pubs[agent_id] = self.create_publisher(Bool, f"/agent_{agent_id}/collision", 10)

        self.create_subscription(MarkerArray, "/world_manager/obstacles", self.on_obstacles, 10)

        self.create_timer(self.dt, self.step)
        self.get_logger().info("Mock env is running and publishing /agent_{i}/odom + /agent_{i}/collision")

    def on_cmd(self, agent_id: int, msg: Accel) -> None:
        self.accels[agent_id][0] = msg.linear.x
        self.accels[agent_id][1] = msg.linear.y

    def on_obstacles(self, msg: MarkerArray) -> None:
        self.obstacles = []
        for marker in msg.markers:
            self.obstacles.append(
                (
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.scale.x,
                    marker.scale.y,
                )
            )

    def step(self) -> None:
        try:
            self.tick_count += 1
            for agent_id in self.agent_ids:
                vel = self.velocities[agent_id]
                pos = self.positions[agent_id]
                acc = self.accels[agent_id]

                vel = vel + acc * self.dt
                vel = vel * max(0.0, 1.0 - self.linear_drag * self.dt)

                speed = float(np.linalg.norm(vel))
                if speed > self.max_speed:
                    vel = vel * (self.max_speed / speed)

                pos = pos + vel * self.dt
                collision = self._check_collision(pos)

                if collision:
                    pos[0] = float(np.clip(pos[0], -self.world_half_extent, self.world_half_extent))
                    pos[1] = float(np.clip(pos[1], -self.world_half_extent, self.world_half_extent))
                    vel = np.zeros(2, dtype=float)

                self.positions[agent_id] = pos
                self.velocities[agent_id] = vel

                odom = Odometry()
                odom.header.frame_id = "world"
                odom.child_frame_id = f"agent_{agent_id}"
                odom.pose.pose.position.x = float(pos[0])
                odom.pose.pose.position.y = float(pos[1])
                odom.pose.pose.position.z = 0.6
                yaw = math.atan2(vel[1], vel[0]) if float(np.linalg.norm(vel)) > 1e-6 else 0.0
                odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
                odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
                odom.twist.twist.linear.x = float(vel[0])
                odom.twist.twist.linear.y = float(vel[1])
                self.odom_pubs[agent_id].publish(odom)

                collision_msg = Bool()
                collision_msg.data = collision
                self.collision_pubs[agent_id].publish(collision_msg)

            if self.tick_count % 200 == 0:
                self.get_logger().info("Mock env tick running, odom stream is active")
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Mock env step error: {exc}")

    def _check_collision(self, pos: np.ndarray) -> bool:
        x = float(pos[0])
        y = float(pos[1])

        if abs(x) >= self.world_half_extent or abs(y) >= self.world_half_extent:
            return True

        for ox, oy, sx, sy in self.obstacles:
            if abs(x - ox) <= sx / 2.0 and abs(y - oy) <= sy / 2.0:
                return True

        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockEnvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
