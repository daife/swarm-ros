import math
from functools import partial
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Accel, Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool


class SimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_bridge_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("pursuer_name_prefix", "pursuer")
        self.declare_parameter("evader_name_prefix", "evader")
        self.declare_parameter("world_name", "urban")
        self.declare_parameter("controller_enabled", True)
        self.declare_parameter("controller_dt", 0.05)
        self.declare_parameter("max_speed", 3.0)
        self.declare_parameter("pursuer_max_speed", -1.0)
        self.declare_parameter("evader_max_speed", -1.0)
        self.declare_parameter("linear_drag", 0.15)
        self.declare_parameter("world_half_extent", 10.0)
        self.declare_parameter("fixed_z", 0.7)
        self.declare_parameter("control_requires_reset", True)

        num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value
        pursuer_prefix = self.get_parameter("pursuer_name_prefix").get_parameter_value().string_value
        evader_prefix = self.get_parameter("evader_name_prefix").get_parameter_value().string_value
        self.world_name = self.get_parameter("world_name").get_parameter_value().string_value
        self.num_pursuers = num_pursuers

        self.controller_enabled = (
            self.get_parameter("controller_enabled").get_parameter_value().bool_value
        )
        self.controller_dt = self.get_parameter("controller_dt").get_parameter_value().double_value
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        self.pursuer_max_speed = (
            self.get_parameter("pursuer_max_speed").get_parameter_value().double_value
        )
        self.evader_max_speed = (
            self.get_parameter("evader_max_speed").get_parameter_value().double_value
        )
        self.linear_drag = self.get_parameter("linear_drag").get_parameter_value().double_value
        self.world_half_extent = self.get_parameter("world_half_extent").get_parameter_value().double_value
        self.fixed_z = self.get_parameter("fixed_z").get_parameter_value().double_value
        self.control_requires_reset = (
            self.get_parameter("control_requires_reset").get_parameter_value().bool_value
        )

        self.agent_names: List[Tuple[int, str]] = []
        for i in range(num_pursuers):
            self.agent_names.append((i, f"{pursuer_prefix}_{i}"))
        for i in range(num_evaders):
            self.agent_names.append((num_pursuers + i, f"{evader_prefix}_{i}"))

        self.odom_pubs: Dict[int, object] = {}
        self.collision_pubs: Dict[int, object] = {}
        self.accels: Dict[int, np.ndarray] = {}
        self.positions: Dict[int, np.ndarray] = {}
        self.velocities: Dict[int, np.ndarray] = {}
        self.model_ready: Dict[int, bool] = {}
        self.has_received_reset = False

        set_pose_service = f"/world/{self.world_name}/set_pose"
        self.set_pose_client = self.create_client(SetEntityPose, set_pose_service)

        for agent_id, model_name in self.agent_names:
            model_odom_topic = f"/model/{model_name}/odometry"
            default_contacts_topic = f"/model/{model_name}/contacts"
            model_contacts_topic = (
                f"/world/{self.world_name}/model/{model_name}/link/base_link/sensor/contact_sensor/contact"
            )
            odom_topic = f"/agent_{agent_id}/odom"
            collision_topic = f"/agent_{agent_id}/collision"
            cmd_topic = f"/agent_{agent_id}/cmd_accel"

            self.odom_pubs[agent_id] = self.create_publisher(Odometry, odom_topic, 10)
            self.collision_pubs[agent_id] = self.create_publisher(Bool, collision_topic, 10)
            self.accels[agent_id] = np.zeros(2, dtype=float)
            self.positions[agent_id] = np.zeros(2, dtype=float)
            self.velocities[agent_id] = np.zeros(2, dtype=float)
            self.model_ready[agent_id] = False

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
            self.create_subscription(
                Contacts,
                default_contacts_topic,
                partial(self.on_model_contacts, agent_id),
                10,
            )
            self.create_subscription(
                Accel,
                cmd_topic,
                partial(self.on_cmd_accel, agent_id),
                10,
            )

        self.create_subscription(PoseArray, "/agent_manager/reset_poses", self.on_reset_poses, 10)

        if self.controller_enabled:
            self.create_timer(self.controller_dt, self.control_step)

        self.get_logger().info(
            f"Sim bridge online for {len(self.agent_names)} agents; odom+contacts forwarding enabled"
        )

    def on_model_odom(self, agent_id: int, msg: Odometry) -> None:
        self.model_ready[agent_id] = True
        self.positions[agent_id][0] = msg.pose.pose.position.x
        self.positions[agent_id][1] = msg.pose.pose.position.y
        self.velocities[agent_id][0] = msg.twist.twist.linear.x
        self.velocities[agent_id][1] = msg.twist.twist.linear.y

        msg.pose.pose.position.z = self.fixed_z
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        self.odom_pubs[agent_id].publish(msg)

    def on_model_contacts(self, agent_id: int, msg: Contacts) -> None:
        out = Bool()
        out.data = len(msg.contacts) > 0
        self.collision_pubs[agent_id].publish(out)

    def on_cmd_accel(self, agent_id: int, msg: Accel) -> None:
        self.accels[agent_id][0] = msg.linear.x
        self.accels[agent_id][1] = msg.linear.y

    def on_reset_poses(self, msg: PoseArray) -> None:
        self.has_received_reset = True
        for idx, pose in enumerate(msg.poses):
            if idx not in self.positions:
                continue
            self.positions[idx][0] = pose.position.x
            self.positions[idx][1] = pose.position.y
            self.velocities[idx][:] = 0.0

    def _set_model_pose(self, model_name: str, pos: np.ndarray, vel: np.ndarray) -> None:
        if not self.set_pose_client.wait_for_service(timeout_sec=0.01):
            return

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = model_name
        req.entity.type = Entity.MODEL

        pose = Pose()
        pose.position.x = float(pos[0])
        pose.position.y = float(pos[1])
        pose.position.z = self.fixed_z
        yaw = math.atan2(float(vel[1]), float(vel[0])) if np.linalg.norm(vel) > 1e-6 else 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        req.pose = pose
        self.set_pose_client.call_async(req)

    def _speed_limit_for_agent(self, agent_id: int) -> float:
        if agent_id < self.num_pursuers:
            return self.pursuer_max_speed if self.pursuer_max_speed > 0.0 else self.max_speed
        return self.evader_max_speed if self.evader_max_speed > 0.0 else self.max_speed

    def control_step(self) -> None:
        if self.control_requires_reset and not self.has_received_reset:
            return

        if self.model_ready and not all(self.model_ready.values()):
            return

        for agent_id, model_name in self.agent_names:
            if not self.model_ready.get(agent_id, False):
                continue

            acc = self.accels[agent_id]
            vel = self.velocities[agent_id]
            pos = self.positions[agent_id]

            vel = vel + acc * self.controller_dt
            vel = vel * max(0.0, 1.0 - self.linear_drag * self.controller_dt)
            speed = float(np.linalg.norm(vel))
            speed_limit = self._speed_limit_for_agent(agent_id)
            if speed > speed_limit:
                vel = vel * (speed_limit / speed)

            pos = pos + vel * self.controller_dt
            pos[0] = float(np.clip(pos[0], -self.world_half_extent, self.world_half_extent))
            pos[1] = float(np.clip(pos[1], -self.world_half_extent, self.world_half_extent))

            self.positions[agent_id] = pos
            self.velocities[agent_id] = vel
            self._set_model_pose(model_name, pos, vel)


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
