import importlib.util
import math
import os
from functools import partial
from typing import Dict

import rclpy
from geometry_msgs.msg import Accel
from nav_msgs.msg import Odometry
from rclpy.node import Node


class RLPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_policy_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("pursuer_algo", "cbc_tp_net")
        self.declare_parameter("pursuer_train_mode", "train_scratch")
        self.declare_parameter("pursuer_weights", "")
        self.declare_parameter("evader_algo", "ddpg")
        self.declare_parameter("evader_train_mode", "eval_only")
        self.declare_parameter("evader_weights", "")
        self.declare_parameter("max_accel", 1.0)

        pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value

        self.states: Dict[str, Odometry] = {}
        self.cmd_publishers = {}

        total_agents = pursuers + evaders
        for i in range(total_agents):
            odom_topic = f"/agent_{i}/odom"
            cmd_topic = f"/agent_{i}/cmd_accel"
            self.create_subscription(Odometry, odom_topic, partial(self.on_odom, i), 10)
            self.cmd_publishers[i] = self.create_publisher(Accel, cmd_topic, 10)

        self.num_pursuers = pursuers
        self.num_evaders = evaders

        self._validate_modes_and_weights()

        self.create_timer(0.1, self.control_tick)

    def _validate_modes_and_weights(self) -> None:
        self._validate_single_role("pursuer")
        self._validate_single_role("evader")

    def _validate_single_role(self, role: str) -> None:
        mode = self.get_parameter(f"{role}_train_mode").get_parameter_value().string_value
        weights = self.get_parameter(f"{role}_weights").get_parameter_value().string_value
        valid_modes = {"train_scratch", "train_load", "eval_only"}
        if mode not in valid_modes:
            self.get_logger().warning(f"Unknown {role}_train_mode={mode}; fallback to train_scratch")
            return

        if mode in {"train_load", "eval_only"}:
            if not weights:
                self.get_logger().warning(f"{role}_train_mode={mode} but no weights path provided")
                return
            if not os.path.exists(weights):
                self.get_logger().warning(f"{role} weights not found: {weights}")
                return
            if not importlib.util.find_spec("torch"):
                self.get_logger().warning("PyTorch not installed; skip loading weights")
                return

            self.get_logger().info(f"{role} weights ready for load from {weights}")

    def on_odom(self, agent_id: int, msg: Odometry) -> None:
        self.states[str(agent_id)] = msg

    def control_tick(self) -> None:
        max_accel = self.get_parameter("max_accel").get_parameter_value().double_value

        pursuer_ids = list(range(self.num_pursuers))
        evader_ids = list(range(self.num_pursuers, self.num_pursuers + self.num_evaders))
        if not evader_ids:
            return

        target_evader_id = evader_ids[0]
        target_evader_state = self.states.get(str(target_evader_id))

        for pursuer_id in pursuer_ids:
            cmd = Accel()
            pursuer_state = self.states.get(str(pursuer_id))
            if pursuer_state is not None and target_evader_state is not None:
                dx = (
                    target_evader_state.pose.pose.position.x
                    - pursuer_state.pose.pose.position.x
                )
                dy = (
                    target_evader_state.pose.pose.position.y
                    - pursuer_state.pose.pose.position.y
                )
                norm = math.hypot(dx, dy)
                if norm > 1e-6:
                    cmd.linear.x = max_accel * dx / norm
                    cmd.linear.y = max_accel * dy / norm
            self.cmd_publishers[pursuer_id].publish(cmd)

        for evader_id in evader_ids:
            cmd = Accel()
            evader_state = self.states.get(str(evader_id))
            if evader_state is not None:
                nearest = None
                nearest_dist = None
                ex = evader_state.pose.pose.position.x
                ey = evader_state.pose.pose.position.y
                for pursuer_id in pursuer_ids:
                    pursuer_state = self.states.get(str(pursuer_id))
                    if pursuer_state is None:
                        continue
                    px = pursuer_state.pose.pose.position.x
                    py = pursuer_state.pose.pose.position.y
                    dist = math.hypot(ex - px, ey - py)
                    if nearest_dist is None or dist < nearest_dist:
                        nearest_dist = dist
                        nearest = (px, py)
                if nearest is not None:
                    dx = ex - nearest[0]
                    dy = ey - nearest[1]
                    norm = math.hypot(dx, dy)
                    if norm > 1e-6:
                        cmd.linear.x = max_accel * dx / norm
                        cmd.linear.y = max_accel * dy / norm
            self.cmd_publishers[evader_id].publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
