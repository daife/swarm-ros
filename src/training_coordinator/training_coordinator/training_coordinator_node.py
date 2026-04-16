import math
from functools import partial
from typing import Dict, Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int8


class TrainingCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("training_coordinator_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("max_episodes", 4000)
        self.declare_parameter("max_steps", 500)
        self.declare_parameter("reset_policy", 1)
        self.declare_parameter("collision_penalty", -2.0)
        self.declare_parameter("capture_radius", 2.0)
        self.declare_parameter("capture_steps", 5)
        self.declare_parameter("initial_reset_delay_sec", 5.0)
        self.declare_parameter("reset_agents_delay_sec", 0.4)
        self.declare_parameter("reset_grace_steps", 12)
        self.declare_parameter("always_reset_world", True)

        self.num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        self.num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value

        self.reset_world_pub = self.create_publisher(Empty, "/scheduler/reset_world", 10)
        self.reset_agents_pub = self.create_publisher(Int8, "/scheduler/reset_agents", 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, "/rl/episode_metrics", 10)

        self.current_episode = 0
        self.current_step = 0
        self.capture_streak = 0
        self.collision_events = 0
        self.collision_penalty_total = 0.0
        self.odom_states: Dict[int, Odometry] = {}
        self.startup_reset_done = False
        self.reset_grace_remaining = 0
        self.pending_agent_reset_mode: Optional[int] = None
        self.pending_agent_reset_reason = ""
        self.pending_agent_reset_time = 0.0

        delay_sec = self.get_parameter("initial_reset_delay_sec").get_parameter_value().double_value
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.startup_reset_deadline = now_sec + max(0.0, delay_sec)

        total_agents = self.num_pursuers + self.num_evaders
        for agent_id in range(total_agents):
            self.create_subscription(Odometry, f"/agent_{agent_id}/odom", partial(self.on_odom, agent_id), 10)
            self.create_subscription(Bool, f"/agent_{agent_id}/collision", partial(self.on_collision, agent_id), 10)

        self.create_timer(0.05, self.tick)

    def on_odom(self, agent_id: int, msg: Odometry) -> None:
        self.odom_states[agent_id] = msg

    def on_collision(self, _agent_id: int, msg: Bool) -> None:
        if not self.startup_reset_done:
            return
        if self.reset_grace_remaining > 0:
            return
        if msg.data:
            self.collision_events += 1
            penalty = self.get_parameter("collision_penalty").get_parameter_value().double_value
            self.collision_penalty_total += penalty

    def _agent_reset_mode(self, reset_policy: int) -> int:
        return 1 if reset_policy == 1 else 2

    def _publish_agent_reset(self, mode: int, reason: str) -> None:
        msg = Int8()
        msg.data = mode
        self.reset_agents_pub.publish(msg)
        self.get_logger().info(f"Published /scheduler/reset_agents with mode={mode} ({reason})")

    def _schedule_agent_reset(self, mode: int, reason: str) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        delay = self.get_parameter("reset_agents_delay_sec").get_parameter_value().double_value
        if delay <= 0.0:
            self._publish_agent_reset(mode, reason)
            return

        self.pending_agent_reset_mode = mode
        self.pending_agent_reset_reason = reason
        self.pending_agent_reset_time = now_sec + delay

    def _flush_pending_agent_reset(self) -> None:
        if self.pending_agent_reset_mode is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec < self.pending_agent_reset_time:
            return

        self._publish_agent_reset(self.pending_agent_reset_mode, self.pending_agent_reset_reason)
        self.pending_agent_reset_mode = None
        self.pending_agent_reset_reason = ""
        self.pending_agent_reset_time = 0.0

    def _trigger_reset(self, reset_policy: int, reason: str, startup: bool = False) -> None:
        always_reset_world = self.get_parameter("always_reset_world").get_parameter_value().bool_value
        publish_world_reset = startup or always_reset_world or reset_policy == 3

        if publish_world_reset:
            self.reset_world_pub.publish(Empty())
            self.get_logger().info(f"Published /scheduler/reset_world ({reason})")

        mode = self._agent_reset_mode(reset_policy)
        self._schedule_agent_reset(mode, reason)

        self.reset_grace_remaining = max(
            0,
            self.get_parameter("reset_grace_steps").get_parameter_value().integer_value,
        )

    def _is_capture(self) -> bool:
        capture_radius = self.get_parameter("capture_radius").get_parameter_value().double_value
        if self.num_evaders <= 0:
            return False

        for evader_offset in range(self.num_evaders):
            evader_id = self.num_pursuers + evader_offset
            evader = self.odom_states.get(evader_id)
            if evader is None:
                continue
            ex = evader.pose.pose.position.x
            ey = evader.pose.pose.position.y

            for pursuer_id in range(self.num_pursuers):
                pursuer = self.odom_states.get(pursuer_id)
                if pursuer is None:
                    continue
                px = pursuer.pose.pose.position.x
                py = pursuer.pose.pose.position.y
                if math.hypot(ex - px, ey - py) <= capture_radius:
                    return True
        return False

    def _publish_metrics(self, done_reason: int) -> None:
        # [episode, steps, done_reason, capture_streak, collision_events, collision_penalty_total]
        msg = Float32MultiArray()
        msg.data = [
            float(self.current_episode),
            float(self.current_step),
            float(done_reason),
            float(self.capture_streak),
            float(self.collision_events),
            float(self.collision_penalty_total),
        ]
        self.metrics_pub.publish(msg)

    def tick(self) -> None:
        max_episodes = self.get_parameter("max_episodes").get_parameter_value().integer_value
        max_steps = self.get_parameter("max_steps").get_parameter_value().integer_value
        reset_policy = self.get_parameter("reset_policy").get_parameter_value().integer_value
        capture_steps = self.get_parameter("capture_steps").get_parameter_value().integer_value

        if self.current_episode >= max_episodes:
            return

        self._flush_pending_agent_reset()

        if not self.startup_reset_done:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec < self.startup_reset_deadline:
                return
            self._trigger_reset(reset_policy, reason="startup", startup=True)
            self.startup_reset_done = True
            self.get_logger().info("Startup reset done, waiting reset grace before Episode 1")
            return

        if self.reset_grace_remaining > 0:
            self.reset_grace_remaining -= 1
            return

        self.current_step += 1
        if self._is_capture():
            self.capture_streak += 1
        else:
            self.capture_streak = 0

        done_by_capture = self.capture_streak >= capture_steps
        done_by_collision = self.collision_events > 0
        done_by_step = self.current_step >= max_steps

        if not (done_by_capture or done_by_collision or done_by_step):
            return

        done_reason = 1 if done_by_capture else (2 if done_by_collision else 3)
        self._publish_metrics(done_reason)

        self.current_episode += 1
        self.get_logger().info(
            f"Episode {self.current_episode} done reason={done_reason}, apply reset policy {reset_policy}"
        )

        self._trigger_reset(reset_policy, reason=f"episode_{self.current_episode}_done")

        self.current_step = 0
        self.capture_streak = 0
        self.collision_events = 0
        self.collision_penalty_total = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrainingCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
