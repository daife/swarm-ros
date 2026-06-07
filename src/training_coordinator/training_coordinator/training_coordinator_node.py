import csv
import math
import os
from collections import deque
from functools import partial
from typing import Deque, Dict

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32, Int32MultiArray


class TrainingCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("training_coordinator_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("max_episodes", 4000)
        self.declare_parameter("max_steps", 500)
        self.declare_parameter("reset_policy", 1)
        self.declare_parameter("collision_penalty", -10.0)
        self.declare_parameter("capture_radius", 2.0)
        self.declare_parameter("capture_steps", 5)
        self.declare_parameter("capture_required_pursuers", 3)
        self.declare_parameter("initial_reset_delay_sec", 5.0)
        self.declare_parameter("always_reset_world", False)
        self.declare_parameter("metrics_output_dir", "artifacts/metrics")
        self.declare_parameter("metrics_window_episodes", 100)

        self.num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        self.num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value

        self.reset_world_pub = self.create_publisher(Int32, "/scheduler/reset_world", 10)
        self.reset_begin_pub = self.create_publisher(Int32, "/scheduler/reset_begin", 10)
        self.reset_agents_cmd_pub = self.create_publisher(Int32MultiArray, "/scheduler/reset_agents_cmd", 10)
        self.episode_start_pub = self.create_publisher(Int32, "/scheduler/episode_start", 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, "/rl/episode_metrics", 10)

        self.current_episode = 0
        self.current_step = 0
        self.capture_streak = 0
        self.collision_events = 0
        self.collision_penalty_total = 0.0
        self.odom_states: Dict[int, Odometry] = {}
        self.startup_reset_issued = False
        self.episode_records: Deque[Dict[str, float]] = deque(
            maxlen=max(1, self.get_parameter("metrics_window_episodes").get_parameter_value().integer_value)
        )
        self.metrics_dir = self._resolve_metrics_dir()
        self.episode_metrics_path = os.path.join(self.metrics_dir, "episode_metrics.csv")
        self.window_metrics_path = os.path.join(self.metrics_dir, "paper_window_metrics.csv")
        self._init_metrics_files()

        self.active_reset_id = 0
        self.awaiting_reset = False
        self.world_ready = False
        self.agents_ready = False
        self.policy_ready = False
        self.agent_reset_sent = False
        self.pending_agent_reset_mode = 0

        delay_sec = self.get_parameter("initial_reset_delay_sec").get_parameter_value().double_value
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.startup_reset_deadline = now_sec + max(0.0, delay_sec)

        total_agents = self.num_pursuers + self.num_evaders
        for agent_id in range(total_agents):
            self.create_subscription(Odometry, f"/agent_{agent_id}/odom", partial(self.on_odom, agent_id), 10)
            self.create_subscription(Bool, f"/agent_{agent_id}/collision", partial(self.on_collision, agent_id), 10)

        self.create_subscription(Int32, "/world_manager/reset_done", self.on_world_reset_done, 10)
        self.create_subscription(Int32, "/agent_manager/reset_done", self.on_agent_reset_done, 10)
        self.create_subscription(Int32, "/rl_policy/reset_ready", self.on_policy_reset_ready, 10)

        self.create_timer(0.05, self.tick)

    def on_odom(self, agent_id: int, msg: Odometry) -> None:
        self.odom_states[agent_id] = msg

    def on_collision(self, _agent_id: int, msg: Bool) -> None:
        if not self.startup_reset_issued:
            return
        if self.awaiting_reset:
            return
        if msg.data:
            self.collision_events += 1
            penalty = self.get_parameter("collision_penalty").get_parameter_value().double_value
            self.collision_penalty_total += penalty

    def _is_current_reset(self, reset_id: int) -> bool:
        return self.awaiting_reset and reset_id == self.active_reset_id

    def on_world_reset_done(self, msg: Int32) -> None:
        reset_id = int(msg.data)
        if not self._is_current_reset(reset_id):
            return
        if not self.world_ready:
            self.world_ready = True
            self.get_logger().info(f"World ready for reset_id={reset_id}")
        self._publish_agent_reset_if_ready(reset_id, "world_ready")

    def on_agent_reset_done(self, msg: Int32) -> None:
        reset_id = int(msg.data)
        if not self._is_current_reset(reset_id):
            return
        if not self.agents_ready:
            self.agents_ready = True
            self.get_logger().info(f"Agents ready for reset_id={reset_id}")

    def on_policy_reset_ready(self, msg: Int32) -> None:
        reset_id = int(msg.data)
        if not self._is_current_reset(reset_id):
            return
        if not self.policy_ready:
            self.policy_ready = True
            self.get_logger().info(f"Policy ready for reset_id={reset_id}")

    def _agent_reset_mode(self, reset_policy: int) -> int:
        return 1 if reset_policy == 1 else 2

    def _publish_reset_begin(self, reset_id: int, reason: str) -> None:
        msg = Int32()
        msg.data = reset_id
        self.reset_begin_pub.publish(msg)
        self.get_logger().info(f"Published /scheduler/reset_begin reset_id={reset_id} ({reason})")

    def _publish_agent_reset_if_ready(self, reset_id: int, reason: str) -> None:
        if not self._is_current_reset(reset_id):
            return
        if self.agent_reset_sent:
            return
        if not self.world_ready:
            return

        agent_msg = Int32MultiArray()
        agent_msg.data = [int(reset_id), int(self.pending_agent_reset_mode)]
        self.reset_agents_cmd_pub.publish(agent_msg)
        self.agent_reset_sent = True
        self.get_logger().info(
            "Published /scheduler/reset_agents_cmd "
            f"reset_id={reset_id}, mode={self.pending_agent_reset_mode} ({reason})"
        )

    def _publish_episode_start(self, reset_id: int) -> None:
        msg = Int32()
        msg.data = reset_id
        self.episode_start_pub.publish(msg)
        self.get_logger().info(f"Published /scheduler/episode_start reset_id={reset_id}")

    def _reset_handshake_ready(self) -> bool:
        return self.world_ready and self.agent_reset_sent and self.agents_ready and self.policy_ready

    def _trigger_reset(self, reset_policy: int, reason: str, startup: bool = False) -> None:
        always_reset_world = self.get_parameter("always_reset_world").get_parameter_value().bool_value
        publish_world_reset = startup or always_reset_world or reset_policy == 3

        self.active_reset_id += 1
        reset_id = self.active_reset_id
        self.awaiting_reset = True
        self.agent_reset_sent = False
        self.pending_agent_reset_mode = self._agent_reset_mode(reset_policy)
        self.world_ready = not publish_world_reset
        self.agents_ready = False
        self.policy_ready = False

        self._publish_reset_begin(reset_id, reason)

        if publish_world_reset:
            msg = Int32()
            msg.data = reset_id
            self.reset_world_pub.publish(msg)
            self.get_logger().info(f"Published /scheduler/reset_world reset_id={reset_id} ({reason})")

        self._publish_agent_reset_if_ready(reset_id, reason)

    def _resolve_metrics_dir(self) -> str:
        raw_dir = self.get_parameter("metrics_output_dir").get_parameter_value().string_value
        out_dir = raw_dir if os.path.isabs(raw_dir) else os.path.join(os.getcwd(), raw_dir)
        os.makedirs(out_dir, exist_ok=True)
        return out_dir

    def _init_metrics_files(self) -> None:
        if not os.path.exists(self.episode_metrics_path):
            with open(self.episode_metrics_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "episode",
                        "steps",
                        "done_reason",
                        "success",
                        "capture_streak",
                        "capturing_pursuers",
                        "collision_events",
                        "collision_penalty_total",
                    ]
                )

        if not os.path.exists(self.window_metrics_path):
            with open(self.window_metrics_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "episode",
                        "window_size",
                        "mission_success_rate",
                        "mean_episode_steps",
                        "mean_success_capture_steps",
                        "collision_rate",
                        "timeout_rate",
                    ]
                )

    def _capturing_pursuer_count(self) -> int:
        capture_radius = self.get_parameter("capture_radius").get_parameter_value().double_value
        if self.num_evaders <= 0:
            return 0

        max_count = 0
        for evader_offset in range(self.num_evaders):
            evader_id = self.num_pursuers + evader_offset
            evader = self.odom_states.get(evader_id)
            if evader is None:
                continue
            ex = evader.pose.pose.position.x
            ey = evader.pose.pose.position.y

            count = 0
            for pursuer_id in range(self.num_pursuers):
                pursuer = self.odom_states.get(pursuer_id)
                if pursuer is None:
                    continue
                px = pursuer.pose.pose.position.x
                py = pursuer.pose.pose.position.y
                if math.hypot(ex - px, ey - py) <= capture_radius:
                    count += 1
            max_count = max(max_count, count)
        return max_count

    def _is_capture(self) -> bool:
        required = self.get_parameter("capture_required_pursuers").get_parameter_value().integer_value
        return self._capturing_pursuer_count() >= max(1, required)

    def _save_metrics(self, done_reason: int, capturing_pursuers: int) -> None:
        episode = self.current_episode + 1
        success = 1 if done_reason == 1 else 0
        record = {
            "episode": float(episode),
            "steps": float(self.current_step),
            "done_reason": float(done_reason),
            "success": float(success),
            "collision": 1.0 if done_reason == 2 else 0.0,
            "timeout": 1.0 if done_reason == 3 else 0.0,
        }
        self.episode_records.append(record)

        with open(self.episode_metrics_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    episode,
                    self.current_step,
                    done_reason,
                    success,
                    self.capture_streak,
                    capturing_pursuers,
                    self.collision_events,
                    self.collision_penalty_total,
                ]
            )
        self.get_logger().info(
            "[paper_metrics] episode=%d success=%d steps=%d done_reason=%d "
            "capture_streak=%d capturing_pursuers=%d collisions=%d"
            % (
                episode,
                success,
                self.current_step,
                done_reason,
                self.capture_streak,
                capturing_pursuers,
                self.collision_events,
            )
        )

        window = list(self.episode_records)
        if not window:
            return
        window_size = len(window)
        success_records = [r for r in window if r["success"] > 0.5]
        mission_success_rate = sum(r["success"] for r in window) / float(window_size)
        mean_episode_steps = sum(r["steps"] for r in window) / float(window_size)
        mean_success_capture_steps = (
            sum(r["steps"] for r in success_records) / float(len(success_records))
            if success_records
            else 0.0
        )
        collision_rate = sum(r["collision"] for r in window) / float(window_size)
        timeout_rate = sum(r["timeout"] for r in window) / float(window_size)

        metrics_window_episodes = self.get_parameter("metrics_window_episodes").get_parameter_value().integer_value
        if episode % max(1, metrics_window_episodes) == 0:
            with open(self.window_metrics_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        episode,
                        window_size,
                        mission_success_rate,
                        mean_episode_steps,
                        mean_success_capture_steps,
                        collision_rate,
                        timeout_rate,
                    ]
                )
            self.get_logger().info(
                "[paper_window] episode=%d window=%d mission_success_rate=%.3f "
                "mean_episode_steps=%.2f mean_success_capture_steps=%.2f "
                "collision_rate=%.3f timeout_rate=%.3f"
                % (
                    episode,
                    window_size,
                    mission_success_rate,
                    mean_episode_steps,
                    mean_success_capture_steps,
                    collision_rate,
                    timeout_rate,
                )
            )

    def _publish_metrics(self, done_reason: int, capturing_pursuers: int) -> None:
        # Backward-compatible prefix:
        # [episode, steps, done_reason, capture_streak, collision_events, collision_penalty_total, ...]
        msg = Float32MultiArray()
        msg.data = [
            float(self.current_episode),
            float(self.current_step),
            float(done_reason),
            float(self.capture_streak),
            float(self.collision_events),
            float(self.collision_penalty_total),
            float(capturing_pursuers),
            float(1 if done_reason == 1 else 0),
        ]
        self.metrics_pub.publish(msg)

    def tick(self) -> None:
        max_episodes = self.get_parameter("max_episodes").get_parameter_value().integer_value
        max_steps = self.get_parameter("max_steps").get_parameter_value().integer_value
        reset_policy = self.get_parameter("reset_policy").get_parameter_value().integer_value
        capture_steps = self.get_parameter("capture_steps").get_parameter_value().integer_value

        if self.current_episode >= max_episodes:
            return

        if not self.startup_reset_issued:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec < self.startup_reset_deadline:
                return
            self._trigger_reset(reset_policy, reason="startup", startup=True)
            self.startup_reset_issued = True
            self.get_logger().info("Startup reset requested, waiting strong handshake")
            return

        if self.awaiting_reset:
            if not self._reset_handshake_ready():
                return
            done_reset_id = self.active_reset_id
            self.awaiting_reset = False
            self._publish_episode_start(done_reset_id)
            self.get_logger().info(
                f"Reset handshake complete for reset_id={done_reset_id}, episode {self.current_episode + 1} starts"
            )
            return

        self.current_step += 1
        capturing_pursuers = self._capturing_pursuer_count()
        required_pursuers = self.get_parameter("capture_required_pursuers").get_parameter_value().integer_value
        if capturing_pursuers >= max(1, required_pursuers):
            self.capture_streak += 1
        else:
            self.capture_streak = 0

        done_by_capture = self.capture_streak >= capture_steps
        done_by_collision = self.collision_events > 0
        done_by_step = self.current_step >= max_steps

        if not (done_by_capture or done_by_collision or done_by_step):
            return

        done_reason = 1 if done_by_capture else (2 if done_by_collision else 3)
        self._publish_metrics(done_reason, capturing_pursuers)
        self._save_metrics(done_reason, capturing_pursuers)

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
