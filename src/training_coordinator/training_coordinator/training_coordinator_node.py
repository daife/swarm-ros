import math
from functools import partial
from typing import Dict

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

        total_agents = self.num_pursuers + self.num_evaders
        for agent_id in range(total_agents):
            self.create_subscription(Odometry, f"/agent_{agent_id}/odom", partial(self.on_odom, agent_id), 10)
            self.create_subscription(Bool, f"/agent_{agent_id}/collision", partial(self.on_collision, agent_id), 10)

        self.create_timer(0.05, self.tick)

    def on_odom(self, agent_id: int, msg: Odometry) -> None:
        self.odom_states[agent_id] = msg

    def on_collision(self, _agent_id: int, msg: Bool) -> None:
        if msg.data:
            self.collision_events += 1
            penalty = self.get_parameter("collision_penalty").get_parameter_value().double_value
            self.collision_penalty_total += penalty

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

        if reset_policy == 3:
            self.reset_world_pub.publish(Empty())
            self.get_logger().info("Published /scheduler/reset_world")

        mode = 1 if reset_policy == 1 else 2
        msg = Int8()
        msg.data = mode
        self.reset_agents_pub.publish(msg)
        self.get_logger().info(f"Published /scheduler/reset_agents with mode={mode}")

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
