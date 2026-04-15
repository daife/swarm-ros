import math
import random
from typing import List, Optional, Tuple

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from rclpy.node import Node
from std_msgs.msg import Int8
from visualization_msgs.msg import MarkerArray


class AgentManagerNode(Node):
	def __init__(self) -> None:
		super().__init__("agent_manager_node")
		self.declare_parameter("num_pursuers", 5)
		self.declare_parameter("num_evaders", 1)
		self.declare_parameter("safe_spawn_distance", 1.5)
		self.declare_parameter("spawn_half_extent", 9.0)
		self.declare_parameter("spawn_attempts", 300)
		self.declare_parameter("world_name", "urban")
		self.declare_parameter("pursuer_name_prefix", "pursuer")
		self.declare_parameter("evader_name_prefix", "evader")

		self.obstacles: List[Tuple[float, float, float, float]] = []
		self.initial_poses: List[Pose] = []
		self.last_reset_poses: List[Pose] = []

		self.create_subscription(MarkerArray, "/world_manager/obstacles", self.on_obstacles, 10)
		self.create_subscription(Int8, "/scheduler/reset_agents", self.on_reset_agents, 10)
		self.reset_pose_debug_pub = self.create_publisher(PoseArray, "/agent_manager/reset_poses", 10)

		world_name = self.get_parameter("world_name").get_parameter_value().string_value
		service_name = f"/world/{world_name}/set_pose"
		self.set_pose_client = self.create_client(SetEntityPose, service_name)

		self.num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
		self.num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value
		self.agent_model_names = self._build_agent_model_names()

	def _build_agent_model_names(self) -> List[str]:
		pursuer_prefix = self.get_parameter("pursuer_name_prefix").get_parameter_value().string_value
		evader_prefix = self.get_parameter("evader_name_prefix").get_parameter_value().string_value
		names: List[str] = []
		for i in range(self.num_pursuers):
			names.append(f"{pursuer_prefix}_{i}")
		for i in range(self.num_evaders):
			names.append(f"{evader_prefix}_{i}")
		return names

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
		self.get_logger().info(f"Cached {len(self.obstacles)} obstacles for safe spawn checks")
		if not self.initial_poses:
			self.initial_poses = self._sample_pose_set()

	def _sample_safe_pose(self) -> Optional[Pose]:
		safe_spawn_distance = self.get_parameter("safe_spawn_distance").get_parameter_value().double_value
		half_extent = self.get_parameter("spawn_half_extent").get_parameter_value().double_value
		attempts = self.get_parameter("spawn_attempts").get_parameter_value().integer_value

		for _ in range(attempts):
			x = random.uniform(-half_extent, half_extent)
			y = random.uniform(-half_extent, half_extent)
			yaw = random.uniform(-math.pi, math.pi)

			if self._point_in_forbidden_zone(x, y, safe_spawn_distance):
				continue

			pose = Pose()
			pose.position.x = x
			pose.position.y = y
			pose.position.z = 0.6
			pose.orientation.z = math.sin(yaw / 2.0)
			pose.orientation.w = math.cos(yaw / 2.0)
			return pose

		return None

	def _sample_pose_set(self) -> List[Pose]:
		all_poses: List[Pose] = []
		total = self.num_pursuers + self.num_evaders
		for _ in range(total):
			pose = self._sample_safe_pose()
			if pose is None:
				self.get_logger().warning("Failed to sample a safe pose; using origin fallback")
				pose = Pose()
				pose.position.z = 0.6
				pose.orientation.w = 1.0
			all_poses.append(pose)
		return all_poses

	def _point_in_forbidden_zone(self, x: float, y: float, safe_margin: float) -> bool:
		for ox, oy, sx, sy in self.obstacles:
			dx = abs(x - ox)
			dy = abs(y - oy)
			if dx <= (sx / 2.0 + safe_margin) and dy <= (sy / 2.0 + safe_margin):
				return True
		return False

	def _call_set_pose(self, model_name: str, pose: Pose) -> None:
		if not self.set_pose_client.wait_for_service(timeout_sec=0.2):
			self.get_logger().warning("SetEntityPose service not available; keep running in dry mode")
			return

		req = SetEntityPose.Request()
		req.entity = Entity()
		req.entity.name = model_name
		req.entity.type = Entity.MODEL
		req.pose = pose

		future = self.set_pose_client.call_async(req)

		def _on_done(done_future) -> None:
			try:
				resp = done_future.result()
				if not resp.success:
					self.get_logger().warning(f"SetEntityPose failed for {model_name}")
			except Exception as exc:  # pragma: no cover
				self.get_logger().warning(f"SetEntityPose exception for {model_name}: {exc}")

		future.add_done_callback(_on_done)

	def _apply_pose_set(self, poses: List[Pose]) -> None:
		for model_name, pose in zip(self.agent_model_names, poses):
			self._call_set_pose(model_name, pose)
		debug_msg = PoseArray()
		debug_msg.header.frame_id = "world"
		debug_msg.poses = poses
		self.reset_pose_debug_pub.publish(debug_msg)
		self.last_reset_poses = poses

	def on_reset_agents(self, msg: Int8) -> None:
		mode = msg.data
		if not self.initial_poses:
			self.initial_poses = self._sample_pose_set()

		if mode == 1:
			self.get_logger().info("Reset mode=1: restore initial poses")
			self._apply_pose_set(self.initial_poses)
		elif mode == 2:
			self.get_logger().info("Reset mode=2: sample safe random poses")
			new_poses = self._sample_pose_set()
			self._apply_pose_set(new_poses)
		else:
			self.get_logger().warning(f"Unsupported reset mode: {mode}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgentManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
