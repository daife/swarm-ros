import math
import random
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class AgentManagerNode(Node):
	def __init__(self) -> None:
		super().__init__("agent_manager_node")
		self.declare_parameter("num_pursuers", 5)
		self.declare_parameter("num_evaders", 1)
		self.declare_parameter("collision_distance", 0.8)
		self.declare_parameter("capture_radius", 2.0)
		self.declare_parameter("world_half_extent", 10.0)
		self.declare_parameter("spawn_half_extent", 9.0)
		self.declare_parameter("spawn_attempts", 300)
		self.declare_parameter("spawn_z", 0.7)
		self.declare_parameter("world_name", "urban")
		self.declare_parameter("pursuer_name_prefix", "pursuer")
		self.declare_parameter("evader_name_prefix", "evader")
		self.declare_parameter("agent_model_package", "swarm_bringup")
		self.declare_parameter("agent_staging_x", 1000.0)
		self.declare_parameter("agent_staging_y", 1200.0)
		self.declare_parameter("agent_staging_stride", 1.5)

		self.obstacles: List[Tuple[float, float, float, float, bool]] = []
		self.initial_poses: List[Pose] = []
		self.last_reset_poses: List[Pose] = []
		self.pending_reset_id: Optional[int] = None
		self.pending_reset_mode: Optional[int] = None
		self.spawned: Dict[str, bool] = {}
		self.spawn_pending: Dict[str, bool] = {}
		self.spawn_pending_since: Dict[str, float] = {}

		self.create_subscription(MarkerArray, "/world_manager/obstacles", self.on_obstacles, 10)
		self.create_subscription(Int32MultiArray, "/scheduler/reset_agents_cmd", self.on_reset_agents_cmd, 10)
		self.reset_pose_debug_pub = self.create_publisher(PoseArray, "/agent_manager/reset_poses", 10)
		self.reset_done_pub = self.create_publisher(Int32, "/agent_manager/reset_done", 10)

		self.world_name = self.get_parameter("world_name").get_parameter_value().string_value
		service_name = f"/world/{self.world_name}/set_pose"
		spawn_service_name = f"/world/{self.world_name}/create"
		self.set_pose_client = self.create_client(SetEntityPose, service_name)
		self.spawn_client = self.create_client(SpawnEntity, spawn_service_name)

		self.num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
		self.num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value
		self.total_agents = self.num_pursuers + self.num_evaders
		self.spawn_z = self.get_parameter("spawn_z").get_parameter_value().double_value

		self.agent_model_names = self._build_agent_model_names()
		self.agent_specs = self._build_agent_specs()
		self.create_timer(0.1, self._process_pending_work)

	def _build_agent_model_names(self) -> List[str]:
		pursuer_prefix = self.get_parameter("pursuer_name_prefix").get_parameter_value().string_value
		evader_prefix = self.get_parameter("evader_name_prefix").get_parameter_value().string_value

		names: List[str] = []
		for i in range(self.num_pursuers):
			names.append(f"{pursuer_prefix}_{i}")
		for i in range(self.num_evaders):
			names.append(f"{evader_prefix}_{i}")
		return names

	def _resolve_model_paths(self) -> Tuple[str, str]:
		model_pkg = self.get_parameter("agent_model_package").get_parameter_value().string_value
		share = get_package_share_directory(model_pkg)
		return (
			f"{share}/models/pursuer/model.sdf",
			f"{share}/models/evader/model.sdf",
		)

	def _build_agent_specs(self) -> List[Tuple[str, str]]:
		pursuer_sdf, evader_sdf = self._resolve_model_paths()
		specs: List[Tuple[str, str]] = []
		for i in range(self.num_pursuers):
			specs.append((self.agent_model_names[i], pursuer_sdf))
		for i in range(self.num_evaders):
			specs.append((self.agent_model_names[self.num_pursuers + i], evader_sdf))
		return specs

	def _collision_distance(self) -> float:
		return max(
			0.01,
			self.get_parameter("collision_distance").get_parameter_value().double_value,
		)

	def _is_pursuer(self, agent_id: int) -> bool:
		return agent_id < self.num_pursuers

	def on_obstacles(self, msg: MarkerArray) -> None:
		self.obstacles = []
		for marker in msg.markers:
			self.obstacles.append(
				(
					marker.pose.position.x,
					marker.pose.position.y,
					marker.scale.x,
					marker.scale.y,
					marker.type == Marker.CYLINDER,
				)
			)
		self.get_logger().info(f"Cached {len(self.obstacles)} obstacles for collision-distance spawn checks")

	def _staging_pose(self, idx: int) -> Pose:
		staging_x = self.get_parameter("agent_staging_x").get_parameter_value().double_value
		staging_y = self.get_parameter("agent_staging_y").get_parameter_value().double_value
		stride = self.get_parameter("agent_staging_stride").get_parameter_value().double_value

		row = idx // 12
		col = idx % 12
		pose = Pose()
		pose.position.x = staging_x + col * stride
		pose.position.y = staging_y + row * stride
		pose.position.z = self.spawn_z
		pose.orientation.w = 1.0
		return pose

	def _request_spawn_if_needed(self, model_name: str, sdf_path: str, idx: int) -> bool:
		now_sec = self.get_clock().now().nanoseconds * 1e-9

		if self.spawned.get(model_name, False):
			return True
		if self.spawn_pending.get(model_name, False):
			since = self.spawn_pending_since.get(model_name, now_sec)
			if now_sec - since <= 3.0:
				return False

			self.spawn_pending[model_name] = False
			self.spawn_pending_since.pop(model_name, None)
			self.get_logger().warning(f"Spawn request timeout, retrying: {model_name}")

		if not self.spawn_client.wait_for_service(timeout_sec=0.02):
			return False

		req = SpawnEntity.Request()
		req.entity_factory.name = model_name
		req.entity_factory.allow_renaming = False
		req.entity_factory.sdf_filename = sdf_path
		req.entity_factory.pose = self._staging_pose(idx)

		future = self.spawn_client.call_async(req)
		self.spawn_pending[model_name] = True
		self.spawn_pending_since[model_name] = now_sec

		def _on_done(done_future) -> None:
			self.spawn_pending[model_name] = False
			self.spawn_pending_since.pop(model_name, None)
			try:
				resp = done_future.result()
				status_text = ""
				if resp is not None:
					status_text = getattr(resp, "status_message", "") or ""
				if resp is not None and resp.success:
					self.spawned[model_name] = True
					self.get_logger().info(f"Spawned agent model in staging: {model_name}")
				elif "already exists" in status_text.lower():
					self.spawned[model_name] = True
					self.get_logger().info(f"Agent model already exists, reuse it: {model_name}")
				else:
					self.get_logger().warning(f"Spawn failed for agent model: {model_name}")
			except Exception as exc:  # pragma: no cover
				self.get_logger().warning(f"Spawn exception for {model_name}: {exc}")

		future.add_done_callback(_on_done)
		return False

	def _spawn_all_agents_in_staging(self) -> bool:
		all_ready = True
		for idx, (model_name, sdf_path) in enumerate(self.agent_specs):
			if not self._request_spawn_if_needed(model_name, sdf_path, idx):
				all_ready = False
		return all_ready

	def _obstacle_clearance_ok(self, x: float, y: float, collision_distance: float) -> bool:
		for ox, oy, sx, sy, is_cylinder in self.obstacles:
			if is_cylinder:
				radius = max(sx, sy) / 2.0
				center_dist = math.hypot(x - ox, y - oy)
				surface_dist = center_dist - radius
				if surface_dist < collision_distance:
					return False
			else:
				half_x = sx / 2.0
				half_y = sy / 2.0
				dx = max(abs(x - ox) - half_x, 0.0)
				dy = max(abs(y - oy) - half_y, 0.0)
				surface_dist = math.hypot(dx, dy)
				if surface_dist < collision_distance:
					return False
		return True

	def _pairwise_clearance_ok(
		self,
		agent_id: int,
		x: float,
		y: float,
		placed: Dict[int, Tuple[float, float]],
		collision_distance: float,
	) -> bool:
		capture_radius = self.get_parameter("capture_radius").get_parameter_value().double_value

		for other_id, (ox, oy) in placed.items():
			d = math.hypot(x - ox, y - oy)
			if d < collision_distance:
				return False

			is_cross_role = self._is_pursuer(agent_id) != self._is_pursuer(other_id)
			if is_cross_role and d <= (capture_radius + collision_distance):
				return False
		return True

	def _boundary_ok(self, x: float, y: float, collision_distance: float) -> bool:
		world_half_extent = self.get_parameter("world_half_extent").get_parameter_value().double_value
		margin = max(0.0, world_half_extent - collision_distance)
		return abs(x) <= margin and abs(y) <= margin

	def _sample_safe_pose(
		self,
		agent_id: int,
		placed: Dict[int, Tuple[float, float]],
	) -> Optional[Pose]:
		collision_distance = self._collision_distance()
		spawn_half_extent = self.get_parameter("spawn_half_extent").get_parameter_value().double_value
		world_half_extent = self.get_parameter("world_half_extent").get_parameter_value().double_value
		attempts = self.get_parameter("spawn_attempts").get_parameter_value().integer_value

		sample_half_extent = min(spawn_half_extent, max(0.1, world_half_extent - collision_distance))

		for _ in range(max(1, attempts)):
			x = random.uniform(-sample_half_extent, sample_half_extent)
			y = random.uniform(-sample_half_extent, sample_half_extent)
			yaw = random.uniform(-math.pi, math.pi)

			if not self._boundary_ok(x, y, collision_distance):
				continue
			if not self._obstacle_clearance_ok(x, y, collision_distance):
				continue
			if not self._pairwise_clearance_ok(agent_id, x, y, placed, collision_distance):
				continue

			pose = Pose()
			pose.position.x = x
			pose.position.y = y
			pose.position.z = self.spawn_z
			pose.orientation.z = math.sin(yaw / 2.0)
			pose.orientation.w = math.cos(yaw / 2.0)
			return pose

		return None

	def _fallback_pose(self, idx: int) -> Pose:
		# Fallback: deterministic spread near corners while respecting current world bounds.
		world_half_extent = self.get_parameter("world_half_extent").get_parameter_value().double_value
		collision_distance = self._collision_distance()
		anchor = max(0.0, world_half_extent - collision_distance - 0.2)

		anchors = [
			(-anchor, -anchor),
			(-anchor, anchor),
			(anchor, -anchor),
			(anchor, anchor),
			(0.0, -anchor),
			(0.0, anchor),
			(-anchor, 0.0),
			(anchor, 0.0),
		]
		x, y = anchors[idx % len(anchors)]
		pose = Pose()
		pose.position.x = x
		pose.position.y = y
		pose.position.z = self.spawn_z
		pose.orientation.w = 1.0
		return pose

	def _sample_pose_set(self) -> List[Pose]:
		poses: List[Pose] = [Pose() for _ in range(self.total_agents)]
		placed: Dict[int, Tuple[float, float]] = {}

		# 先放 evader，再放 pursuer，确保 pursuer 不会初始落在 capture_radius 内。
		sample_order = list(range(self.num_pursuers, self.total_agents)) + list(range(self.num_pursuers))

		for agent_id in sample_order:
			pose = self._sample_safe_pose(agent_id, placed)
			if pose is None:
				self.get_logger().warning(
					f"Failed to sample safe pose for agent_{agent_id}; using deterministic fallback"
				)
				pose = self._fallback_pose(agent_id)

			poses[agent_id] = pose
			placed[agent_id] = (pose.position.x, pose.position.y)

		return poses

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
			pose.position.z = self.spawn_z
			self._call_set_pose(model_name, pose)

		debug_msg = PoseArray()
		debug_msg.header.frame_id = "world"
		debug_msg.poses = poses
		self.reset_pose_debug_pub.publish(debug_msg)
		self.last_reset_poses = poses

	def _process_pending_work(self) -> None:
		if not self._spawn_all_agents_in_staging():
			return

		if self.pending_reset_mode is None:
			return

		if not self.initial_poses:
			self.initial_poses = self._sample_pose_set()

		reset_id = self.pending_reset_id
		mode = self.pending_reset_mode
		self.pending_reset_id = None
		self.pending_reset_mode = None

		if mode == 1:
			self.get_logger().info("Reset mode=1: restore initial poses")
			self._apply_pose_set(self.initial_poses)
		elif mode == 2:
			self.get_logger().info("Reset mode=2: sample collision-distance random poses")
			new_poses = self._sample_pose_set()
			self._apply_pose_set(new_poses)
		else:
			self.get_logger().warning(f"Unsupported reset mode: {mode}, fallback to mode=2")
			new_poses = self._sample_pose_set()
			self._apply_pose_set(new_poses)

		if reset_id is not None:
			done_msg = Int32()
			done_msg.data = int(reset_id)
			self.reset_done_pub.publish(done_msg)
			self.get_logger().info(f"Published /agent_manager/reset_done reset_id={reset_id}")

	def on_reset_agents_cmd(self, msg: Int32MultiArray) -> None:
		if len(msg.data) < 2:
			self.get_logger().warning("Invalid /scheduler/reset_agents_cmd payload, expected [reset_id, mode]")
			return

		self.pending_reset_id = int(msg.data[0])
		self.pending_reset_mode = int(msg.data[1])


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
