import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class ObstacleSpec:
    name: str
    kind: str
    size_x: float
    size_y: float
    size_z: float


class WorldManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("world_manager_node")
        self.declare_parameter("world_mode", "random")
        self.declare_parameter("seed_mode", "none")
        self.declare_parameter("seed_value", 42)
        self.declare_parameter("seed_sequence", [42, 43, 44, 45])
        self.declare_parameter("obstacle_density", 0.3)
        self.declare_parameter("world_half_extent", 10.0)
        self.declare_parameter("world_name", "urban")
        self.declare_parameter("obstacle_model_package", "swarm_bringup")
        self.declare_parameter("obstacle_pool_box", 8)
        self.declare_parameter("obstacle_pool_rect", 8)
        self.declare_parameter("obstacle_pool_cylinder", 8)
        self.declare_parameter("obstacle_spawn_attempts", 400)
        self.declare_parameter("obstacle_min_clearance", 0.2)
        self.declare_parameter("obstacle_staging_x", 1000.0)
        self.declare_parameter("obstacle_staging_y", 1000.0)
        self.declare_parameter("obstacle_staging_stride", 1.5)

        self.world_name = self.get_parameter("world_name").get_parameter_value().string_value

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

        set_pose_service = f"/world/{self.world_name}/set_pose"
        spawn_service = f"/world/{self.world_name}/create"
        self.set_pose_client = self.create_client(SetEntityPose, set_pose_service)
        self.spawn_client = self.create_client(SpawnEntity, spawn_service)

        self._world_reset_count = 0
        self._reset_pending = False
        self._spawn_pending: Dict[str, bool] = {}
        self._spawned: Dict[str, bool] = {}
        self._last_obstacles: List[Tuple[ObstacleSpec, float, float]] = []

        self._box_sdf, self._rect_sdf, self._cylinder_sdf = self._resolve_model_paths()
        self._pool: List[ObstacleSpec] = self._init_obstacle_pool()

        self.on_reset_world(Empty())
        self.create_timer(0.1, self._process_pending_reset)

    def _resolve_model_paths(self) -> Tuple[str, str, str]:
        model_pkg = self.get_parameter("obstacle_model_package").get_parameter_value().string_value
        share = get_package_share_directory(model_pkg)
        return (
            f"{share}/models/obstacle_box/model.sdf",
            f"{share}/models/obstacle_rect/model.sdf",
            f"{share}/models/obstacle_cylinder/model.sdf",
        )

    def _init_obstacle_pool(self) -> List[ObstacleSpec]:
        pool: List[ObstacleSpec] = []

        box_count = self.get_parameter("obstacle_pool_box").get_parameter_value().integer_value
        rect_count = self.get_parameter("obstacle_pool_rect").get_parameter_value().integer_value
        cyl_count = self.get_parameter("obstacle_pool_cylinder").get_parameter_value().integer_value

        for i in range(max(0, box_count)):
            pool.append(
                ObstacleSpec(
                    name=f"obstacle_box_{i}",
                    kind="box",
                    size_x=1.2,
                    size_y=1.2,
                    size_z=1.6,
                )
            )
        for i in range(max(0, rect_count)):
            pool.append(
                ObstacleSpec(
                    name=f"obstacle_rect_{i}",
                    kind="rect",
                    size_x=2.2,
                    size_y=1.0,
                    size_z=1.6,
                )
            )
        for i in range(max(0, cyl_count)):
            pool.append(
                ObstacleSpec(
                    name=f"obstacle_cylinder_{i}",
                    kind="cylinder",
                    size_x=1.4,
                    size_y=1.4,
                    size_z=1.6,
                )
            )
        return pool

    def _target_count(self) -> int:
        density = self.get_parameter("obstacle_density").get_parameter_value().double_value
        density = max(0.0, min(1.0, density))
        base = max(1, int(round(max(1, len(self._pool)) * density)))
        return base

    def _sdf_path_for_kind(self, kind: str) -> str:
        if kind == "box":
            return self._box_sdf
        if kind == "rect":
            return self._rect_sdf
        return self._cylinder_sdf

    def _staging_pose(self, idx: int, z: float) -> Pose:
        staging_x = self.get_parameter("obstacle_staging_x").get_parameter_value().double_value
        staging_y = self.get_parameter("obstacle_staging_y").get_parameter_value().double_value
        stride = self.get_parameter("obstacle_staging_stride").get_parameter_value().double_value

        row = idx // 12
        col = idx % 12
        p = Pose()
        p.position.x = staging_x + col * stride
        p.position.y = staging_y + row * stride
        p.position.z = z / 2.0
        p.orientation.w = 1.0
        return p

    def _request_spawn_if_needed(self, spec: ObstacleSpec) -> bool:
        if self._spawned.get(spec.name, False):
            return True
        if self._spawn_pending.get(spec.name, False):
            return False
        if not self.spawn_client.wait_for_service(timeout_sec=0.02):
            return False

        req = SpawnEntity.Request()
        req.entity_factory.name = spec.name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf_filename = self._sdf_path_for_kind(spec.kind)
        pool_idx = next((i for i, s in enumerate(self._pool) if s.name == spec.name), 0)
        req.entity_factory.pose = self._staging_pose(pool_idx, spec.size_z)

        fut = self.spawn_client.call_async(req)
        self._spawn_pending[spec.name] = True

        def _done(done_future) -> None:
            self._spawn_pending[spec.name] = False
            try:
                res = done_future.result()
                if res is not None and res.success:
                    self._spawned[spec.name] = True
                    self.get_logger().info(f"Spawned obstacle model: {spec.name}")
                else:
                    self.get_logger().warning(f"Spawn failed for obstacle model: {spec.name}")
            except Exception as exc:  # pragma: no cover
                self.get_logger().warning(f"Spawn exception for {spec.name}: {exc}")

        fut.add_done_callback(_done)
        return False

    def _expand_pool_if_needed(self, required: int) -> None:
        if required <= len(self._pool):
            return

        while len(self._pool) < required:
            idx = len(self._pool)
            kind = ["box", "rect", "cylinder"][idx % 3]
            if kind == "box":
                spec = ObstacleSpec(f"obstacle_box_extra_{idx}", "box", 1.2, 1.2, 1.6)
            elif kind == "rect":
                spec = ObstacleSpec(f"obstacle_rect_extra_{idx}", "rect", 2.2, 1.0, 1.6)
            else:
                spec = ObstacleSpec(f"obstacle_cylinder_extra_{idx}", "cylinder", 1.4, 1.4, 1.6)
            self._pool.append(spec)
            self.get_logger().info(f"Expanded obstacle pool with {spec.name}")

    def _set_pose(self, model_name: str, pose: Pose) -> None:
        if not self.set_pose_client.wait_for_service(timeout_sec=0.02):
            return

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = model_name
        req.entity.type = Entity.MODEL
        req.pose = pose
        self.set_pose_client.call_async(req)

    def _sample_random_layout(self, chosen: List[ObstacleSpec]) -> List[Tuple[ObstacleSpec, float, float]]:
        half_extent = self.get_parameter("world_half_extent").get_parameter_value().double_value
        attempts = self.get_parameter("obstacle_spawn_attempts").get_parameter_value().integer_value
        clearance = self.get_parameter("obstacle_min_clearance").get_parameter_value().double_value

        placements: List[Tuple[ObstacleSpec, float, float]] = []
        for spec in chosen:
            radius = max(spec.size_x, spec.size_y) / 2.0
            sampled = False
            for _ in range(max(1, attempts)):
                x = random.uniform(-half_extent + radius, half_extent - radius)
                y = random.uniform(-half_extent + radius, half_extent - radius)

                ok = True
                for other, ox, oy in placements:
                    other_radius = max(other.size_x, other.size_y) / 2.0
                    min_dist = radius + other_radius + clearance
                    if (x - ox) ** 2 + (y - oy) ** 2 < min_dist ** 2:
                        ok = False
                        break
                if not ok:
                    continue

                placements.append((spec, x, y))
                sampled = True
                break

            if not sampled:
                x = random.uniform(-half_extent + radius, half_extent - radius)
                y = random.uniform(-half_extent + radius, half_extent - radius)
                placements.append((spec, x, y))

        return placements

    def _sample_fixed_layout(self, chosen: List[ObstacleSpec]) -> List[Tuple[ObstacleSpec, float, float]]:
        anchors = [
            (-6.0, -4.0),
            (-2.5, 3.5),
            (2.8, -2.8),
            (5.5, 4.5),
            (0.0, -5.8),
            (-6.0, 2.0),
            (6.0, -0.5),
            (1.0, 5.8),
            (-3.8, -0.5),
            (4.0, 1.8),
            (-0.5, -3.0),
            (0.5, 0.5),
        ]

        out: List[Tuple[ObstacleSpec, float, float]] = []
        for i, spec in enumerate(chosen):
            x, y = anchors[i % len(anchors)]
            out.append((spec, x, y))
        return out

    def _compute_layout(self) -> List[Tuple[ObstacleSpec, float, float]]:
        world_mode = self.get_parameter("world_mode").get_parameter_value().string_value
        seed_mode = self.get_parameter("seed_mode").get_parameter_value().string_value
        seed_value = self.get_parameter("seed_value").get_parameter_value().integer_value
        seed_sequence = list(self.get_parameter("seed_sequence").get_parameter_value().integer_array_value)
        target = self._target_count()
        self._expand_pool_if_needed(target)

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

        chosen = self._pool[:target]
        if world_mode == "fixed":
            return self._sample_fixed_layout(chosen)
        return self._sample_random_layout(chosen)

    def _publish_obstacles(self, obstacles: List[Tuple[ObstacleSpec, float, float]]) -> None:
        arr = MarkerArray()
        for i, (spec, x, y) in enumerate(obstacles):
            mk = Marker()
            mk.header.frame_id = "world"
            mk.ns = "obstacles"
            mk.id = i
            mk.type = Marker.CYLINDER if spec.kind == "cylinder" else Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position = Point(x=x, y=y, z=spec.size_z / 2.0)
            mk.pose.orientation.w = 1.0
            mk.scale.x = spec.size_x
            mk.scale.y = spec.size_y
            mk.scale.z = spec.size_z
            mk.color.a = 0.85
            if spec.kind == "box":
                mk.color.r = 0.40
                mk.color.g = 0.62
                mk.color.b = 0.82
            elif spec.kind == "rect":
                mk.color.r = 0.72
                mk.color.g = 0.58
                mk.color.b = 0.42
            else:
                mk.color.r = 0.36
                mk.color.g = 0.72
                mk.color.b = 0.42
            arr.markers.append(mk)

        self.obstacles_pub.publish(arr)
        self.get_logger().info(f"Published {len(arr.markers)} active obstacles")

    def _apply_layout(self, obstacles: List[Tuple[ObstacleSpec, float, float]]) -> bool:
        all_ready = True
        active_names = {spec.name for spec, _, _ in obstacles}

        for spec, _, _ in obstacles:
            if not self._request_spawn_if_needed(spec):
                all_ready = False

        if not all_ready:
            return False

        for idx, spec in enumerate(self._pool):
            if not self._spawned.get(spec.name, False):
                continue
            pose = self._staging_pose(idx, spec.size_z)
            if spec.name in active_names:
                continue
            self._set_pose(spec.name, pose)

        for spec, x, y in obstacles:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = spec.size_z / 2.0
            pose.orientation.w = 1.0
            self._set_pose(spec.name, pose)

        return True

    def _process_pending_reset(self) -> None:
        if not self._reset_pending:
            return

        layout = self._compute_layout()
        applied = self._apply_layout(layout)
        if not applied:
            return

        self._last_obstacles = layout
        self._publish_obstacles(layout)
        self._reset_pending = False
        self.get_logger().info("World reset complete: obstacles repositioned")

    def on_reset_world(self, _msg: Empty) -> None:
        self._world_reset_count += 1
        self._reset_pending = True


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
