import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_agent_names(num_pursuers: int, num_evaders: int) -> list[str]:
    names: list[str] = []
    for i in range(num_pursuers):
        names.append(f"pursuer_{i}")
    for i in range(num_evaders):
        names.append(f"evader_{i}")
    return names


def _create_bridge_process(context, *args, **kwargs):
    num_pursuers = int(LaunchConfiguration("num_pursuers").perform(context))
    num_evaders = int(LaunchConfiguration("num_evaders").perform(context))
    world_name = LaunchConfiguration("world_name").perform(context)

    bridge_args = [
        f"/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity",
        f"/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
    ]

    for model_name in _build_agent_names(num_pursuers, num_evaders):
        bridge_args.append(
            f"/model/{model_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"
        )

    return [
        ExecuteProcess(
            cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    artifacts_run_dir = os.path.join(
        "artifacts",
        datetime.now().strftime("%Y%m%d_%H%M%S"),
    )

    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_name = LaunchConfiguration("world_name")
    world_sdf = LaunchConfiguration("world_sdf")
    num_pursuers = LaunchConfiguration("num_pursuers")
    num_evaders = LaunchConfiguration("num_evaders")
    max_episodes = LaunchConfiguration("max_episodes")
    max_steps = LaunchConfiguration("max_steps")
    reset_policy = LaunchConfiguration("reset_policy")
    world_mode = LaunchConfiguration("world_mode")
    seed_mode = LaunchConfiguration("seed_mode")
    seed_value = LaunchConfiguration("seed_value")
    collision_distance = LaunchConfiguration("collision_distance")
    capture_radius = LaunchConfiguration("capture_radius")
    capture_required_pursuers = LaunchConfiguration("capture_required_pursuers")
    capture_success_reward = LaunchConfiguration("capture_success_reward")
    capture_steps = LaunchConfiguration("capture_steps")
    collision_penalty = LaunchConfiguration("collision_penalty")
    terminate_on_collision = LaunchConfiguration("terminate_on_collision")
    obstacle_density = LaunchConfiguration("obstacle_density")
    pursuer_algo = LaunchConfiguration("pursuer_algo")
    pursuer_train_mode = LaunchConfiguration("pursuer_train_mode")
    pursuer_weights = LaunchConfiguration("pursuer_weights")
    evader_algo = LaunchConfiguration("evader_algo")
    evader_train_mode = LaunchConfiguration("evader_train_mode")
    evader_weights = LaunchConfiguration("evader_weights")
    checkpoint_dir = LaunchConfiguration("checkpoint_dir")
    metrics_output_dir = LaunchConfiguration("metrics_output_dir")
    metrics_window_episodes = LaunchConfiguration("metrics_window_episodes")
    buffer_capacity = LaunchConfiguration("buffer_capacity")
    batch_size = LaunchConfiguration("batch_size")
    warmup_steps = LaunchConfiguration("warmup_steps")
    train_updates_per_step = LaunchConfiguration("train_updates_per_step")
    train_every_steps = LaunchConfiguration("train_every_steps")
    discount_gamma = LaunchConfiguration("discount_gamma")
    target_tau = LaunchConfiguration("target_tau")
    pursuer_actor_lr = LaunchConfiguration("pursuer_actor_lr")
    pursuer_critic_lr = LaunchConfiguration("pursuer_critic_lr")
    evader_actor_lr = LaunchConfiguration("evader_actor_lr")
    evader_critic_lr = LaunchConfiguration("evader_critic_lr")
    exploration_noise_std = LaunchConfiguration("exploration_noise_std")
    policy_update_interval = LaunchConfiguration("policy_update_interval")
    optimize_horizon_steps = LaunchConfiguration("optimize_horizon_steps")
    tp_history_steps = LaunchConfiguration("tp_history_steps")
    tp_predict_steps = LaunchConfiguration("tp_predict_steps")
    tp_hidden_dim = LaunchConfiguration("tp_hidden_dim")
    tp_lr = LaunchConfiguration("tp_lr")
    cbc_hidden_dim = LaunchConfiguration("cbc_hidden_dim")
    reward_safe_distance = LaunchConfiguration("reward_safe_distance")
    reward_normalization = LaunchConfiguration("reward_normalization")
    use_synthetic_obs_when_missing = LaunchConfiguration("use_synthetic_obs_when_missing")
    torch_device = LaunchConfiguration("torch_device")
    world_half_extent = LaunchConfiguration("world_half_extent")
    spawn_half_extent = LaunchConfiguration("spawn_half_extent")
    spawn_attempts = LaunchConfiguration("spawn_attempts")
    agent_altitude_step = LaunchConfiguration("agent_altitude_step")
    paper_spawn_layout = LaunchConfiguration("paper_spawn_layout")
    target_spawn_half_extent = LaunchConfiguration("target_spawn_half_extent")
    pursuer_edge_band = LaunchConfiguration("pursuer_edge_band")
    ignore_same_role_spawn_clearance = LaunchConfiguration("ignore_same_role_spawn_clearance")
    pursuer_max_accel = LaunchConfiguration("pursuer_max_accel")
    evader_max_accel = LaunchConfiguration("evader_max_accel")
    max_accel = LaunchConfiguration("max_accel")
    checkpoint_prefix = LaunchConfiguration("checkpoint_prefix")
    obstacle_model_package = LaunchConfiguration("obstacle_model_package")
    obstacle_pool_box = LaunchConfiguration("obstacle_pool_box")
    obstacle_pool_rect = LaunchConfiguration("obstacle_pool_rect")
    obstacle_pool_cylinder = LaunchConfiguration("obstacle_pool_cylinder")
    obstacle_spawn_attempts = LaunchConfiguration("obstacle_spawn_attempts")
    obstacle_min_clearance = LaunchConfiguration("obstacle_min_clearance")
    obstacle_staging_x = LaunchConfiguration("obstacle_staging_x")
    obstacle_staging_y = LaunchConfiguration("obstacle_staging_y")
    obstacle_staging_stride = LaunchConfiguration("obstacle_staging_stride")
    pursuer_name_prefix = LaunchConfiguration("pursuer_name_prefix")
    evader_name_prefix = LaunchConfiguration("evader_name_prefix")
    agent_model_package = LaunchConfiguration("agent_model_package")
    agent_staging_x = LaunchConfiguration("agent_staging_x")
    agent_staging_y = LaunchConfiguration("agent_staging_y")
    agent_staging_stride = LaunchConfiguration("agent_staging_stride")
    gazebo_controller_enabled = LaunchConfiguration("gazebo_controller_enabled")
    gazebo_controller_dt = LaunchConfiguration("gazebo_controller_dt")
    gazebo_max_speed = LaunchConfiguration("gazebo_max_speed")
    gazebo_pursuer_max_speed = LaunchConfiguration("gazebo_pursuer_max_speed")
    gazebo_evader_max_speed = LaunchConfiguration("gazebo_evader_max_speed")
    gazebo_linear_drag = LaunchConfiguration("gazebo_linear_drag")
    gazebo_control_requires_reset = LaunchConfiguration("gazebo_control_requires_reset")
    fixed_z = LaunchConfiguration("fixed_z")
    initial_reset_delay_sec = LaunchConfiguration("initial_reset_delay_sec")
    always_reset_world = LaunchConfiguration("always_reset_world")
    gazebo_gui_condition = IfCondition(PythonExpression(["'", gazebo_gui, "' == 'true'"]))
    gazebo_headless_condition = IfCondition(PythonExpression(["'", gazebo_gui, "' != 'true'"]))

    swarm_share = get_package_share_directory("swarm_bringup")
    default_world_path = os.path.join(swarm_share, "worlds", "urban.world")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_launch_file = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="urban"),
            DeclareLaunchArgument("world_sdf", default_value=default_world_path),
            DeclareLaunchArgument("num_pursuers", default_value="5"),
            DeclareLaunchArgument("num_evaders", default_value="1"),
            DeclareLaunchArgument("max_episodes", default_value="4000"),
            DeclareLaunchArgument("max_steps", default_value="1000"),
            DeclareLaunchArgument("reset_policy", default_value="2"),
            DeclareLaunchArgument("world_mode", default_value="none"),
            DeclareLaunchArgument("seed_mode", default_value="single"),
            DeclareLaunchArgument("seed_value", default_value="42"),
            DeclareLaunchArgument("collision_distance", default_value="0.8"),
            DeclareLaunchArgument("capture_radius", default_value="2.0"),
            DeclareLaunchArgument("capture_required_pursuers", default_value="3"),
            DeclareLaunchArgument("capture_success_reward", default_value="5.0"),
            DeclareLaunchArgument("capture_steps", default_value="5"),
            DeclareLaunchArgument("collision_penalty", default_value="-2.0"),
            DeclareLaunchArgument("terminate_on_collision", default_value="false"),
            DeclareLaunchArgument("obstacle_density", default_value="0.0"),
            DeclareLaunchArgument("pursuer_algo", default_value="cbc_tp_net"),
            DeclareLaunchArgument("pursuer_train_mode", default_value="train_scratch"),
            DeclareLaunchArgument("pursuer_weights", default_value=""),
            DeclareLaunchArgument("evader_algo", default_value="ddpg"),
            DeclareLaunchArgument("evader_train_mode", default_value="eval_only"),
            DeclareLaunchArgument("evader_weights", default_value=""),
            DeclareLaunchArgument("max_accel", default_value="1.0"),
            DeclareLaunchArgument("pursuer_max_accel", default_value="1.2"),
            DeclareLaunchArgument("evader_max_accel", default_value="0.8"),
            DeclareLaunchArgument(
                "checkpoint_dir",
                default_value=os.path.join(artifacts_run_dir, "checkpoints"),
            ),
            DeclareLaunchArgument("checkpoint_prefix", default_value="pursuer_policy"),
            DeclareLaunchArgument(
                "metrics_output_dir",
                default_value=os.path.join(artifacts_run_dir, "metrics"),
            ),
            DeclareLaunchArgument("metrics_window_episodes", default_value="100"),
            DeclareLaunchArgument("buffer_capacity", default_value="1000000"),
            DeclareLaunchArgument("batch_size", default_value="512"),
            DeclareLaunchArgument("warmup_steps", default_value="500"),
            DeclareLaunchArgument("train_updates_per_step", default_value="1"),
            DeclareLaunchArgument("train_every_steps", default_value="50"),
            DeclareLaunchArgument("discount_gamma", default_value="0.95"),
            DeclareLaunchArgument("target_tau", default_value="0.01"),
            DeclareLaunchArgument("pursuer_actor_lr", default_value="0.001"),
            DeclareLaunchArgument("pursuer_critic_lr", default_value="0.001"),
            DeclareLaunchArgument("evader_actor_lr", default_value="0.001"),
            DeclareLaunchArgument("evader_critic_lr", default_value="0.001"),
            DeclareLaunchArgument("exploration_noise_std", default_value="0.2"),
            DeclareLaunchArgument("policy_update_interval", default_value="1"),
            DeclareLaunchArgument("optimize_horizon_steps", default_value="500"),
            DeclareLaunchArgument("tp_history_steps", default_value="10"),
            DeclareLaunchArgument("tp_predict_steps", default_value="3"),
            DeclareLaunchArgument("tp_hidden_dim", default_value="256"),
            DeclareLaunchArgument("tp_lr", default_value="0.001"),
            DeclareLaunchArgument("cbc_hidden_dim", default_value="256"),
            DeclareLaunchArgument("reward_safe_distance", default_value="2.0"),
            DeclareLaunchArgument("reward_normalization", default_value="0.05"),
            DeclareLaunchArgument("use_synthetic_obs_when_missing", default_value="true"),
            DeclareLaunchArgument("torch_device", default_value="auto"),
            DeclareLaunchArgument("world_half_extent", default_value="7.0"),
            DeclareLaunchArgument("spawn_half_extent", default_value="6.5"),
            DeclareLaunchArgument("spawn_attempts", default_value="300"),
            DeclareLaunchArgument("agent_altitude_step", default_value="0.35"),
            DeclareLaunchArgument("paper_spawn_layout", default_value="true"),
            DeclareLaunchArgument("target_spawn_half_extent", default_value="2.0"),
            DeclareLaunchArgument("pursuer_edge_band", default_value="2.0"),
            DeclareLaunchArgument("ignore_same_role_spawn_clearance", default_value="true"),
            DeclareLaunchArgument("obstacle_model_package", default_value="swarm_bringup"),
            DeclareLaunchArgument("obstacle_pool_box", default_value="8"),
            DeclareLaunchArgument("obstacle_pool_rect", default_value="8"),
            DeclareLaunchArgument("obstacle_pool_cylinder", default_value="8"),
            DeclareLaunchArgument("obstacle_spawn_attempts", default_value="400"),
            DeclareLaunchArgument("obstacle_min_clearance", default_value="0.2"),
            DeclareLaunchArgument("obstacle_staging_x", default_value="1000.0"),
            DeclareLaunchArgument("obstacle_staging_y", default_value="1000.0"),
            DeclareLaunchArgument("obstacle_staging_stride", default_value="1.5"),
            DeclareLaunchArgument("pursuer_name_prefix", default_value="pursuer"),
            DeclareLaunchArgument("evader_name_prefix", default_value="evader"),
            DeclareLaunchArgument("agent_model_package", default_value="swarm_bringup"),
            DeclareLaunchArgument("agent_staging_x", default_value="1000.0"),
            DeclareLaunchArgument("agent_staging_y", default_value="1200.0"),
            DeclareLaunchArgument("agent_staging_stride", default_value="1.5"),
            DeclareLaunchArgument("gazebo_controller_enabled", default_value="true"),
            DeclareLaunchArgument("gazebo_controller_dt", default_value="0.02"),
            DeclareLaunchArgument("gazebo_max_speed", default_value="3.0"),
            DeclareLaunchArgument("gazebo_pursuer_max_speed", default_value="2.5"),
            DeclareLaunchArgument("gazebo_evader_max_speed", default_value="3.0"),
            DeclareLaunchArgument("gazebo_linear_drag", default_value="0.15"),
            DeclareLaunchArgument("gazebo_control_requires_reset", default_value="true"),
            DeclareLaunchArgument("fixed_z", default_value="1.0"),
            DeclareLaunchArgument("initial_reset_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("always_reset_world", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_file),
                launch_arguments={
                    "gz_args": PythonExpression(["'-r -v 3 ", world_sdf, "'"])
                }.items(),
                condition=gazebo_gui_condition,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_file),
                launch_arguments={
                    "gz_args": PythonExpression(["'-r -v 3 -s ", world_sdf, "'"])
                }.items(),
                condition=gazebo_headless_condition,
            ),
            OpaqueFunction(function=_create_bridge_process),
            Node(
                package="world_manager",
                executable="world_manager_node",
                name="world_manager_node",
                output="screen",
                parameters=[
                    {
                        "world_mode": world_mode,
                        "seed_mode": seed_mode,
                        "seed_value": ParameterValue(seed_value, value_type=int),
                        "seed_sequence": [42, 100, 777, 2026],
                        "obstacle_density": ParameterValue(obstacle_density, value_type=float),
                        "world_half_extent": ParameterValue(world_half_extent, value_type=float),
                        "world_name": world_name,
                        "obstacle_model_package": obstacle_model_package,
                        "obstacle_pool_box": ParameterValue(obstacle_pool_box, value_type=int),
                        "obstacle_pool_rect": ParameterValue(obstacle_pool_rect, value_type=int),
                        "obstacle_pool_cylinder": ParameterValue(
                            obstacle_pool_cylinder,
                            value_type=int,
                        ),
                        "obstacle_spawn_attempts": ParameterValue(
                            obstacle_spawn_attempts,
                            value_type=int,
                        ),
                        "obstacle_min_clearance": ParameterValue(
                            obstacle_min_clearance,
                            value_type=float,
                        ),
                        "obstacle_staging_x": ParameterValue(obstacle_staging_x, value_type=float),
                        "obstacle_staging_y": ParameterValue(obstacle_staging_y, value_type=float),
                        "obstacle_staging_stride": ParameterValue(
                            obstacle_staging_stride,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="agent_manager",
                executable="agent_manager_node",
                name="agent_manager_node",
                output="screen",
                parameters=[
                    {
                        "num_pursuers": ParameterValue(num_pursuers, value_type=int),
                        "num_evaders": ParameterValue(num_evaders, value_type=int),
                        "collision_distance": ParameterValue(collision_distance, value_type=float),
                        "capture_radius": ParameterValue(capture_radius, value_type=float),
                        "spawn_half_extent": ParameterValue(spawn_half_extent, value_type=float),
                        "spawn_attempts": ParameterValue(spawn_attempts, value_type=int),
                        "world_half_extent": ParameterValue(world_half_extent, value_type=float),
                        "spawn_z": ParameterValue(fixed_z, value_type=float),
                        "agent_altitude_step": ParameterValue(agent_altitude_step, value_type=float),
                        "paper_spawn_layout": ParameterValue(paper_spawn_layout, value_type=bool),
                        "target_spawn_half_extent": ParameterValue(
                            target_spawn_half_extent,
                            value_type=float,
                        ),
                        "pursuer_edge_band": ParameterValue(pursuer_edge_band, value_type=float),
                        "ignore_same_role_spawn_clearance": ParameterValue(
                            ignore_same_role_spawn_clearance,
                            value_type=bool,
                        ),
                        "world_name": world_name,
                        "pursuer_name_prefix": pursuer_name_prefix,
                        "evader_name_prefix": evader_name_prefix,
                        "agent_model_package": agent_model_package,
                        "agent_staging_x": ParameterValue(agent_staging_x, value_type=float),
                        "agent_staging_y": ParameterValue(agent_staging_y, value_type=float),
                        "agent_staging_stride": ParameterValue(agent_staging_stride, value_type=float),
                    }
                ],
            ),
            Node(
                package="sim_bridge",
                executable="sim_bridge_node",
                name="sim_bridge_node",
                output="screen",
                parameters=[
                    {
                        "num_pursuers": ParameterValue(num_pursuers, value_type=int),
                        "num_evaders": ParameterValue(num_evaders, value_type=int),
                        "pursuer_name_prefix": pursuer_name_prefix,
                        "evader_name_prefix": evader_name_prefix,
                        "world_name": world_name,
                        "controller_enabled": ParameterValue(gazebo_controller_enabled, value_type=bool),
                        "controller_dt": ParameterValue(gazebo_controller_dt, value_type=float),
                        "max_speed": ParameterValue(gazebo_max_speed, value_type=float),
                        "pursuer_max_speed": ParameterValue(gazebo_pursuer_max_speed, value_type=float),
                        "evader_max_speed": ParameterValue(gazebo_evader_max_speed, value_type=float),
                        "linear_drag": ParameterValue(gazebo_linear_drag, value_type=float),
                        "collision_distance": ParameterValue(collision_distance, value_type=float),
                        "control_requires_reset": ParameterValue(
                            gazebo_control_requires_reset,
                            value_type=bool,
                        ),
                        "world_half_extent": ParameterValue(world_half_extent, value_type=float),
                        "fixed_z": ParameterValue(fixed_z, value_type=float),
                        "agent_altitude_step": ParameterValue(agent_altitude_step, value_type=float),
                    }
                ],
            ),
            Node(
                package="rl_policy",
                executable="rl_policy_node",
                name="rl_policy_node",
                output="screen",
                parameters=[
                    {
                        "num_pursuers": ParameterValue(num_pursuers, value_type=int),
                        "num_evaders": ParameterValue(num_evaders, value_type=int),
                        "pursuer_algo": pursuer_algo,
                        "pursuer_train_mode": pursuer_train_mode,
                        "pursuer_weights": pursuer_weights,
                        "evader_algo": evader_algo,
                        "evader_train_mode": evader_train_mode,
                        "evader_weights": evader_weights,
                        "max_accel": ParameterValue(max_accel, value_type=float),
                        "pursuer_max_accel": ParameterValue(pursuer_max_accel, value_type=float),
                        "evader_max_accel": ParameterValue(evader_max_accel, value_type=float),
                        "capture_radius": ParameterValue(capture_radius, value_type=float),
                        "capture_required_pursuers": ParameterValue(
                            capture_required_pursuers,
                            value_type=int,
                        ),
                        "capture_success_reward": ParameterValue(
                            capture_success_reward,
                            value_type=float,
                        ),
                        "collision_penalty": ParameterValue(collision_penalty, value_type=float),
                        "terminate_on_collision": ParameterValue(terminate_on_collision, value_type=bool),
                        "discount_gamma": ParameterValue(discount_gamma, value_type=float),
                        "target_tau": ParameterValue(target_tau, value_type=float),
                        "pursuer_actor_lr": ParameterValue(pursuer_actor_lr, value_type=float),
                        "pursuer_critic_lr": ParameterValue(pursuer_critic_lr, value_type=float),
                        "evader_actor_lr": ParameterValue(evader_actor_lr, value_type=float),
                        "evader_critic_lr": ParameterValue(evader_critic_lr, value_type=float),
                        "buffer_capacity": ParameterValue(buffer_capacity, value_type=int),
                        "batch_size": ParameterValue(batch_size, value_type=int),
                        "warmup_steps": ParameterValue(warmup_steps, value_type=int),
                        "train_updates_per_step": ParameterValue(train_updates_per_step, value_type=int),
                        "train_every_steps": ParameterValue(train_every_steps, value_type=int),
                        "exploration_noise_std": ParameterValue(exploration_noise_std, value_type=float),
                        "policy_update_interval": ParameterValue(policy_update_interval, value_type=int),
                        "optimize_horizon_steps": ParameterValue(optimize_horizon_steps, value_type=int),
                        "tp_history_steps": ParameterValue(tp_history_steps, value_type=int),
                        "tp_predict_steps": ParameterValue(tp_predict_steps, value_type=int),
                        "tp_hidden_dim": ParameterValue(tp_hidden_dim, value_type=int),
                        "tp_lr": ParameterValue(tp_lr, value_type=float),
                        "cbc_hidden_dim": ParameterValue(cbc_hidden_dim, value_type=int),
                        "reward_safe_distance": ParameterValue(reward_safe_distance, value_type=float),
                        "reward_normalization": ParameterValue(reward_normalization, value_type=float),
                        "use_synthetic_obs_when_missing": ParameterValue(
                            use_synthetic_obs_when_missing,
                            value_type=bool,
                        ),
                        "torch_device": torch_device,
                        "checkpoint_dir": checkpoint_dir,
                        "checkpoint_prefix": checkpoint_prefix,
                        "metrics_output_dir": metrics_output_dir,
                    }
                ],
            ),
            Node(
                package="training_coordinator",
                executable="training_coordinator_node",
                name="training_coordinator_node",
                output="screen",
                parameters=[
                    {
                        "num_pursuers": ParameterValue(num_pursuers, value_type=int),
                        "num_evaders": ParameterValue(num_evaders, value_type=int),
                        "max_episodes": ParameterValue(max_episodes, value_type=int),
                        "max_steps": ParameterValue(max_steps, value_type=int),
                        "reset_policy": ParameterValue(reset_policy, value_type=int),
                        "collision_penalty": ParameterValue(collision_penalty, value_type=float),
                        "terminate_on_collision": ParameterValue(terminate_on_collision, value_type=bool),
                        "capture_radius": ParameterValue(capture_radius, value_type=float),
                        "capture_required_pursuers": ParameterValue(
                            capture_required_pursuers,
                            value_type=int,
                        ),
                        "capture_steps": ParameterValue(capture_steps, value_type=int),
                        "initial_reset_delay_sec": ParameterValue(
                            initial_reset_delay_sec,
                            value_type=float,
                        ),
                        "always_reset_world": ParameterValue(always_reset_world, value_type=bool),
                        "metrics_output_dir": metrics_output_dir,
                        "metrics_window_episodes": ParameterValue(
                            metrics_window_episodes,
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
