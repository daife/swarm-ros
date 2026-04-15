import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
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
        bridge_args.append(
            f"/model/{model_name}/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts"
        )
        bridge_args.append(
            f"/world/{world_name}/model/{model_name}/link/base_link/sensor/contact_sensor/contact"
            "@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts"
        )

    return [
        ExecuteProcess(
            cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args],
            output="screen",
        )
    ]


def _spawn_agents(context, *args, **kwargs):
    num_pursuers = int(LaunchConfiguration("num_pursuers").perform(context))
    num_evaders = int(LaunchConfiguration("num_evaders").perform(context))
    world_name = LaunchConfiguration("world_name").perform(context)

    package_share = get_package_share_directory("swarm_bringup")
    pursuer_sdf = os.path.join(package_share, "models", "pursuer", "model.sdf")
    evader_sdf = os.path.join(package_share, "models", "evader", "model.sdf")

    spawn_actions = []
    spacing = 2.2
    x0 = -6.0
    y0 = -6.0

    all_models = []
    for i in range(num_pursuers):
        all_models.append((f"pursuer_{i}", pursuer_sdf))
    for i in range(num_evaders):
        all_models.append((f"evader_{i}", evader_sdf))

    for idx, (model_name, model_sdf) in enumerate(all_models):
        row = idx // 4
        col = idx % 4
        x = x0 + col * spacing
        y = y0 + row * spacing
        req = (
            f'sdf_filename: "{model_sdf}" '
            f'name: "{model_name}" '
            f'pose: {{position: {{x: {x:.2f}, y: {y:.2f}, z: 0.70}}}}'
        )
        spawn_actions.append(
            ExecuteProcess(
                cmd=[
                    "gz",
                    "service",
                    "-s",
                    f"/world/{world_name}/create",
                    "--reqtype",
                    "gz.msgs.EntityFactory",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "3000",
                    "--req",
                    req,
                ],
                output="screen",
            )
        )

    if not spawn_actions:
        return []

    return [TimerAction(period=4.0, actions=spawn_actions)]


def generate_launch_description() -> LaunchDescription:
    env_backend = LaunchConfiguration("env_backend")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_name = LaunchConfiguration("world_name")
    world_sdf = LaunchConfiguration("world_sdf")
    num_pursuers = LaunchConfiguration("num_pursuers")
    num_evaders = LaunchConfiguration("num_evaders")
    max_episodes = LaunchConfiguration("max_episodes")
    max_steps = LaunchConfiguration("max_steps")
    reset_policy = LaunchConfiguration("reset_policy")
    safe_spawn_distance = LaunchConfiguration("safe_spawn_distance")
    pursuer_algo = LaunchConfiguration("pursuer_algo")
    pursuer_train_mode = LaunchConfiguration("pursuer_train_mode")
    pursuer_weights = LaunchConfiguration("pursuer_weights")
    evader_algo = LaunchConfiguration("evader_algo")
    evader_train_mode = LaunchConfiguration("evader_train_mode")
    evader_weights = LaunchConfiguration("evader_weights")
    checkpoint_dir = LaunchConfiguration("checkpoint_dir")
    buffer_capacity = LaunchConfiguration("buffer_capacity")
    batch_size = LaunchConfiguration("batch_size")
    warmup_steps = LaunchConfiguration("warmup_steps")
    train_updates_per_step = LaunchConfiguration("train_updates_per_step")
    discount_gamma = LaunchConfiguration("discount_gamma")
    target_tau = LaunchConfiguration("target_tau")
    pursuer_actor_lr = LaunchConfiguration("pursuer_actor_lr")
    pursuer_critic_lr = LaunchConfiguration("pursuer_critic_lr")
    evader_actor_lr = LaunchConfiguration("evader_actor_lr")
    evader_critic_lr = LaunchConfiguration("evader_critic_lr")
    exploration_noise_std = LaunchConfiguration("exploration_noise_std")
    policy_update_interval = LaunchConfiguration("policy_update_interval")
    use_synthetic_obs_when_missing = LaunchConfiguration("use_synthetic_obs_when_missing")
    world_half_extent = LaunchConfiguration("world_half_extent")
    gazebo_controller_enabled = LaunchConfiguration("gazebo_controller_enabled")
    gazebo_controller_dt = LaunchConfiguration("gazebo_controller_dt")
    gazebo_max_speed = LaunchConfiguration("gazebo_max_speed")
    gazebo_linear_drag = LaunchConfiguration("gazebo_linear_drag")

    gazebo_condition = IfCondition(PythonExpression(["'", env_backend, "' == 'gazebo'"]))
    gazebo_gui_condition = IfCondition(
        PythonExpression(["'", env_backend, "' == 'gazebo' and '", gazebo_gui, "' == 'true'"])
    )
    gazebo_headless_condition = IfCondition(
        PythonExpression(["'", env_backend, "' == 'gazebo' and '", gazebo_gui, "' != 'true'"])
    )
    mock_condition = IfCondition(PythonExpression(["'", env_backend, "' == 'mock'"]))

    swarm_share = get_package_share_directory("swarm_bringup")
    default_world_path = os.path.join(swarm_share, "worlds", "urban.world")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_launch_file = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("env_backend", default_value="mock"),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("world_name", default_value="urban"),
            DeclareLaunchArgument("world_sdf", default_value=default_world_path),
            DeclareLaunchArgument("num_pursuers", default_value="5"),
            DeclareLaunchArgument("num_evaders", default_value="1"),
            DeclareLaunchArgument("max_episodes", default_value="60"),
            DeclareLaunchArgument("max_steps", default_value="120"),
            DeclareLaunchArgument("reset_policy", default_value="2"),
            DeclareLaunchArgument("safe_spawn_distance", default_value="1.5"),
            DeclareLaunchArgument("pursuer_algo", default_value="maddpg"),
            DeclareLaunchArgument("pursuer_train_mode", default_value="train_scratch"),
            DeclareLaunchArgument("pursuer_weights", default_value=""),
            DeclareLaunchArgument("evader_algo", default_value="ddpg"),
            DeclareLaunchArgument("evader_train_mode", default_value="eval_only"),
            DeclareLaunchArgument("evader_weights", default_value=""),
            DeclareLaunchArgument("checkpoint_dir", default_value="artifacts/checkpoints"),
            DeclareLaunchArgument("buffer_capacity", default_value="200000"),
            DeclareLaunchArgument("batch_size", default_value="128"),
            DeclareLaunchArgument("warmup_steps", default_value="500"),
            DeclareLaunchArgument("train_updates_per_step", default_value="1"),
            DeclareLaunchArgument("discount_gamma", default_value="0.99"),
            DeclareLaunchArgument("target_tau", default_value="0.01"),
            DeclareLaunchArgument("pursuer_actor_lr", default_value="0.0003"),
            DeclareLaunchArgument("pursuer_critic_lr", default_value="0.001"),
            DeclareLaunchArgument("evader_actor_lr", default_value="0.0003"),
            DeclareLaunchArgument("evader_critic_lr", default_value="0.001"),
            DeclareLaunchArgument("exploration_noise_std", default_value="0.2"),
            DeclareLaunchArgument("policy_update_interval", default_value="1"),
            DeclareLaunchArgument("use_synthetic_obs_when_missing", default_value="true"),
            DeclareLaunchArgument("world_half_extent", default_value="10.0"),
            DeclareLaunchArgument("gazebo_controller_enabled", default_value="true"),
            DeclareLaunchArgument("gazebo_controller_dt", default_value="0.05"),
            DeclareLaunchArgument("gazebo_max_speed", default_value="3.0"),
            DeclareLaunchArgument("gazebo_linear_drag", default_value="0.15"),
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
            OpaqueFunction(function=_create_bridge_process, condition=gazebo_condition),
            OpaqueFunction(function=_spawn_agents, condition=gazebo_condition),
            Node(
                package="world_manager",
                executable="world_manager_node",
                name="world_manager_node",
                output="screen",
                parameters=[
                    {
                        "world_mode": ParameterValue(
                            PythonExpression([
                                "'fixed' if '",
                                env_backend,
                                "' == 'gazebo' else 'random'",
                            ]),
                            value_type=str,
                        ),
                        "seed_mode": "none",
                        "seed_value": 42,
                        "seed_sequence": [42, 100, 777, 2026],
                        "obstacle_density": 0.3,
                        "world_half_extent": ParameterValue(world_half_extent, value_type=float),
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
                        "safe_spawn_distance": ParameterValue(safe_spawn_distance, value_type=float),
                        "spawn_half_extent": 9.0,
                        "spawn_attempts": 300,
                        "world_name": world_name,
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
                        "world_name": world_name,
                        "controller_enabled": ParameterValue(gazebo_controller_enabled, value_type=bool),
                        "controller_dt": ParameterValue(gazebo_controller_dt, value_type=float),
                        "max_speed": ParameterValue(gazebo_max_speed, value_type=float),
                        "linear_drag": ParameterValue(gazebo_linear_drag, value_type=float),
                        "world_half_extent": ParameterValue(world_half_extent, value_type=float),
                    }
                ],
                condition=gazebo_condition,
            ),
            Node(
                package="mock_env",
                executable="mock_env_node",
                name="mock_env_node",
                output="screen",
                parameters=[
                    {
                        "num_pursuers": ParameterValue(num_pursuers, value_type=int),
                        "num_evaders": ParameterValue(num_evaders, value_type=int),
                        "dt": 0.05,
                        "world_half_extent": ParameterValue(world_half_extent, value_type=float),
                        "max_speed": 3.0,
                        "linear_drag": 0.15,
                        "spawn_half_extent": 8.0,
                    }
                ],
                condition=mock_condition,
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
                        "max_accel": 1.0,
                        "capture_radius": 2.0,
                        "collision_penalty": -2.0,
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
                        "exploration_noise_std": ParameterValue(exploration_noise_std, value_type=float),
                        "policy_update_interval": ParameterValue(policy_update_interval, value_type=int),
                        "use_synthetic_obs_when_missing": ParameterValue(
                            use_synthetic_obs_when_missing,
                            value_type=bool,
                        ),
                        "checkpoint_dir": checkpoint_dir,
                        "checkpoint_prefix": "pursuer_policy",
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
                        "collision_penalty": -2.0,
                        "capture_radius": 2.0,
                        "capture_steps": 5,
                    }
                ],
            ),
        ]
    )
