from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    env_backend = LaunchConfiguration("env_backend")
    num_pursuers = LaunchConfiguration("num_pursuers")
    num_evaders = LaunchConfiguration("num_evaders")
    max_episodes = LaunchConfiguration("max_episodes")
    max_steps = LaunchConfiguration("max_steps")
    reset_policy = LaunchConfiguration("reset_policy")
    pursuer_train_mode = LaunchConfiguration("pursuer_train_mode")
    pursuer_weights = LaunchConfiguration("pursuer_weights")
    checkpoint_dir = LaunchConfiguration("checkpoint_dir")

    gazebo_condition = IfCondition(PythonExpression(["'", env_backend, "' == 'gazebo'"]))
    mock_condition = IfCondition(PythonExpression(["'", env_backend, "' == 'mock'"]))

    bridge_args = [
        "/world/urban/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        "/model/pursuer_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/model/pursuer_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/model/pursuer_2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/model/pursuer_3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/model/pursuer_4/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/model/evader_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/model/pursuer_0/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
        "/model/pursuer_1/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
        "/model/pursuer_2/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
        "/model/pursuer_3/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
        "/model/pursuer_4/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
        "/model/evader_0/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("env_backend", default_value="mock"),
            DeclareLaunchArgument("num_pursuers", default_value="5"),
            DeclareLaunchArgument("num_evaders", default_value="1"),
            DeclareLaunchArgument("max_episodes", default_value="60"),
            DeclareLaunchArgument("max_steps", default_value="120"),
            DeclareLaunchArgument("reset_policy", default_value="2"),
            DeclareLaunchArgument("pursuer_train_mode", default_value="train_scratch"),
            DeclareLaunchArgument("pursuer_weights", default_value=""),
            DeclareLaunchArgument("checkpoint_dir", default_value="artifacts/checkpoints"),
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args],
                output="screen",
                condition=gazebo_condition,
            ),
            Node(
                package="world_manager",
                executable="world_manager_node",
                name="world_manager_node",
                output="screen",
                parameters=[
                    {
                        "world_mode": "random",
                        "seed_mode": "none",
                        "seed_value": 42,
                        "seed_sequence": [42, 100, 777, 2026],
                        "obstacle_density": 0.3,
                        "world_half_extent": 10.0,
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
                        "safe_spawn_distance": 1.5,
                        "spawn_half_extent": 9.0,
                        "spawn_attempts": 300,
                        "world_name": "urban",
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
                        "world_half_extent": 10.0,
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
                        "pursuer_algo": "cbc_tp_net",
                        "pursuer_train_mode": pursuer_train_mode,
                        "pursuer_weights": pursuer_weights,
                        "evader_algo": "ddpg",
                        "evader_train_mode": "eval_only",
                        "evader_weights": "",
                        "max_accel": 1.0,
                        "capture_radius": 2.0,
                        "collision_penalty": -2.0,
                        "pursuer_lr": 0.0003,
                        "discount_gamma": 0.99,
                        "entropy_coef": 0.001,
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
