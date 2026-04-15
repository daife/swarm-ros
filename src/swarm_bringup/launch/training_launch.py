from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    num_pursuers = LaunchConfiguration("num_pursuers")
    num_evaders = LaunchConfiguration("num_evaders")

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
            DeclareLaunchArgument("num_pursuers", default_value="5"),
            DeclareLaunchArgument("num_evaders", default_value="1"),
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args],
                output="screen",
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
                        "num_pursuers": num_pursuers,
                        "num_evaders": num_evaders,
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
                        "num_pursuers": num_pursuers,
                        "num_evaders": num_evaders,
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
                        "num_pursuers": num_pursuers,
                        "num_evaders": num_evaders,
                        "pursuer_algo": "cbc_tp_net",
                        "pursuer_train_mode": "train_scratch",
                        "pursuer_weights": "",
                        "evader_algo": "ddpg",
                        "evader_train_mode": "eval_only",
                        "evader_weights": "",
                        "max_accel": 1.0,
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
                        "num_pursuers": num_pursuers,
                        "num_evaders": num_evaders,
                        "max_episodes": 4000,
                        "max_steps": 500,
                        "reset_policy": 2,
                        "collision_penalty": -2.0,
                        "capture_radius": 2.0,
                        "capture_steps": 5,
                    }
                ],
            ),
        ]
    )
