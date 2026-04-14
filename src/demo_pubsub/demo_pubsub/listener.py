import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self) -> None:
        super().__init__("demo_listener")
        self.subscription = self.create_subscription(
            String, "chatter", self.listener_callback, 10
        )

    def listener_callback(self, msg: String) -> None:
        self.get_logger().info(f"I heard: {msg.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
