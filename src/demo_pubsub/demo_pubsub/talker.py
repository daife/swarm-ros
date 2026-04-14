import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self) -> None:
        super().__init__("demo_talker")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.count_ = 0

    def timer_callback(self) -> None:
        msg = String()
        msg.data = f"Hello ROS 2: {self.count_}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.count_ += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
