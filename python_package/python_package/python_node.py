import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPythonNode(Node):
    def __init__(self):
        super().__init__('python_hello_node')
        self.count = 1
        # Publisher to CPP
        self.python_to_cpp_publisher = self.create_publisher(String, '/python_to_cpp', 10)
        # Subscriber from CPP
        self.cpp_to_python_subscriber = self.create_subscription(
            String, '/cpp_to_python', self.on_message_received, 10
        )
        self.timer = self.create_timer(1.0, self.publish_message)

    def on_message_received(self, msg):
        self.get_logger().info(f"Received from CPP: '{msg.data}'")

    def publish_message(self):
        message = String()
        message.data = f"Hello CPP -- {self.count}"
        self.count += 1
        self.get_logger().info(f"Publishing: '{message.data}'")
        self.python_to_cpp_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = HelloPythonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

