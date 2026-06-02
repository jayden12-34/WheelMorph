import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WheelNode(Node):
    def __init__(self):
        super().__init__('wheel_node')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'hello from wheel_node'
        self.pub.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)


def main():
    rclpy.init()
    node = WheelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
