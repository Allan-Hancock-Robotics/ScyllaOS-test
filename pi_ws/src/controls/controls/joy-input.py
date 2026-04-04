import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class joyNode(Node):
    def __init__(self):
        super().__init__("joy-input")
        self.get_logger().info("Starting joystick input read...")
        self.cmd_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, Twist: Twist):
        self.get_logger().info("Linear x: " + Twist.linear.x)
        self.get_logger().info("Linear y: " + Twist.linear.y)
        self.get_logger().info("Linear z: " + Twist.linear.z)
        self.get_logger().info("angular x: " + Twist.angular.x)
        self.get_logger().info("angular y: " + Twist.angular.y)
        self.get_logger().info("angular z: " + Twist.angular.z)

def main(args=None):
    rclpy.init(args=args)
    node = joyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()