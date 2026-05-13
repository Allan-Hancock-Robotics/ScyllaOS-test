import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class joyNode(Node):
    def __init__(self):
        super().__init__("joy_input")
        self.get_logger().info("Starting joystick input read...")
        self.cmd_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.prev_lin_x = 0.0
        self.prev_lin_y = 0.0
        self.prev_lin_z = 0.0
        self.prev_ang_x = 0.0
        self.prev_ang_y = 0.0
        self.prev_ang_z = 0.0

    def cmd_vel_callback(self, Twist: Twist):
        if (Twist.linear.x != self.prev_lin_x):
            self.get_logger().info(f"Linear x: {Twist.linear.x}")
            self.prev_lin_x = Twist.linear.x
        if (Twist.linear.y != self.prev_lin_y):
            self.get_logger().info(f"Linear y: {Twist.linear.y}")
            self.prev_lin_y = Twist.linear.y
        if (Twist.linear.z != self.prev_lin_z):
            self.get_logger().info(f"Linear z: {Twist.linear.z}")
            self.prev_lin_z = Twist.linear.z
        if (Twist.angular.x != self.prev_ang_x):
            self.get_logger().info(f"angular x: {Twist.angular.x}")
            self.prev_ang_x = Twist.angular.x
        if (Twist.angular.y != self.prev_ang_y):
            self.get_logger().info(f"angular y: {Twist.angular.y}")
            self.prev_ang_y = Twist.angular.y
        if (Twist.angular.z != self.prev_ang_z):
            self.get_logger().info(f"angular z: {Twist.angular.z}")
            self.prev_ang_z = Twist.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = joyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()