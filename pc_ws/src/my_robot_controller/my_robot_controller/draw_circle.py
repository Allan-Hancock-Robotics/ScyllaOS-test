import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class myNode(Node):
        def __init__(self):
                super().__init__("draw_circle")
                self.get_logger().info("Drawing cmd_vel...")#print log statement
                self.vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)#Used to send comtrolls to turtle
                self.timer_ = self.create_timer(0.5, self.send_cmd_vel)#calls send_cmd_vel every .5 secs

        def send_cmd_vel(self):
                msg = Twist()
                msg.linear.x = 10.0
                msg.angular.z = 5.0
                self.vel_pub_.publish(msg)#publish the defined movements

def main(args=None):
    rclpy.init(args=args)
    Node = myNode()
    rclpy.spin(Node)#keeps the node alive by repeating self.timer_
    rclpy.shutdown()
    