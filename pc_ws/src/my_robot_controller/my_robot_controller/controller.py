#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #Twist topic for sending velocity commands
from turtlesim.msg import Pose #turtlesim's Pose topic
from turtlesim.srv import SetPen #turtlesim's set_pen service
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__("controller")
        self.previous_x = 0
        self.get_logger().info("Turtle Controller Starting!")
        self.PoseSubscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.cmd_vel_callback,10)
        self.TwistPublisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        #self.timer_ = self.create_timer(.5, self.cmd_vel_callback)

    def cmd_vel_callback(self, Pose: Pose):
        cmd = Twist()
        #self.get_logger().info(f"x: {Pose.x}, y:{Pose.y}")
        if Pose.x > 9 or Pose.x < 2 or Pose.y > 9 or Pose.y < 2:
            cmd.linear.x = 1.0
            cmd.angular.z = .9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.TwistPublisher_.publish(cmd)

        #because we don't want to call services as many times as we do topics (around 60 times per sec)
        #we only want to call set_pen when the turtle crosses the threshold at the middle
        if Pose.x > 5.5 and self.previous_x <= 5.5:
            self.previous_x = Pose.x
            self.get_logger().info("Setting to red...")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif Pose.x <= 5.5 and self.previous_x > 5.5:
            self.previous_x = Pose.x
            self.get_logger().info("Setting to green...")
            self.call_set_pen_service(0, 255, 0, 3, 0)

    def call_set_pen_service(self, r, g, b, width, off): #calls set_pen service to change trail of turtlesim
        ### parameters are the same as the ones the service needs
        client = self.create_client(SetPen, "/turtle1/set_pen") #
        while not client.wait_for_service(1.0): #if service is not avaiable for 1 sec
            self.get_logger().info().warn("Waiting for service...") #show that service is unavailable
       
        request = SetPen.Request()
        request.r = r
        request.g = g 
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request) #calls service asyncronously so we don't wait for response
        future.add_done_callback(partial(self.callback_set_pen)) #calls callback when service responds

    def callback_set_pen(self, future): #callback for when service replies
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()