#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class CmdVelToManualControl(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_manual_control")

        # Max expected cmd_vel magnitudes. Tune these to your planner/joystick output.
        self.declare_parameter("max_linear_x", 1.0)   # m/s or normalized max
        self.declare_parameter("max_linear_y", 1.0)
        self.declare_parameter("max_linear_z", 1.0)
        self.declare_parameter("max_angular_z", 1.0)  # rad/s or normalized max

        # Sign flips for ROS convention -> ArduSub convention.
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", True)      # ROS +Y left, ArduSub +Y right
        self.declare_parameter("invert_z", False)     # ROS +Z up -> ArduSub z > 500
        self.declare_parameter("invert_yaw", True)    # ROS +yaw CCW, ArduSub +r clockwise

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)

        self.pub = self.create_publisher(
            ManualControl,
            "/mavros/manual_control/send",
            10,
        )

        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.on_cmd_vel,
            10,
        )

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.publish_manual_control)

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def scale_axis(self, value: float, max_abs: float, out_max: float = 1000.0) -> float:
        if max_abs <= 0.0:
            return 0.0
        return clamp(value / max_abs, -1.0, 1.0) * out_max

    def publish_manual_control(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9

        timeout = float(self.get_parameter("cmd_timeout_sec").value)

        if age > timeout:
            # Failsafe neutral command.
            x = 0.0
            y = 0.0
            z = 500.0
            r = 0.0
        else:
            cmd = self.last_cmd

            max_x = float(self.get_parameter("max_linear_x").value)
            max_y = float(self.get_parameter("max_linear_y").value)
            max_z = float(self.get_parameter("max_linear_z").value)
            max_yaw = float(self.get_parameter("max_angular_z").value)

            x = self.scale_axis(cmd.linear.x, max_x)
            y = self.scale_axis(cmd.linear.y, max_y)
            z_delta = self.scale_axis(cmd.linear.z, max_z, out_max=500.0)
            r = self.scale_axis(cmd.angular.z, max_yaw)

            if bool(self.get_parameter("invert_x").value):
                x = -x
            if bool(self.get_parameter("invert_y").value):
                y = -y
            if bool(self.get_parameter("invert_z").value):
                z_delta = -z_delta
            if bool(self.get_parameter("invert_yaw").value):
                r = -r

            # ArduSub z is 0..1000, with 500 neutral.
            z = clamp(500.0 + z_delta, 0.0, 1000.0)

        msg = ManualControl()
        msg.header.stamp = now.to_msg()
        msg.x = float(clamp(x, -1000.0, 1000.0))
        msg.y = float(clamp(y, -1000.0, 1000.0))
        msg.z = float(clamp(z, 0.0, 1000.0))
        msg.r = float(clamp(r, -1000.0, 1000.0))
        msg.buttons = 0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelToManualControl()
    try:
        rclpy.spin(node)
    finally:
        # Send one neutral command on shutdown.
        neutral = ManualControl()
        neutral.x = 0.0
        neutral.y = 0.0
        neutral.z = 500.0
        neutral.r = 0.0
        neutral.buttons = 0
        node.pub.publish(neutral)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()