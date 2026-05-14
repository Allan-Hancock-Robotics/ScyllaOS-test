#!/usr/bin/env python3
"""Bridge sensor_msgs/Joy to mavros_msgs/ManualControl for ArduSub.

This node publishes raw MAVLink MANUAL_CONTROL-style values through MAVROS:
  x, y, r: -1000..1000
  z:       0..1000, with 500 neutral

Use `ros2 topic echo /joy` to confirm your joystick axis indices, then tune
axis_* and scale_* parameters in the YAML/launch file.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence
from mavros_msgs.srv import CommandLong

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Joy
from mavros_msgs.msg import ManualControl


@dataclass(frozen=True)
class AxisConfig:
    index: int
    scale: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class JoyToManualControl(Node):
    """ROS 2 node that converts /joy to /mavros/manual_control/send."""

    def __init__(self) -> None:
        super().__init__("joy_to_manual_control")

        # Topic parameters
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("manual_control_topic", "/mavros/manual_control/send")

        # Common ROS joystick defaults; tune these after checking `ros2 topic echo /joy`.
        # Signs are handled by scale_*; flip a scale if that direction is backwards.
        self.declare_parameter("axis_forward", 1)      # ManualControl.x
        self.declare_parameter("axis_lateral", 0)      # ManualControl.y
        self.declare_parameter("axis_vertical", 4)     # ManualControl.z about neutral=500
        self.declare_parameter("axis_yaw", 3)          # ManualControl.r

        self.declare_parameter("scale_forward", 1000.0)
        self.declare_parameter("scale_lateral", 1000.0)
        self.declare_parameter("scale_vertical", 500.0)
        self.declare_parameter("scale_yaw", 1000.0)

        # Optional trigger-pair vertical mode. If both indices are >= 0, this overrides
        # axis_vertical. trigger_up and trigger_down are mapped from trigger_rest to
        # trigger_pressed into 0..1, then vertical = up - down.
        self.declare_parameter("axis_vertical_up", -1)
        self.declare_parameter("axis_vertical_down", -1)
        self.declare_parameter("trigger_rest", -1.0)
        self.declare_parameter("trigger_pressed", 1.0)

        # Controls/safety behavior
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("enable_button", -1)    # deadman; -1 disables
        self.declare_parameter("publish_buttons", True)
        self.declare_parameter("log_neutral_timeout", True)

        # Gripper button parameters
        self.declare_parameter("gripper_open_button", 0)   # example: A
        self.declare_parameter("gripper_close_button", 1)  # example: B
        self.declare_parameter("gripper_hold_button", -1)  # disabled by default
        self.declare_parameter("gripper_id", 1)

        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.previous_buttons = []

        joy_topic = str(self.get_parameter("joy_topic").value)
        manual_control_topic = str(self.get_parameter("manual_control_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self._last_joy: Joy | None = None
        self._last_joy_time: Time | None = None
        self._was_timed_out = False

        self._pub = self.create_publisher(ManualControl, manual_control_topic, 10)
        self._sub = self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish)

        self.get_logger().info(
            f"Publishing ManualControl to {manual_control_topic} from {joy_topic} "
            f"at {publish_rate_hz:.1f} Hz"
        )
        self.get_logger().info(
            "ArduSub ManualControl ranges: x/y/r=-1000..1000, z=0..1000 with z=500 neutral"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy = msg
        self._last_joy_time = self.get_clock().now()

        buttons = list(msg.buttons)

        open_button = int(self.get_parameter("gripper_open_button").value)
        close_button = int(self.get_parameter("gripper_close_button").value)
        hold_button = int(self.get_parameter("gripper_hold_button").value)

        if self._button_rising_edge(buttons, open_button):
            self.get_logger().info("Gripper open/release")
            self._send_gripper_action(0)

        if self._button_rising_edge(buttons, close_button):
            self.get_logger().info("Gripper close/grab")
            self._send_gripper_action(1)

        if self._button_rising_edge(buttons, hold_button):
            self.get_logger().info("Gripper hold")
            self._send_gripper_action(2)

        self.previous_buttons = buttons

    def _axis_value(self, axes: Sequence[float], index: int) -> float:
        if index < 0 or index >= len(axes):
            return 0.0
        value = float(axes[index])
        deadzone = float(self.get_parameter("deadzone").value)
        if abs(value) < deadzone:
            return 0.0
        return clamp(value, -1.0, 1.0)

    def _trigger_value(self, axes: Sequence[float], index: int) -> float:
        """Map a trigger axis to 0..1 using trigger_rest/trigger_pressed."""
        if index < 0 or index >= len(axes):
            return 0.0
        raw = float(axes[index])
        rest = float(self.get_parameter("trigger_rest").value)
        pressed = float(self.get_parameter("trigger_pressed").value)
        if abs(pressed - rest) < 1e-6:
            return 0.0
        value = (raw - rest) / (pressed - rest)
        value = clamp(value, 0.0, 1.0)
        deadzone = float(self.get_parameter("deadzone").value)
        if value < deadzone:
            return 0.0
        return value

    def _button_pressed(self, buttons: Sequence[int], index: int) -> bool:
        return 0 <= index < len(buttons) and bool(buttons[index])
    
    def _button_rising_edge(self, buttons, index: int) -> bool:
        if index < 0:
            return False
        if index >= len(buttons):
            return False

        previous = 0
        if index < len(self.previous_buttons):
            previous = self.previous_buttons[index]

        return buttons[index] == 1 and previous == 0


    def _send_gripper_action(self, action: int):
        """
        action:
        0 = release/open
        1 = grab/close
        2 = hold
        """
        if not self.command_client.service_is_ready():
            self.get_logger().warn("/mavros/cmd/command is not available; gripper command skipped")
            return

        gripper_id = int(self.get_parameter("gripper_id").value)

        req = CommandLong.Request()
        req.broadcast = False
        req.command = 211  # MAV_CMD_DO_GRIPPER
        req.confirmation = 0
        req.param1 = float(gripper_id)
        req.param2 = float(action)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self.command_client.call_async(req)
        future.add_done_callback(self._on_gripper_response)


    def _on_gripper_response(self, future):
        try:
            result = future.result()
            if result is None:
                self.get_logger().warn("Gripper command returned no result")
            elif not result.success:
                self.get_logger().warn(f"Gripper command failed: result={result.result}")
        except Exception as exc:
            self.get_logger().warn(f"Gripper command exception: {exc}")


    def _buttons_bitmask(self, buttons: Sequence[int]) -> int:
        if not bool(self.get_parameter("publish_buttons").value):
            return 0
        bitmask = 0
        # MAVLink MANUAL_CONTROL button fields are 16-bit in common use.
        for i, pressed in enumerate(buttons[:16]):
            if pressed:
                bitmask |= 1 << i
        return bitmask

    def _neutral_msg(self) -> ManualControl:
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 500.0
        msg.r = 0.0
        msg.buttons = 0
        return msg

    def _publish(self) -> None:
        now = self.get_clock().now()
        timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)

        if self._last_joy is None or self._last_joy_time is None:
            self._pub.publish(self._neutral_msg())
            return

        age = (now - self._last_joy_time).nanoseconds * 1e-9
        if age > timeout_sec:
            if bool(self.get_parameter("log_neutral_timeout").value) and not self._was_timed_out:
                self.get_logger().warn(
                    f"No /joy input for {age:.2f}s; publishing neutral ManualControl"
                )
            self._was_timed_out = True
            self._pub.publish(self._neutral_msg())
            return

        self._was_timed_out = False
        joy = self._last_joy

        enable_button = int(self.get_parameter("enable_button").value)
        if enable_button >= 0 and not self._button_pressed(joy.buttons, enable_button):
            self._pub.publish(self._neutral_msg())
            return

        forward = self._axis_value(joy.axes, int(self.get_parameter("axis_forward").value))
        lateral = self._axis_value(joy.axes, int(self.get_parameter("axis_lateral").value))
        yaw = self._axis_value(joy.axes, int(self.get_parameter("axis_yaw").value))

        vertical_up_axis = int(self.get_parameter("axis_vertical_up").value)
        vertical_down_axis = int(self.get_parameter("axis_vertical_down").value)
        if vertical_up_axis >= 0 and vertical_down_axis >= 0:
            vertical = (
                self._trigger_value(joy.axes, vertical_up_axis)
                - self._trigger_value(joy.axes, vertical_down_axis)
            )
        else:
            vertical = self._axis_value(joy.axes, int(self.get_parameter("axis_vertical").value))

        scale_forward = float(self.get_parameter("scale_forward").value)
        scale_lateral = float(self.get_parameter("scale_lateral").value)
        scale_vertical = float(self.get_parameter("scale_vertical").value)
        scale_yaw = float(self.get_parameter("scale_yaw").value)

        msg = ManualControl()
        msg.header.stamp = now.to_msg()
        msg.x = float(clamp(forward * scale_forward, -1000.0, 1000.0))
        msg.y = float(clamp(lateral * scale_lateral, -1000.0, 1000.0))
        msg.z = float(clamp(500.0 + vertical * scale_vertical, 0.0, 1000.0))
        msg.r = float(clamp(yaw * scale_yaw, -1000.0, 1000.0))
        msg.buttons = int(self._buttons_bitmask(joy.buttons))

        self._pub.publish(msg)

    def send_neutral_once(self) -> None:
        self._pub.publish(self._neutral_msg())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JoyToManualControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral a few times on shutdown so ArduSub receives it even with packet loss.
        for _ in range(5):
            node.send_neutral_once()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
