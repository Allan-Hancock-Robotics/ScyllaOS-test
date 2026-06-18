#!/usr/bin/env python3

from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from mavros_msgs.msg import ManualControl
from mavros_msgs.srv import CommandLong

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

class JoyToManualControl(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_manual_control")

        # -------------------------
        # Movement axis parameters
        # -------------------------
        self.declare_parameter("axis_forward", 1)
        self.declare_parameter("axis_lateral", 0)
        self.declare_parameter("axis_yaw", 3)

        # Single-axis vertical fallback.
        self.declare_parameter("axis_vertical", 4)

        # Trigger-pair vertical mode.
        # If both are >= 0, this overrides axis_vertical.
        self.declare_parameter("axis_vertical_up", -1)
        self.declare_parameter("axis_vertical_down", -1)
        self.declare_parameter("trigger_rest", 0.0)
        self.declare_parameter("trigger_pressed", -1.0)

        # Scaling
        self.declare_parameter("scale_forward", 1000.0)
        self.declare_parameter("scale_lateral", 1000.0)
        self.declare_parameter("scale_vertical", 500.0)
        self.declare_parameter("scale_yaw", 1000.0)

        # Behavior
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("enable_button", -1)
        self.declare_parameter("publish_buttons", False)
        self.declare_parameter("log_neutral_timeout", True)

        # -------------------------
        # Gripper / claw parameters
        # -------------------------
        self.declare_parameter("gripper_open_button", 0)    # A
        self.declare_parameter("gripper_close_button", 1)   # B
        self.declare_parameter("gripper_hold_button", -1)

        # Direct-servo mode: uses MAV_CMD_DO_SET_SERVO.
        self.declare_parameter("gripper_use_direct_servo", True)
        self.declare_parameter("gripper_servo_number", 9)

        self.declare_parameter("gripper_open_pwm", 1100.0)
        self.declare_parameter("gripper_close_pwm", 1900.0)
        self.declare_parameter("gripper_neutral_pwm", 1500.0)
        self.declare_parameter("gripper_pulse_sec", 0.5)

        # Legacy gripper-command mode: uses MAV_CMD_DO_GRIPPER.
        self.declare_parameter("gripper_id", 1)

        self.manual_pub = self.create_publisher(
            ManualControl,
            "/mavros/manual_control/send",
            10,
        )

        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self._on_joy,
            10,
        )

        self.command_client = self.create_client(
            CommandLong,
            "/mavros/cmd/command",
        )

        self._last_joy: Joy | None = None
        self._last_joy_time = self.get_clock().now()
        self._previous_buttons: list[int] = []
        self._neutral_timeout_logged = False
        self._gripper_neutral_timer = None

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            publish_rate_hz = 20.0

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_manual_control,
        )

        self.get_logger().info("joy_to_manual_control started")

    # -------------------------------------------------------------------------
    # Joy callback
    # -------------------------------------------------------------------------

    def _on_joy(self, msg: Joy) -> None:
        self._handle_gripper_buttons(msg.buttons)

        self._last_joy = msg
        self._last_joy_time = self.get_clock().now()
        self._neutral_timeout_logged = False

        self._previous_buttons = list(msg.buttons)

    # -------------------------------------------------------------------------
    # ManualControl publishing
    # -------------------------------------------------------------------------

    def _publish_manual_control(self) -> None:
        now = self.get_clock().now()

        timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        joy_is_fresh = False

        if self._last_joy is not None:
            age_sec = (now - self._last_joy_time).nanoseconds * 1e-9
            joy_is_fresh = age_sec <= timeout_sec

        enable_button = int(self.get_parameter("enable_button").value)
        enabled = True

        if self._last_joy is not None and enable_button >= 0:
            enabled = (
                enable_button < len(self._last_joy.buttons)
                and self._last_joy.buttons[enable_button] == 1
            )

        if self._last_joy is None or not joy_is_fresh or not enabled:
            if (
                self._last_joy is not None
                and not joy_is_fresh
                and bool(self.get_parameter("log_neutral_timeout").value)
                and not self._neutral_timeout_logged
            ):
                self.get_logger().warn("No recent /joy message; publishing neutral")
                self._neutral_timeout_logged = True

            self._publish_manual_neutral()
            return

        joy = self._last_joy

        x = self._axis_to_scaled(
            joy.axes,
            int(self.get_parameter("axis_forward").value),
            float(self.get_parameter("scale_forward").value),
        )

        y = self._axis_to_scaled(
            joy.axes,
            int(self.get_parameter("axis_lateral").value),
            float(self.get_parameter("scale_lateral").value),
        )

        r = self._axis_to_scaled(
            joy.axes,
            int(self.get_parameter("axis_yaw").value),
            float(self.get_parameter("scale_yaw").value),
        )

        z = self._vertical_to_manual_z(joy.axes)

        buttons = 0
        if bool(self.get_parameter("publish_buttons").value):
            buttons = self._buttons_to_bitmask(joy.buttons)

        msg = ManualControl()
        msg.header.stamp = now.to_msg()
        msg.x = float(clamp(x, -1000.0, 1000.0))
        msg.y = float(clamp(y, -1000.0, 1000.0))
        msg.z = float(clamp(z, 0.0, 1000.0))
        msg.r = float(clamp(r, -1000.0, 1000.0))
        msg.buttons = int(buttons)

        self.manual_pub.publish(msg)

    def _publish_manual_neutral(self) -> None:
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 500.0
        msg.r = 0.0
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def send_neutral_once(self) -> None:
        self._publish_manual_neutral()

    # -------------------------------------------------------------------------
    # Axis helpers
    # -------------------------------------------------------------------------

    def _axis_to_scaled(
        self,
        axes: Sequence[float],
        axis_index: int,
        scale: float,
    ) -> float:
        if axis_index < 0 or axis_index >= len(axes):
            return 0.0

        value = float(axes[axis_index])
        deadzone = float(self.get_parameter("deadzone").value)

        if abs(value) < deadzone:
            value = 0.0

        return clamp(value * scale, -1000.0, 1000.0)

    def _vertical_to_manual_z(self, axes: Sequence[float]) -> float:
        axis_up = int(self.get_parameter("axis_vertical_up").value)
        axis_down = int(self.get_parameter("axis_vertical_down").value)

        scale_vertical = float(self.get_parameter("scale_vertical").value)

        # Trigger-pair mode
        if axis_up >= 0 and axis_down >= 0:
            up = self._trigger_to_unit(axes, axis_up)
            down = self._trigger_to_unit(axes, axis_down)

            vertical = up - down
            z_delta = vertical * scale_vertical
            return clamp(500.0 + z_delta, 0.0, 1000.0)

        # Single-axis fallback
        axis_vertical = int(self.get_parameter("axis_vertical").value)
        vertical_scaled = self._axis_to_scaled(
            axes,
            axis_vertical,
            scale_vertical,
        )

        return clamp(500.0 + vertical_scaled, 0.0, 1000.0)

    def _trigger_to_unit(self, axes: Sequence[float], axis_index: int) -> float:
        if axis_index < 0 or axis_index >= len(axes):
            return 0.0

        value = float(axes[axis_index])
        rest = float(self.get_parameter("trigger_rest").value)
        pressed = float(self.get_parameter("trigger_pressed").value)

        denom = pressed - rest
        if abs(denom) < 1e-6:
            return 0.0

        unit = (value - rest) / denom
        return clamp(unit, 0.0, 1.0)

    def _buttons_to_bitmask(self, buttons: Sequence[int]) -> int:
        mask = 0
        for index, value in enumerate(buttons[:16]):
            if int(value) != 0:
                mask |= 1 << index
        return mask

    # -------------------------------------------------------------------------
    # Gripper / claw handling
    # -------------------------------------------------------------------------

    def _button_rising_edge(self, buttons: Sequence[int], index: int) -> bool:
        if index < 0 or index >= len(buttons):
            return False

        previous = 0
        if index < len(self._previous_buttons):
            previous = int(self._previous_buttons[index])

        return int(buttons[index]) == 1 and previous == 0

    def _handle_gripper_buttons(self, buttons: Sequence[int]) -> None:
        open_button = int(self.get_parameter("gripper_open_button").value)
        close_button = int(self.get_parameter("gripper_close_button").value)
        hold_button = int(self.get_parameter("gripper_hold_button").value)

        if self._button_rising_edge(buttons, open_button):
            self.get_logger().info("Gripper open/release")
            self._gripper_open()

        if self._button_rising_edge(buttons, close_button):
            self.get_logger().info("Gripper close/grab")
            self._gripper_close()

        if self._button_rising_edge(buttons, hold_button):
            self.get_logger().info("Gripper neutral/hold")
            self._cancel_gripper_neutral_timer()
            self._gripper_neutral()

    def _gripper_open(self) -> None:
        if bool(self.get_parameter("gripper_use_direct_servo").value):
            pwm = float(self.get_parameter("gripper_open_pwm").value)
            self._send_gripper_servo_pwm(pwm)
            self._schedule_gripper_neutral()
        else:
            self._send_gripper_action(0)

    def _gripper_close(self) -> None:
        if bool(self.get_parameter("gripper_use_direct_servo").value):
            pwm = float(self.get_parameter("gripper_close_pwm").value)
            self._send_gripper_servo_pwm(pwm)
            self._schedule_gripper_neutral()
        else:
            self._send_gripper_action(1)

    def _gripper_neutral(self) -> None:
        if bool(self.get_parameter("gripper_use_direct_servo").value):
            neutral_pwm = float(self.get_parameter("gripper_neutral_pwm").value)
            self._send_gripper_servo_pwm(neutral_pwm)
        else:
            self._send_gripper_action(2)

    def _send_gripper_servo_pwm(self, pwm: float) -> None:
        if not self.command_client.service_is_ready():
            self.get_logger().warn(
                "/mavros/cmd/command is not available; gripper servo command skipped"
            )
            return

        servo_number = int(self.get_parameter("gripper_servo_number").value)

        req = CommandLong.Request()
        req.broadcast = False
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(servo_number)
        req.param2 = float(pwm)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        self.get_logger().info(f"SERVO{servo_number} -> {pwm:.0f} us")

        future = self.command_client.call_async(req)
        future.add_done_callback(self._on_command_response)

    def _send_gripper_action(self, action: int) -> None:
        if not self.command_client.service_is_ready():
            self.get_logger().warn(
                "/mavros/cmd/command is not available; gripper command skipped"
            )
            return

        gripper_id = int(self.get_parameter("gripper_id").value)

        req = CommandLong.Request()
        req.broadcast = False
        req.command = 211  # MAV_CMD_DO_GRIPPER
        req.confirmation = 0
        req.param1 = float(gripper_id)
        req.param2 = float(action)  # 0=release, 1=grab, 2=hold
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self.command_client.call_async(req)
        future.add_done_callback(self._on_command_response)

    def _on_command_response(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Command exception: {exc}")
            return

        if result is None:
            self.get_logger().warn("Command returned no result")
            return

        if not result.success:
            self.get_logger().warn(f"Command failed: result={result.result}")

    def _schedule_gripper_neutral(self) -> None:
        self._cancel_gripper_neutral_timer()

        pulse_sec = float(self.get_parameter("gripper_pulse_sec").value)

        if pulse_sec <= 0.0:
            self._gripper_neutral()
            return

        self._gripper_neutral_timer = self.create_timer(
            pulse_sec,
            self._gripper_neutral_timer_callback,
        )

    def _gripper_neutral_timer_callback(self) -> None:
        timer = self._gripper_neutral_timer
        self._gripper_neutral_timer = None

        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

        self.get_logger().info("Gripper returning to neutral")
        self._gripper_neutral()

    def _cancel_gripper_neutral_timer(self) -> None:
        if self._gripper_neutral_timer is not None:
            self._gripper_neutral_timer.cancel()
            self.destroy_timer(self._gripper_neutral_timer)
            self._gripper_neutral_timer = None

def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToManualControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_neutral_once()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()