#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from interfaces.msg import StateChange as StateChangeMsg

import serial

# How long the arm's transient pick-status colors (grasp success/failure)
# hold before the LEDs revert to whatever the mobile-base state was already
# showing. Success is a quick double-flash (~150ms per half-cycle -> ~0.6s
# for 2 full flashes), held slightly past that for margin; failure flashes
# longer so it reads as a clear alert.
_PICK_SUCCESS_HOLD_SECONDS = 0.7
_PICK_FAILURE_HOLD_SECONDS = 2.0

_PICK_STATUS_COLOR = {
    "searching": b"S",  # pulsing PURPLE
    "approaching": b"A",  # chasing YELLOW
    "success": b"K",  # flashing GREEN (verified grasp -- own code, distinct from
                       # the mobile-base's solid-GREEN "driving" indicator ('G'))
    "failure": b"X",  # flashing RED
    # Sticky (no revert timer, same as searching/approaching) -- stays lit
    # until "idle" arrives, which manipulation_node publishes on
    # manipulator_remote_control's deactivation. Solid CYAN, deliberately
    # distinct from the mobile base's own solid-BLUE teleop indicator ('B')
    # so the operator can tell at a glance which one is under joystick
    # control right now.
    "remote_control_active": b"M",
}

# Priority order matters: gentle_stop can be active together with
# accept_remote_drive_commands (remote-control pause), and
# pause_autonomous_navigation is active during both teleop and a plain pause,
# so the more specific/urgent feature must be checked first.
_COLOR_BY_FEATURE_PRIORITY = (
    ("gentle_stop", b"E"),  # RED: safety stop (obstacle, e-stop from software)
    ("accept_remote_drive_commands", b"B"),  # BLUE: joystick/teleop has control
    ("command_movement", b"G"),  # GREEN: actively driving a mission
)


def _color_for_active_features(active_features):
    features = set(active_features)
    for feature_name, color in _COLOR_BY_FEATURE_PRIORITY:
        if feature_name in features:
            return color
    return b"Y"  # YELLOW: waiting for controller / paused / idle between orders


class ArduinoStatusIndicator(Node):
    """Drives the LED/speaker Arduino from the robot's actual state.

    Two jobs:
    - Tell it once the motor drives are ready, so "Ich bin bereit" plays when
      the robot is actually ready, not the moment it gets power.
    - Forward the mission_control state machine's active features as a single
      color byte, so the LEDs show what the robot is doing instead of a
      decorative animation.
    """

    def __init__(self):
        super().__init__("arduino_status_indicator")
        self.declare_parameter("connected_port", "/dev/ttyArduinoStatus")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("motors_ready_topic", "motors_ready")
        self.declare_parameter("motors_failed_topic", "motors_failed")
        self.declare_parameter("state_change_topic", "mission_control/state_change")
        self.declare_parameter("drives_unpowered_topic", "drives_unpowered")
        self.declare_parameter("pick_status_topic", "/manipulation/pick_status")

        port = str(self.get_parameter("connected_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        motors_ready_topic = str(self.get_parameter("motors_ready_topic").value)
        motors_failed_topic = str(self.get_parameter("motors_failed_topic").value)
        state_change_topic = str(self.get_parameter("state_change_topic").value)
        drives_unpowered_topic = str(self.get_parameter("drives_unpowered_topic").value)
        pick_status_topic = str(self.get_parameter("pick_status_topic").value)

        self._sent_ready = False
        self._sent_failed = False
        self._last_color = None
        self._drives_unpowered = False
        self._pick_revert_timer = None
        self._serial_connection = None
        try:
            self._serial_connection = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            # Opening the port toggles DTR, which resets this Arduino clone;
            # give the bootloader time to finish before writing, or the byte
            # is lost.
            time.sleep(2.0)
            self._serial_connection.write(b"P")  # pulse: motors booting
        except serial.SerialException as exc:
            self.get_logger().error(f"Could not open {port} for the status Arduino: {exc}")

        self.create_subscription(
            Bool,
            motors_ready_topic,
            self._on_motors_ready,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            Bool,
            motors_failed_topic,
            self._on_motors_failed,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            StateChangeMsg,
            state_change_topic,
            self._on_state_change,
            10,
        )
        self.create_subscription(
            Bool,
            drives_unpowered_topic,
            self._on_drives_unpowered,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            String,
            pick_status_topic,
            self._on_pick_status,
            10,
        )

    def _on_motors_ready(self, msg: Bool):
        if not msg.data or self._sent_ready or self._serial_connection is None:
            return
        self._serial_connection.write(b"R")
        self._serial_connection.write(b"G")  # static GREEN: motors ready
        self._sent_ready = True
        self.get_logger().info("Motors ready, told the status Arduino to announce it.")

    def _on_motors_failed(self, msg: Bool):
        if not msg.data or self._sent_failed or self._serial_connection is None:
            return
        self._serial_connection.write(b"F")  # static RED-ORANGE: boot failed
        self._sent_failed = True
        self.get_logger().error("Motor driver boot failed, told the status Arduino.")

    def _on_drives_unpowered(self, msg: Bool):
        if msg.data == self._drives_unpowered or self._serial_connection is None:
            return
        self._drives_unpowered = msg.data
        if msg.data:
            self._serial_connection.write(b"E")  # RED, held until power returns
            self.get_logger().error("Drives unpowered (emergency stop?) -- LEDs red.")
        else:
            # Repaint whatever mission_control last asked for; its state may
            # well have changed while we were holding red.
            self._serial_connection.write(self._last_color or b"Y")
            self.get_logger().info("Drives powered again -- LEDs restored.")

    def _on_pick_status(self, msg: String):
        if self._serial_connection is None:
            return

        if self._pick_revert_timer is not None:
            self._pick_revert_timer.cancel()
            self._pick_revert_timer = None

        if msg.data == "idle":
            # No color of its own -- just means "not moving right now",
            # so revert immediately (no flash/hold) to whatever the
            # mobile-base state already was. Used between motions (e.g.
            # after home_arm/place_object finish, or between a pick's
            # approach and its gripper-close) so "approaching" doesn't
            # stay stuck showing yellow when the arm isn't actually moving.
            if not self._drives_unpowered:
                self._serial_connection.write(self._last_color or b"Y")
            return

        color = _PICK_STATUS_COLOR.get(msg.data)
        if color is None:
            return

        if self._drives_unpowered:
            return  # e-stop red outranks everything, don't paint over it

        self._serial_connection.write(color)

        if msg.data in ("success", "failure"):
            hold = _PICK_SUCCESS_HOLD_SECONDS if msg.data == "success" else _PICK_FAILURE_HOLD_SECONDS
            self._pick_revert_timer = self.create_timer(hold, self._revert_after_pick_status)

    def _revert_after_pick_status(self):
        if self._pick_revert_timer is not None:
            self._pick_revert_timer.cancel()
            self._pick_revert_timer = None
        if self._serial_connection is None or self._drives_unpowered:
            return
        # Repaint whatever mission_control's mobile-base state was already
        # showing before the transient pick-status color took over.
        self._serial_connection.write(self._last_color or b"Y")

    def _on_state_change(self, msg: StateChangeMsg):
        if self._serial_connection is None:
            return
        color = _color_for_active_features(msg.active_features)
        if color == self._last_color:
            return
        self._last_color = color
        # Emergency stop outranks any mission state; remember the color above
        # so it can be restored on release, but don't paint over the red.
        if self._drives_unpowered:
            return
        self._serial_connection.write(color)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoStatusIndicator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
