#!/usr/bin/env python3
import math
import socket
import struct
import threading
import time

import rclpy
from canopen_interfaces.srv import COTargetDouble
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class MecanumEpos4Controller(Node):
    def __init__(self):
        super().__init__('mecanum_epos4_controller')
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('wheelbase', 0.40)
        self.declare_parameter('track_width', 0.35)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('motor_directions', [1.0, -1.0, 1.0, -1.0])
        self.declare_parameter('max_motor_rpm', 1000.0)
        self.declare_parameter('max_linear_velocity', 4.0)
        self.declare_parameter('max_angular_velocity', 3.0)
        self.declare_parameter('max_wheel_acceleration', 8.0)
        self.declare_parameter('max_wheel_deceleration', 8.0)
        self.declare_parameter('command_timeout', 0.30)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('service_timeout', 8.0)
        self.declare_parameter('startup_retries', 3)
        self.declare_parameter('direct_can_velocity', True)
        self.declare_parameter('can_interface', 'can0')

        self.radius = float(self.get_parameter('wheel_radius').value)
        self.k = 0.5 * (float(self.get_parameter('wheelbase').value) +
                        float(self.get_parameter('track_width').value))
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.directions = list(self.get_parameter('motor_directions').value)
        self.max_rpm = float(self.get_parameter('max_motor_rpm').value)
        self.max_linear_velocity = float(
            self.get_parameter('max_linear_velocity').value)
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value)
        self.max_accel = float(self.get_parameter('max_wheel_acceleration').value)
        self.max_decel = float(self.get_parameter('max_wheel_deceleration').value)
        self.timeout = float(self.get_parameter('command_timeout').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.service_timeout = float(self.get_parameter('service_timeout').value)
        self.startup_retries = int(self.get_parameter('startup_retries').value)
        self.direct_can_velocity = bool(
            self.get_parameter('direct_can_velocity').value)
        self.can_interface = str(self.get_parameter('can_interface').value)
        if self.radius <= 0.0 or self.gear_ratio <= 0.0 or len(self.directions) != 4:
            raise ValueError('wheel_radius and gear_ratio must be positive; motor_directions needs 4 entries')

        self.targets = [0.0] * 4
        self.outputs = [0.0] * 4
        self.last_cmd = self.get_clock().now()
        self.ready = False
        self.pending = [False] * 4
        self.last_sent = [None] * 4
        self.service_targets = [None] * 4
        self.target_clients = []
        self.trigger_clients = []
        self.can_socket = None
        if self.direct_can_velocity:
            try:
                self.can_socket = socket.socket(
                    socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.can_socket.bind((self.can_interface,))
                self.get_logger().info(
                    f'Direct velocity RPDO output enabled on {self.can_interface}')
            except OSError as exc:
                self.get_logger().error(
                    f'Cannot open {self.can_interface} for direct velocity output: {exc}; '
                    'falling back to ROS target services')
        for index in range(1, 5):
            prefix = f'/cia402_device_{index}'
            self.target_clients.append(self.create_client(COTargetDouble, prefix + '/target'))
            self.trigger_clients.append({
                'init': self.create_client(Trigger, prefix + '/init'),
                'velocity_mode': self.create_client(Trigger, prefix + '/velocity_mode'),
                'halt': self.create_client(Trigger, prefix + '/halt'),
            })

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(1.0 / self.rate, self.control_loop)
        if bool(self.get_parameter('auto_start').value):
            threading.Thread(target=self.start_drives, daemon=True).start()
        else:
            self.ready = True
        self.get_logger().info('Mecanum controller waiting for four EPOS4 drives')

    def cmd_vel_callback(self, msg):
        vx = max(-self.max_linear_velocity,
                 min(self.max_linear_velocity, msg.linear.x))
        vy = max(-self.max_linear_velocity,
                 min(self.max_linear_velocity, msg.linear.y))
        wz = max(-self.max_angular_velocity,
                 min(self.max_angular_velocity, msg.angular.z))
        raw = [
            (vx - vy - self.k * wz) / self.radius,
            (vx + vy + self.k * wz) / self.radius,
            (vx + vy - self.k * wz) / self.radius,
            (vx - vy + self.k * wz) / self.radius,
        ]
        raw = [raw[i] * self.gear_ratio * self.directions[i] for i in range(4)]
        rpm_factor = 60.0 / (2.0 * math.pi)
        peak_rpm = max(abs(value) * rpm_factor for value in raw)
        if peak_rpm > self.max_rpm:
            scale = self.max_rpm / peak_rpm
            raw = [value * scale for value in raw]
        self.targets = raw
        self.last_cmd = self.get_clock().now()

    def start_drives(self):
        for operation in ('init', 'velocity_mode'):
            for i, clients in enumerate(self.trigger_clients):
                client = clients[operation]
                if not client.wait_for_service(timeout_sec=self.service_timeout):
                    self.get_logger().error(f'Drive {i + 1} {operation} service unavailable')
                    return
                succeeded = False
                for attempt in range(1, self.startup_retries + 1):
                    future = client.call_async(Trigger.Request())
                    deadline = time.monotonic() + self.service_timeout
                    while not future.done() and time.monotonic() < deadline:
                        time.sleep(0.02)
                    succeeded = (
                        future.done() and future.result() is not None and
                        future.result().success
                    )
                    if succeeded:
                        break
                    self.get_logger().warning(
                        f'Drive {i + 1} failed {operation} attempt '
                        f'{attempt}/{self.startup_retries}')
                    time.sleep(1.0)
                if not succeeded:
                    self.get_logger().error(f'Drive {i + 1} failed {operation}')
                    return
        self.ready = True
        self.get_logger().info('All four EPOS4 drives initialized in profile velocity mode')

    def control_loop(self):
        if not self.ready:
            return
        stale = (
            self.timeout > 0.0 and
            (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.timeout
        )
        desired = [0.0] * 4 if stale else self.targets
        for i in range(4):
            slowing_down = (
                abs(desired[i]) < abs(self.outputs[i]) or
                desired[i] * self.outputs[i] < 0.0
            )
            limit = self.max_decel if slowing_down else self.max_accel
            step = limit / self.rate
            delta = max(-step, min(step, desired[i] - self.outputs[i]))
            self.outputs[i] += delta
            changed = (self.last_sent[i] is None or
                       abs(self.outputs[i] - self.last_sent[i]) >= 1e-6)

            # Keep transmitting the direct RPDO. The legacy CANopen driver
            # periodically transmits its cached RPDO too, so a one-shot direct
            # command would be overwritten by the old cached target.
            if self.can_socket is not None:
                self._send_velocity_rpdo(i, self.outputs[i])
                self.last_sent[i] = self.outputs[i]

            if self.pending[i]:
                continue
            service_changed = (
                self.service_targets[i] is None or
                abs(self.outputs[i] - self.service_targets[i]) >= 1e-6
            )
            if not service_changed:
                continue
            if self.can_socket is None and not changed:
                continue
            request = COTargetDouble.Request()
            request.target = self.outputs[i]
            self.last_sent[i] = self.outputs[i]
            self.service_targets[i] = self.outputs[i]
            self.pending[i] = True
            future = self.target_clients[i].call_async(request)
            future.add_done_callback(
                lambda _future, wheel=i: self._target_done(wheel, _future))

    def _send_velocity_rpdo(self, wheel, target_rad_s):
        """Send controlword, profile velocity and mode through configured RPDO1."""
        target_rpm = int(round(target_rad_s * 60.0 / (2.0 * math.pi)))
        target_rpm = max(-(2 ** 31), min(2 ** 31 - 1, target_rpm))
        payload = struct.pack('<Hib', 0x000F, target_rpm, 3)
        frame = struct.pack('=IB3x8s', 0x201 + wheel, len(payload), payload)
        try:
            self.can_socket.send(frame)
        except OSError as exc:
            self.get_logger().error(
                f'Drive {wheel + 1} direct CAN command failed: {exc}',
                throttle_duration_sec=2.0)

    def _target_done(self, wheel, future):
        self.pending[wheel] = False
        try:
            if not future.result().success:
                self.service_targets[wheel] = None
                self.get_logger().warn(f'Drive {wheel + 1} rejected velocity target',
                                       throttle_duration_sec=2.0)
        except Exception as exc:
            self.service_targets[wheel] = None
            self.get_logger().error(f'Drive {wheel + 1} target call failed: {exc}')

    def stop_drives(self):
        """Best-effort zero command followed by CiA 402 halt during clean shutdown."""
        if self.can_socket is not None:
            for wheel in range(4):
                self._send_velocity_rpdo(wheel, 0.0)
        for client in self.target_clients:
            if client.service_is_ready():
                request = COTargetDouble.Request()
                request.target = 0.0
                rclpy.spin_until_future_complete(self, client.call_async(request), timeout_sec=0.5)
        for clients in self.trigger_clients:
            client = clients['halt']
            if client.service_is_ready():
                rclpy.spin_until_future_complete(
                    self, client.call_async(Trigger.Request()), timeout_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumEpos4Controller()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        if rclpy.ok():
            node.stop_drives()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
