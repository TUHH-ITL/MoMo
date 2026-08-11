# Mecanum Maxon control

This package converts ROS 2 `geometry_msgs/Twist` commands on `/cmd_vel` to four
Maxon EPOS4 profile-velocity targets through `ros2_canopen`.

Wheel order and CAN node mapping:

1. front-left (`cia402_device_1`)
2. front-right (`cia402_device_2`)
3. rear-left (`cia402_device_3`)
4. rear-right (`cia402_device_4`)

The checked-in controller configuration contains the dimensions, 21:1 gearbox
ratio, node order, and motor directions verified on this MoMo robot.
The motor command clamp is 3000 RPM (about 1.50 m/s straight-line speed), below
the 3170 RPM maximum profile velocity configured in all four EPOS4 DCF files.

## Build

The matching `ros2_canopen` source is vendored in this workspace at
`src/ros2_canopen`. No files outside this workspace are needed at build time or
runtime.

Install the native build and teleoperation dependencies once:

```bash
sudo apt update
sudo apt install -y libtool libtool-bin libboost-all-dev \
  ros-jazzy-diagnostic-updater ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

```bash
cd /home/itlbot2/ros2_jazzy
source /opt/ros/jazzy/setup.bash
# Overlay any packages already built in this workspace.
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install \
  --base-paths src/ros2_canopen src/mecanum_maxon_control \
  --packages-up-to mecanum_maxon_control \
  --cmake-clean-cache
source install/setup.bash
```

Source `/opt/ros/jazzy/setup.bash` first in every new terminal. Otherwise CMake
cannot discover system ROS packages such as `diagnostic_updater`.

## CAN and launch

### After every PC reboot

`can0` is now brought up automatically by the systemd unit checked in at
`systemd/can0-up.service` in this package. It is bound to the udev-generated
device unit for the Kvaser adapter (`sys-subsystem-net-devices-can0.device`),
so it runs on every boot and every hotplug/replug of the adapter, configuring
`can0` at 1 Mbit/s with `restart-ms 100` before anything else needs it. No
manual step is required before launching ROS 2.

Install it on a new PC with:

```bash
sudo cp systemd/can0-up.service /etc/systemd/system/can0-up.service
sudo systemctl daemon-reload
sudo systemctl enable --now can0-up.service
```

Check it came up correctly:

```bash
systemctl status can0-up.service
ip -details -statistics link show can0
```

The last command must show `can0` as `UP` and bitrate `1000000`. If `can0` does
not exist at all, verify the Kvaser USB adapter is connected
(`lsusb | grep -i kvaser`), then run `sudo modprobe kvaser_usb`.

To force a manual re-bounce of the interface (e.g. after a CAN bus fault),
either restart the unit or run the equivalent commands directly:

```bash
sudo systemctl restart can0-up.service
# equivalent to:
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

No separate EPOS4 initialization command is required. The launch file resets,
configures, starts, and enables all four CANopen drives. Once `can0` is up, launch:

```bash
source /opt/ros/jazzy/setup.bash
source /home/itlbot2/MoMo/install/setup.bash
ros2 launch mecanum_maxon_control mecanum_drive.launch.py
```

Wait for this message before publishing `/cmd_vel`:

```text
All four EPOS4 drives initialized in profile velocity mode
```

The CANopen container needs raw SocketCAN access (`CAP_NET_RAW`/`CAP_NET_ADMIN`),
granted via a file capability on the built binary rather than running the whole
launch as root:

```bash
sudo setcap cap_net_raw,cap_net_admin+ep build/canopen_core/device_container_node
```

Running everything as your normal user (instead of `sudo -E`) matters beyond
convenience: DDS discovery breaks across UIDs. FastDDS's shared-memory
transport creates separate `/dev/shm/fastrtps_*` segments per user, so
root-launched nodes and non-root `ros2` CLI tools (`ros2 node list`,
`ros2 topic echo`, `rqt`, etc.) silently can't see each other even though data
still flows fine *between* same-UID nodes.

**The capability must be re-applied after every rebuild** of `canopen_core`
(colcon overwrites the binary, wiping the capability bit). Run the `setcap`
command above again after `colcon build --symlink-install`, then verify with
`getcap build/canopen_core/device_container_node`.

File capabilities put the binary into the dynamic linker's "secure-execution
mode" (the same rule as setuid binaries), which makes `ld.so` **ignore
`LD_LIBRARY_PATH`** — so `device_container_node` can no longer find workspace
libraries (e.g. `libcanopen_interfaces__rosidl_generator_c.so`) through the
`LD_LIBRARY_PATH` that `install/setup.bash` sets. Fixed by registering every
built package's `lib/` dir with `ldconfig` instead, which isn't an env var and
works regardless of secure-execution mode:

```bash
find /home/itlbot2/MoMo/install -mindepth 2 -maxdepth 2 -type d -name lib | sort \
  | sudo tee /etc/ld.so.conf.d/momo-ros2.conf
sudo ldconfig
```

Re-run this whenever a **new package** (not just a rebuilt one) is added to
the workspace, so its `lib/` dir gets picked up too.

Initial PDO setup can take about 30 seconds; the `cmd_vel` controller starts
after a 45-second safety delay.

Test at low speed with all wheels clear of the floor:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0}, angular: {z: 0.0}}"
```

The controller latches the last velocity until a new command arrives. The joystick
teleoperation node sends a zero command when its enable button is released, so the
robot stops even though the underlying controller uses latched commands.

### Logitech F710 joystick

Connect the F710 USB receiver, put the switch on the front of the gamepad in `X`
(XInput) mode, and turn the gamepad on. Verify Linux can see it before launching:

```bash
lsusb | grep -i logitech
ls -l /dev/input/js*
```

The configuration is taken from the original MoMo backup:

- Hold **LB** (button 4) for normal driving (up to 1.0 m/s and 1.5 rad/s).
- Hold **RB** (button 5) for turbo driving (up to 1.5 m/s and 2.0 rad/s).
- Left stick up/down controls forward/reverse (`linear.x`).
- Left stick left/right controls mecanum strafing (`linear.y`).
- Right stick left/right controls rotation (`angular.z`).
- Release LB or RB to stop.

The launch file starts CANopen motor control, the joystick driver, and the joystick
to `/cmd_vel` converter together. Run it instead of `mecanum_drive.launch.py`:

```bash
source /opt/ros/jazzy/setup.bash
source /home/itlbot2/MoMo/install/setup.bash
ros2 launch mecanum_maxon_control mecanum_teleop.launch.py
```

Wait for `All four EPOS4 drives initialized in profile velocity mode` before
driving. If `/dev/input/js0` is absent, reconnect the receiver, confirm the pad is
in X mode, and press a gamepad button. The ROS 2 joystick driver selects joystick
device 0 by default.

### Changing joystick speed

Joystick speed is controlled in
`mecanum_maxon_control/config/teleop_joy.yaml`:

- `scale_linear.x` and `scale_linear.y` are the LB translation limits in m/s.
- `scale_angular.yaw` is the LB rotation limit in rad/s.
- `scale_linear_turbo.x` and `scale_linear_turbo.y` are the RB turbo
  translation limits in m/s.
- `scale_angular_turbo.yaw` is the RB turbo rotation limit in rad/s.

The controller also applies a final motor-speed ceiling using `max_motor_rpm` in
`mecanum_maxon_control/config/controller.yaml`. With the configured 21:1 gearbox
and 0.10 m wheel radius, 1000 motor RPM is approximately 0.50 m/s and the current
3000 RPM limit is approximately 1.50 m/s. Increasing only the joystick scales
above that limit will not make the wheels turn faster.

Do not set `max_motor_rpm` above the EPOS4 `max profile velocity` of 3170 RPM
configured in the four motor DCF files. After changing either YAML file, rebuild
and restart the launch file:

```bash
cd /home/itlbot2/ros2_jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select mecanum_maxon_control
source install/setup.bash
```

Acceleration and stopping response are controlled by `max_wheel_acceleration`
and `max_wheel_deceleration` in `config/controller.yaml`, with the corresponding
EPOS profile objects `0x6083` and `0x6084` in
`config/maxon_mecanum_bus/bus.yml`. Both currently correspond to approximately
10000 motor RPM/s, giving approximately 0.3 seconds to reach maximum speed or
stop from it. Keep each software limit consistent with its EPOS limit when
tuning the response.

These 10000 RPM/s acceleration and deceleration values were tested on the MoMo
robot and produced responsive driving without the previous slow takeoff or the
wheels continuing to spin after releasing the joystick. For reference, the old
2000 RPM/s values took approximately 1.5 seconds to accelerate to or stop from
the 3000 RPM maximum. Test future changes with the wheels clear of the floor
before driving on the ground.

For keyboard control, leave `mecanum_drive.launch.py` running and use:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use `k` or the space bar to send zero velocity and stop the robot.

Mecanum strafing uses `linear.y`; forward/reverse uses `linear.x`; rotation uses
`angular.z`.

## MoMo drivetrain calibration

The controller uses the dimensions and conversion from the original MoMo ROS 1
configuration: 0.10 m wheel radius, 0.54 m wheelbase, 0.58 m track width, and a
21:1 motor-to-wheel reduction. CANopen nodes 1 through 4 correspond to
front-left, front-right, rear-left, and rear-right respectively.
