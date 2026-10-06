# delta_motors — ROS2 position control for DrEmpower motors

One node (`motor_node`) owns the serial/CAN link and accepts simple position commands.
Replaces the ROS1 `DrEmpower_can` package, which started ~50 separate nodes that all
shared one serial port.

## 1. Setup

Hardware: INNO-MAKER USB2CAN adapter (SocketCAN, shows up as `can0`) → CAN bus → PDA-04 joints, motors on 24 V.

```bash
mkdir -p ~/ros2_ws/src && cp -r delta_motors ~/ros2_ws/src/
cd ~/ros2_ws/src/delta_motors
bash setup_can.sh            # brings up can0 at 1 Mbit/s; run again after every replug/reboot
# bash setup_can.sh 500000   # if your motors are set to 500k
```

**Bitrate:** `can0` must run at the same CAN baud rate as the motors. If nothing answers, run
`candump can0` while sending a command: no reply frames usually means a bitrate mismatch,
so try `bash setup_can.sh 500000` (then 250000, 125000).

**Library files.** `delta_motors/DrEmpower.py` is Daran's library with English comments, adapted
(changes marked `[delta_motors]`):
- Daran's library expects their own serial USB-CAN module. `can_shim.py` stands in for that
  serial port and sends/receives real CAN frames on `can0` instead, so the protocol code is unchanged.
  (Setting `can_interface: ""` and `port: /dev/ttyACM0` switches back to Daran's module.)
- `rospy` removed; 0.5 s read timeout; a missing reply raises an error instead of `sys.exit()`.

**You still need `parameter_interface.py`** from the same Daran bundle (DrEmpower.py
imports it). Copy it into `delta_motors/delta_motors/` next to `DrEmpower.py`.

Build:

```bash
cd ~/ros2_ws && colcon build --packages-select delta_motors && source install/setup.bash
```

## 2. Configure

Edit `config/motors.yaml`: set `motor_ids` to your CAN IDs and **set `angle_min` /
`angle_max` to the safe range for the joint on the exoskeleton** before running on a person.
Rebuild after editing (or use `colcon build --symlink-install`).

## 3. Run

```bash
# Terminal 1 (add mock:=true to test without motors)
ros2 launch delta_motors motors.launch.py

# Terminal 2
ros2 run delta_motors motor_cmd
motor> 1 45          # motor 1 to 45°
motor> 1 0 20        # motor 1 to 0° at 20 r/min
motor> step 1 -10    # 10° back
motor> get           # read angles
motor> stop          # emergency stop all
```

Or from any terminal / your own node:

```bash
ros2 topic pub --once /motor/set_angle std_msgs/msg/Float32MultiArray "{data: [1, 45]}"
ros2 topic pub --once /motor/estop std_msgs/msg/UInt8 "{data: 0}"
ros2 topic echo /motor/joint_states
```

## Topics

| Topic | Type | Data |
|---|---|---|
| `/motor/set_angle` | Float32MultiArray | `[id, angle_deg]` or `[id, angle_deg, speed_rpm]` |
| `/motor/step_angle` | Float32MultiArray | `[id, delta_deg]` or `[id, delta_deg, speed_rpm]` |
| `/motor/estop` | UInt8 | id (0 = all) |
| `/motor/zero` | UInt8 | id — current position becomes temporary zero |
| `/motor/joint_states` | JointState | `position` in **degrees** (not radians) |

Notes:
- Motion uses DrEmpower trapezoidal mode (`mode=1`): `speed` = cruise speed, `accel` = acceleration.
- Every command is clamped to `[angle_min, angle_max]`, including relative steps.
- Unknown motor IDs and malformed messages are rejected and logged.
- Each feedback read talks to the bus; with many motors, lower `feedback_rate_hz` if commands feel laggy.
