#!/usr/bin/env python3
"""One ROS2 node that owns the serial/CAN link to all DrEmpower motors.

Topics in (commands):
  /motor/set_angle   std_msgs/Float32MultiArray  [id, angle_deg]  or [id, angle_deg, speed_rpm]
  /motor/step_angle  std_msgs/Float32MultiArray  [id, delta_deg]  or [id, delta_deg, speed_rpm]
  /motor/estop       std_msgs/UInt8              id   (0 = all motors on the bus)
  /motor/zero        std_msgs/UInt8              id   (temporary zero, lost on power-off)

Topic out (feedback):
  /motor/joint_states  sensor_msgs/JointState    position in degrees for each id in motor_ids
"""

import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, UInt8


class MotorNode(Node):
    def __init__(self):
        super().__init__('delta_motors')

        # ---- parameters (override in config/motors.yaml) ----
        self.declare_parameter('motor_ids', [1])
        self.declare_parameter('default_speed', 10.0)   # r/min at output shaft
        self.declare_parameter('accel', 10.0)           # r/min/s (trapezoidal mode 'param')
        self.declare_parameter('angle_min', -90.0)      # software safety limits [deg]
        self.declare_parameter('angle_max', 90.0)
        self.declare_parameter('feedback_rate_hz', 10.0)  # 0 disables feedback polling
        self.declare_parameter('mock', False)           # True = no hardware needed
        self.declare_parameter('can_interface', 'can0') # SocketCAN adapter; '' = use serial 'port'
        self.declare_parameter('port', '')              # Daran serial module only; '' = first /dev/ttyACM*

        p = lambda n: self.get_parameter(n).value
        self.ids = list(p('motor_ids'))
        self.default_speed = float(p('default_speed'))
        self.accel = float(p('accel'))
        self.amin, self.amax = float(p('angle_min')), float(p('angle_max'))

        if p('mock'):
            from delta_motors import mock_dr as dr
            self.get_logger().warn('MOCK mode: no motors are being driven')
        else:
            if p('can_interface'):
                os.environ['DREMPOWER_CAN'] = p('can_interface')
            elif p('port'):
                os.environ['DREMPOWER_PORT'] = p('port')
            try:
                # DrEmpower opens the serial port on import. Look inside this
                # package first, then anywhere on PYTHONPATH.
                try:
                    from delta_motors import DrEmpower as dr
                except ImportError:
                    import DrEmpower as dr
            except Exception as e:
                self.get_logger().fatal(
                    f'Could not open DrEmpower ({e}). Is the adapter plugged in and '
                    'can0 up (sudo ip link set can0 up type can bitrate 1000000)? See README.')
                raise
        self.dr = dr

        # Serial link is not thread-safe: every dr.* call goes through this lock
        self.lock = threading.Lock()
        self.last_target = {}

        self.create_subscription(Float32MultiArray, 'motor/set_angle', self.on_set_angle, 10)
        self.create_subscription(Float32MultiArray, 'motor/step_angle', self.on_step_angle, 10)
        self.create_subscription(UInt8, 'motor/estop', self.on_estop, 10)
        self.create_subscription(UInt8, 'motor/zero', self.on_zero, 10)
        self.state_pub = self.create_publisher(JointState, 'motor/joint_states', 10)

        rate = float(p('feedback_rate_hz'))
        if rate > 0:
            self.create_timer(1.0 / rate, self.publish_state)

        self.get_logger().info(
            f'Ready. motors={self.ids} limits=[{self.amin}, {self.amax}] deg '
            f'speed={self.default_speed} r/min')

    # ---------- helpers ----------
    def _call(self, fn, **kw):
        """Run one DrEmpower call under the lock. Returns None on any failure.

        The library prints and returns False on errors (and can raise), so
        both are mapped to None here and the node keeps running.
        """
        with self.lock:
            try:
                r = getattr(self.dr, fn)(**kw)
            except Exception as e:
                self.get_logger().error(f'{fn}({kw}) failed: {e}')
                return None
        return None if r is False else r

    def _parse(self, msg, name):
        d = list(msg.data)
        if len(d) not in (2, 3):
            self.get_logger().error(f'{name}: expected [id, angle] or [id, angle, speed], got {d}')
            return None
        id_num = int(d[0])
        if id_num not in self.ids:
            self.get_logger().error(f'{name}: id {id_num} not in motor_ids {self.ids}')
            return None
        speed = float(d[2]) if len(d) == 3 else self.default_speed
        if speed <= 0:
            self.get_logger().error(f'{name}: speed must be > 0')
            return None
        return id_num, float(d[1]), speed

    def _clamp(self, angle):
        c = max(self.amin, min(self.amax, angle))
        if c != angle:
            self.get_logger().warn(f'angle {angle:.1f} clamped to {c:.1f} (limits)')
        return c

    # ---------- callbacks ----------
    def on_set_angle(self, msg):
        parsed = self._parse(msg, 'set_angle')
        if not parsed:
            return
        id_num, angle, speed = parsed
        angle = self._clamp(angle)
        # mode=1: trapezoidal profile, speed = cruise speed, param = acceleration
        self._call('set_angle', id_num=id_num, angle=angle, speed=speed, param=self.accel, mode=1)
        self.last_target[id_num] = angle
        self.get_logger().info(f'motor {id_num} -> {angle:.1f} deg @ {speed} r/min')

    def on_step_angle(self, msg):
        parsed = self._parse(msg, 'step_angle')
        if not parsed:
            return
        id_num, delta, speed = parsed
        current = self._call('get_angle', id_num=id_num)
        if current is None:
            self.get_logger().error(f'step_angle: could not read angle of motor {id_num}')
            return
        # Turn the step into an absolute target so the safety limits still apply
        target = self._clamp(current + delta)
        self._call('set_angle', id_num=id_num, angle=target, speed=speed, param=self.accel, mode=1)
        self.last_target[id_num] = target
        self.get_logger().info(f'motor {id_num} step {delta:+.1f} -> {target:.1f} deg')

    def on_estop(self, msg):
        self._call('estop', id_num=int(msg.data))
        self.get_logger().warn(f'ESTOP motor {msg.data} (0 = all)')

    def on_zero(self, msg):
        self._call('set_zero_position_temp', id_num=int(msg.data))
        self.get_logger().info(f'motor {msg.data}: current position set as temporary zero')

    def publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for id_num in self.ids:
            a = self._call('get_angle', id_num=id_num)
            if a is None:
                continue  # read failed this cycle, skip
            js.name.append(f'motor_{id_num}')
            js.position.append(float(a))  # degrees
        if js.name:
            self.state_pub.publish(js)


def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
