#!/usr/bin/env python3
"""Interactive command line for the motor node.

  ros2 run delta_motors motor_cmd

Commands:
  <id> <angle> [speed]   go to absolute angle, e.g.  1 45  or  1 45 20
  step <id> <deg> [speed] move relative,         e.g.  step 1 -10
  get                    print latest angles
  zero <id>              set current position as temporary zero
  stop [id]              emergency stop (no id = all motors)
  q                      quit
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, UInt8


class MotorCmd(Node):
    def __init__(self):
        super().__init__('motor_cmd')
        self.set_pub = self.create_publisher(Float32MultiArray, 'motor/set_angle', 10)
        self.step_pub = self.create_publisher(Float32MultiArray, 'motor/step_angle', 10)
        self.estop_pub = self.create_publisher(UInt8, 'motor/estop', 10)
        self.zero_pub = self.create_publisher(UInt8, 'motor/zero', 10)
        self.latest = {}
        self.create_subscription(JointState, 'motor/joint_states', self.on_state, 10)

    def on_state(self, msg):
        self.latest = dict(zip(msg.name, msg.position))

    def handle_line(self, line):
        parts = line.split()
        if not parts:
            return True
        cmd = parts[0].lower()
        try:
            if cmd in ('q', 'quit', 'exit'):
                return False
            if cmd == 'stop':
                self.estop_pub.publish(UInt8(data=int(parts[1]) if len(parts) > 1 else 0))
            elif cmd == 'zero':
                self.zero_pub.publish(UInt8(data=int(parts[1])))
            elif cmd == 'get':
                if not self.latest:
                    print('no feedback yet (is motor_node running?)')
                for name, pos in sorted(self.latest.items()):
                    print(f'  {name}: {pos:.2f} deg')
            elif cmd == 'step':
                self.step_pub.publish(Float32MultiArray(data=[float(x) for x in parts[1:4]]))
            else:
                self.set_pub.publish(Float32MultiArray(data=[float(x) for x in parts[:3]]))
        except (ValueError, IndexError):
            print('could not parse that. Type "help" for commands.')
        return True


def main():
    rclpy.init()
    node = MotorCmd()
    # spin in the background so feedback keeps arriving while we wait for input
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(__doc__)
    try:
        while True:
            line = input('motor> ')
            if line.strip() in ('help', '?'):
                print(__doc__)
                continue
            if not node.handle_line(line):
                break
    except (KeyboardInterrupt, EOFError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
