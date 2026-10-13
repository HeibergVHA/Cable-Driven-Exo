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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
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
        # motor_ids and limits of the running motor_node, so we can warn here
        # instead of only in the node's terminal. None = node not found.
        self.ids, self.amin, self.amax = None, None, None

    def load_node_params(self, node_name='delta_motors', timeout=2.0):
        client = AsyncParameterClient(self, node_name)
        if not client.wait_for_services(timeout_sec=timeout):
            print(f'(could not reach /{node_name}: limits and ids will not be checked here)')
            return
        future = client.get_parameters(['motor_ids', 'angle_min', 'angle_max'])
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            return
        ids, amin, amax = future.result().values
        self.ids = list(ids.integer_array_value)
        self.amin, self.amax = amin.double_value, amax.double_value
        print(f'motor_node: motors {self.ids}, limits [{self.amin:g}, {self.amax:g}] deg\n')

    def check_target(self, id_num, target):
        """Print what motor_node will do with this command (it only logs it on its own side)."""
        if self.ids is None:
            return True
        if id_num not in self.ids:
            print(f'  ! motor {id_num} is not in motor_ids {self.ids}: the node will ignore this')
            return False
        if target is not None and not self.amin <= target <= self.amax:
            clamped = max(self.amin, min(self.amax, target))
            print(f'  ! {target:g} deg is outside the limits [{self.amin:g}, {self.amax:g}]: '
                  f'motor will stop at {clamped:g} deg (change angle_min/max in config/motors.yaml)')
        return True

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
                data = [float(x) for x in parts[1:4]]
                current = self.latest.get(f'motor_{int(data[0])}')
                self.check_target(int(data[0]), None if current is None else current + data[1])
                self.step_pub.publish(Float32MultiArray(data=data))
            else:
                data = [float(x) for x in parts[:3]]
                self.check_target(int(data[0]), data[1])
                self.set_pub.publish(Float32MultiArray(data=data))
        except (ValueError, IndexError):
            print('could not parse that. Type "help" for commands.')
        return True


def main():
    rclpy.init()
    node = MotorCmd()
    node.load_node_params()
    # spin in the background so feedback keeps arriving while we wait for input
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
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
    # stop the spin thread before destroying the node, otherwise rclpy aborts on exit
    executor.shutdown()
    spin_thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
