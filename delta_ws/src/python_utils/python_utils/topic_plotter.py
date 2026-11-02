#!/usr/bin/env python3
"""Scope-style plot of /joint_states: last 10 s, SPACE pauses/resumes."""
import threading
from collections import defaultdict, deque

import matplotlib.pyplot as plt
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from sensor_msgs.msg import JointState

WINDOW_S = 10.0


class topic_plotter(Node):
    def __init__(self):
        super().__init__('topic_plotter')
        self.data = defaultdict(deque)  # joint name -> deque of (time, angle)
        self.create_subscription(JointState, 'joint_states', self.on_state, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        t = self.now()
        for name, pos in zip(msg.name, msg.position):
            buf = self.data[name]
            buf.append((t, pos))
            while buf[0][0] < t - WINDOW_S:  # keep only the visible window
                buf.popleft()


def main():
    rclpy.init()
    node = topic_plotter()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, ax = plt.subplots(figsize=(10, 5))
    paused = False

    def on_key(event):
        nonlocal paused
        if event.key == ' ':
            paused = not paused
    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_):
        if paused:
            return
        now = node.now()
        ax.cla()
        for name, buf in list(node.data.items()):
            points = list(buf)
            ax.plot([t - now for t, _ in points], [y for _, y in points], linestyle='none', marker='.', markersize=3, label=name)
        ax.set_xlim(-WINDOW_S, 0)
        ax.set_xlabel('seconds before now')
        ax.set_ylabel('angle [deg]')
        ax.grid(True)
        if node.data:
            ax.legend(loc='upper left')

    ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()