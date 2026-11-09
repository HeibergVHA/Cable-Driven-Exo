#!/usr/bin/env python3
"""Scope + FFT of /joint_states: last 10 s, SPACE pauses/resumes."""
import threading
from collections import defaultdict, deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from sensor_msgs.msg import JointState

WINDOW_S = 10.0


class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')
        self.data = defaultdict(deque)  # joint name -> deque of (time, angle)
        self.latest = 0.0
        self.create_subscription(JointState, 'joint_states', self.on_state, 10)

    def on_state(self, msg):
        # Sensor timestamp (more accurate than arrival time, which matters for the FFT)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        self.latest = t
        for name, pos in zip(msg.name, msg.position):
            buf = self.data[name]
            buf.append((t, pos))
            while buf[0][0] < t - WINDOW_S:
                buf.popleft()


def spectrum(points):
    """Amplitude spectrum (deg) of one joint, or None if there is too little data."""
    t = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])
    n = len(t)
    if n < 16 or t[-1] - t[0] < 1.0:
        return None
    y = np.rad2deg(np.unwrap(np.deg2rad(y)))     # remove the 360 -> 0 jumps
    tu = np.linspace(t[0], t[-1], n)             # resample to an even time grid
    y = np.interp(tu, t, y)
    fs = (n - 1) / (t[-1] - t[0])
    w = np.hanning(n)
    mag = np.abs(np.fft.rfft((y - y.mean()) * w)) * 2 / w.sum()
    return np.fft.rfftfreq(n, 1 / fs), mag


def main():
    rclpy.init()
    node = PlotNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, (ax, fx) = plt.subplots(2, 1, figsize=(10, 8))
    fig.subplots_adjust(hspace=0.35)
    paused = False

    def on_key(event):
        nonlocal paused
        if event.key == ' ':
            paused = not paused
    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_):
        if paused:
            return
        now = node.latest
        ax.cla()
        fx.cla()
        for name, buf in list(node.data.items()):
            points = list(buf)
            ax.plot([t - now for t, _ in points], [y for _, y in points],
                    linestyle='none', marker='.', markersize=3, label=name)
            result = spectrum(points)
            if result:
                fx.plot(*result, label=name)
        ax.set_xlim(-WINDOW_S, 0)
        ax.set_xlabel('seconds before now')
        ax.set_ylabel('angle [deg]')
        fx.set_xlabel('frequency [Hz]')
        fx.set_ylabel('amplitude [deg]')
        for a in (ax, fx):
            a.grid(True)
            if a.get_legend_handles_labels()[0]:
                a.legend(loc='upper right')

    ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()