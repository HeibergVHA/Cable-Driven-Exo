"""Fake DrEmpower module so you can test the ROS2 side without motors connected.

Implements only the functions motor_node uses. Motors "move" instantly.
"""

_angles = {}


def set_angle(id_num, angle, speed, param, mode):
    _angles[id_num] = float(angle)


def step_angle(id_num, angle, speed, param, mode):
    _angles[id_num] = _angles.get(id_num, 0.0) + float(angle)


def get_angle(id_num):
    return _angles.get(id_num, 0.0)


def estop(id_num):
    pass


def set_zero_position_temp(id_num):
    _angles[id_num] = 0.0
