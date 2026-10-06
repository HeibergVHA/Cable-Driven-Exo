# -*- coding=utf-8 -*-

"""
Daran Robotics - smart integrated joint function library

Platforms: Windows, Linux (Jetson Nano, Raspberry Pi)
Language: Python 3
Communication hardware: USB-to-CAN module

****************************Set joint ID*************************
                          set_id: Set the joint ID number.

****************************Motion control functions*************************
                       set_angle: Absolute angle control of a single joint.
                      set_angles: Absolute angle control of multiple joints.
                      step_angle: Relative angle control of a single joint.
                     step_angles: Relative angle control of multiple joints.
              set_angle_adaptive: Adaptive angle control of a single joint.
             set_angles_adaptive: Adaptive angle control of multiple joints.
               impedance_control: Impedance control of a single joint.
         impedance_control_multi: Impedance control of multiple joints.
               motion_follow_aid: Motion following and assist of a single joint.
         motion_follow_aid_multi: Motion following and assist of multiple joints.
                   position_done: Check and wait until a single joint reaches its target angle.
                  positions_done: Check and wait until multiple joints reach their target angles.
                       set_speed: Speed control of a single joint.
                      set_speeds: Speed control of multiple joints.
                      set_torque: Torque control of a single joint.
                     set_torques: Torque control of multiple joints.
                           estop: Emergency stop.

****************************Parameter read-back functions************************
                          get_id: Read back the joint ID number.
                         get_ids: Read back the ID numbers of multiple joints on the bus
                       get_angle: Read back the joint's current angle.
                       get_speed: Read back the joint's current speed.
                       get_state: Read back the current angle and speed at the same time
                      get_torque: Read back the joint's current output torque.
                     get_vol_cur: Read back the bus voltage and FOC q-axis current.
 enable_angle_speed_torque_state: Enable real-time angle, speed and torque feedback
      set_state_feedback_rate_ms: Set the real-time angle/speed/torque feedback interval
        angle_speed_torque_state: Real-time angle, speed and torque feedback
disable_angle_speed_torque_state: Disable real-time angle, speed and torque feedback
                         get_pid: Read back the control loop position gain P, integral gain I and speed gain D.
                   read_property: Read back a configuration parameter.

****************************Parameter setting functions************************
               set_zero_position: Set the joint's current angle as zero; kept after power-off/restart.
          set_zero_position_temp: Set the joint's current angle as zero (temporary); lost after power-off/restart (not available on hollow-shaft joints).
                 set_angle_range: Set joint angle limits for this run; lost after power-off or restart.
             disable_angle_range: Disable the joint angle limits for this run.
          set_angle_range_config: Set the joint angle limit property; kept after power-off or restart.
      disable_angle_range_config: Disable the joint angle limit property; kept after power-off or restart.
                 set_speed_limit: Set the speed limit for this run
                set_torque_limit: Set the torque limit for this run
              set_speed_adaptive: Set the adaptive speed
             set_torque_adaptive: Set the adaptive torque
                         set_pid: Set the control loop position gain P, speed gain D and integral gain I for this run; lost after power-off or restart.
                        set_mode: Set the joint mode: idle mode or closed-loop mode.
               set_can_baud_rate: Set the CAN communication baud rate.
                  write_property: Write a joint configuration parameter.
                     save_config: Save the joint configuration parameters.

****************************Helper functions************************
                      dump_error: Print joint errors.
                          reboot: Reboot the joint.
                     init_config: Restore factory settings.
                    erase_config: Erase the configuration, recalibrate and restore factory settings
"""
import sys
import time
try:
    from delta_motors import *  # ROS2 package layout
except ImportError:
    from parameter_interface import *
import math as cm
import struct
import os

uart_baudrate = 115200 # Serial baud rate, must match the CAN module's serial baud rate (factory default 115200, max 460800)


# [delta_motors] Transport selection (replaces rospy.get_param('device_paths')).
#   DREMPOWER_CAN=can0   -> SocketCAN adapter (e.g. INNO-MAKER USB2CAN) via python-can
#   DREMPOWER_PORT=/dev/ttyACM0 -> Daran's own serial USB-CAN module (original behaviour)
# motor_node sets these from its 'can_interface' / 'port' parameters.
if os.environ.get('DREMPOWER_CAN'):
    try:
        from delta_motors import CanSerialShim
    except ImportError:
        from can_shim import CanSerialShim
    com = os.environ['DREMPOWER_CAN']
    uart = CanSerialShim(channel=com, timeout=0.5)
else:
    import serial
    import glob as _glob
    com = os.environ.get('DREMPOWER_PORT') or (sorted(_glob.glob('/dev/ttyACM*')) or [None])[0]
    if com is None:
        raise IOError('No /dev/ttyACM* found and DREMPOWER_CAN not set')
    uart = serial.Serial(com, uart_baudrate, timeout=0.5)  # [delta_motors] added timeout

READ_FLAG = 0  # Read result flag
cur_angle_list = []  # Current angle list
set_angles_mode_1_flag = 0
angle_speed_torque_state_flag = 0
estop_flag = 0

"""
Functional functions, for users
"""

# Set the joint ID number
def set_id(id_num=0, new_id=0):
    """Set the joint ID number.

    Changes the joint ID number. Set once (kept after power-off).
    Notes:
        (1) Give each joint a unique ID before use, so that no two joints on the bus share an ID, which would cause communication errors.
        (2) Best used before the joint is put into service, so the control target is defined in advance.
        (3) Recommended to run with no load, otherwise the joint may briefly unload.

    Args:
        id_num: ID of the joint whose ID should be changed. If the current ID is unknown, 0 can be used to broadcast,
                but then only one joint may be connected to the bus, otherwise all joints will get the same ID.
        new_id: New joint ID, an integer in the range 1~64.

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        "---error in set_id---"

    """
    try:
        write_property(id_num=id_num, property='dr.config.can_id', value=new_id)
        if read_property(id_num=id_num, property='dr.config.can_id') == new_id:
            save_config(id_num=new_id)
            print("ID 编号成功修改为：", new_id)
        else:
            print("新 ID 编号设置失败")
            return False
    except Exception as e:
        print("---error in set_id---：", e)
        return False
    return True

# Set the current angle as the mechanical zero point; persists after power-off/restart
def set_zero_position(id_num=0):
    """Set the joint zero-point angle.

    Sets the current angle as the zero point of the joint output shaft. Afterwards the current angle is 0 degrees; kept after restart.
    Notes:
        (1) Best used before changing other parameters, because this function calls save_config(), which saves all other parameters too.
        (2) Recommended to run with no load, otherwise the joint may briefly unload.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        "---error in set_zero_position---"

    """
    try:
        order_num = 0x05
        data = format_data([order_num], 'u32', 'encode')
        send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        save_config(id_num=id_num)
        print("零点设置成功！")
    except Exception as e:
        print("---error in set_zero_position---：", e)
        return False
    return True

# Absolute angle control
def set_angle(id_num=0, angle=0, speed=0, param=0, mode=0):
    """Angle control of a single joint.

    Moves the joint with the given ID to the given angle at the given speed (absolute angle, relative to the user-defined zero point).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
         angle: Absolute joint angle (deg).
         speed: Joint speed (r/min); meaning depends on mode:
                mode=1: target speed;
                mode=0/2: feedforward speed (r/min).
         param: Motion parameter, depends on mode:
                mode=0: angle input filter bandwidth (<300);
                mode=1: acceleration during start and stop phases (r/min/s);
                mode=2: feedforward torque (Nm).
          mode: Angle control mode. The joint supports three angle control modes, selected by mode:
                mode=0: trajectory tracking mode, suited to smooth control over many input trajectory points; the angle input filter bandwidth should be set to half the command sending frequency;
                mode=1: trapezoidal trajectory mode; the target speed during motion and the start/stop acceleration can be specified;
                mode=2: feedforward control mode; speed and torque are feedforward terms. Feedforward adds speed and torque feedforward on top of the PID control,
                          improving response and reducing steady-state error.

    Note: In mode=1 (trapezoidal), speed and accel must both be > 0; in mode=0 speed has no effect.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in set_angle---"

    """
    factor = 0.01
    try:
        if mode == 0:
            f_angle = angle
            s16_speed = int((abs(speed)) / factor)
            if param > 300:
                print("input_filter_width = " + str(param) + ", which is too big and resized to 300")
                param = 300
            s16_width = int(abs(param / factor))
            data = format_data([f_angle, s16_speed, s16_width], 'f s16 s16', 'encode')
            send_command(id_num=id_num, cmd=0x19, data=data, rtr=0)
        elif mode == 1:
            if speed > 0 and param > 0:
                f_angle = angle
                s16_speed = int((abs(speed)) / factor)
                s16_accel = int((abs(param)) / factor)
                data = format_data([f_angle, s16_speed, s16_accel], 'f s16 s16', 'encode')
                send_command(id_num=id_num, cmd=0x1A, data=data, rtr=0)
            else:
                print("speed or accel <= 0")
        elif mode == 2:
            f_angle = angle
            s16_speed_ff = int((speed) / factor)
            s16_torque_ff = int((param) / factor)
            data = format_data([f_angle, s16_speed_ff, s16_torque_ff], 'f s16 s16', 'encode')
            send_command(id_num=id_num, cmd=0x1B, data=data, rtr=0)
    except Exception as e:
        print("---error in set_angle---：", e)
        return False
    return True

# Absolute angle control of multiple joints
def set_angles(id_list=[1, 2, 3], angle_list=[0, 0, 0], speed=10, param=10, mode=1):
    """Control of multiple joints.

    Moves multiple joints to the given angles at a given speed.

    Args:
           id_list: List of joint IDs.
        angle_list: List of target angles (deg).
             speed: Speed (r/min), depends on mode:
                    mode=1: speed of the fastest of the joints;
                    mode=0/2: feedforward speed.
             param: Motion parameter, depends on mode:
                    mode=0: angle input filter bandwidth (<300);
                    mode=1: acceleration during start and stop phases (r/min/s);
                    mode=2: feedforward torque (Nm).
              mode: Control mode. The joints support three angle control modes, selected by mode:
                    mode=0: multi-joint trajectory tracking mode, suited to smooth control over many input trajectory points; the angle input bandwidth should be set to half the command sending frequency;
                    mode=1: multi-joint trapezoidal trajectory mode; speed is the fastest speed of these joints (r/min), param is the target acceleration (r/min/s);
                    mode=2: feedforward control mode; speed and torque are feedforward terms. Feedforward adds speed and torque feedforward on top of the PID control,
                            improving response and reducing steady-state error.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        A message is shown if id_list and angle_list have different lengths.
        "---error in set_angles---"

    """
    global cur_angle_list
    global set_angles_mode_1_flag
    global estop_flag
    try:
        if len(id_list) == len(angle_list):
            if mode == 0:
                for i in range(len(id_list)):
                    preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param, mode=mode)
                order_num = 0x10
                data = format_data([order_num, 0], 'u32 u16', 'encode')
                send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
            elif mode == 1:
                if len(cur_angle_list) != len(angle_list) or set_angles_mode_1_flag == 0 or estop_flag == 1:
                    cur_angle_list = []
                    if angle_speed_torque_state_flag == 0:
                        for i in range(len(id_list)):
                            state = get_angle(id_num=id_list[i])
                            if READ_FLAG == 1:
                                cur_angle_list.append(state)
                            else:
                                cur_angle_list.append(0)
                    else:
                        angle_speed_torque_states(id_list=id_list)  # Clear leftover data from the previous run
                        angle_speed_torques = angle_speed_torque_states(id_list=id_list)
                        for i in range(len(id_list)):
                            cur_angle_list.append(angle_speed_torques[i][0])
                if speed > 0 and param > 0:
                    for i in range(len(angle_list)):
                        DETA_angle_list = list(
                            map(lambda x: abs(x[0] - x[1]),
                                zip(angle_list, cur_angle_list)))
                    delta_angle = max(DETA_angle_list)
                    # print(delta_angle)
                    if delta_angle <= (6 * speed * speed / abs(param)):  # Maximum relative angle is smaller than the angle covered by a full acceleration + deceleration
                        t = 2 * cm.sqrt(delta_angle / (6 * abs(param)))  # Time needed for an isosceles triangle (speed) trajectory
                    else:
                        t = speed / abs(param) + delta_angle / (6 * speed)  # Time needed to complete the full trapezoidal trajectory
                    for i in range(len(id_list)):
                        preset_angle(id_num=id_list[i], angle=angle_list[i], t=t, param=param, mode=mode)
                    order_num = 0x11
                    data = format_data([order_num, 0], 'u32 u16', 'encode')  # Convert order_num into 4 bytes of data,
                    send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
                else:
                    print("speed or accel <= 0")
                set_angles_mode_1_flag = 1
            elif mode == 2:
                for i in range(len(id_list)):
                    preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param, mode=mode)
                order_num = 0x12
                data = format_data([order_num, 0], 'u32 u16', 'encode')
                send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
            cur_angle_list = angle_list.copy()
        else:
            print("id_list 和 angle_list 长度不一致")
    except Exception as e:
        print("---error in set_angles---：", e)
        return False
    return True

# Relative angle control
def step_angle(id_num=1, angle=0, speed=0, param=0, mode=0):
    """Relative angle control of a single joint.

    Rotates the joint with the given ID by the given angle at the given speed (relative angle, relative to the angle when the command is sent).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
         angle: Relative joint angle (deg).
         speed: Speed (r/min), depends on mode:
                mode=1: target speed;
                mode=0/2: feedforward speed (r/min).
         param: Motion parameter, depends on mode:
                mode=0: angle input filter bandwidth (<300);
                mode=1: acceleration during start and stop phases (r/min/s);
                mode=2: feedforward torque (Nm).
          mode: Angle control mode. The joint supports three angle control modes, selected by mode:
                mode=0: trajectory tracking mode, suited to smooth control over many input trajectory points; the angle input filter bandwidth should be set to half the command sending frequency;
                mode=1: trapezoidal trajectory mode; the target speed during motion and the start/stop acceleration can be specified;
                mode=2: feedforward control mode; speed and torque are feedforward terms. Feedforward adds speed and torque feedforward on top of the PID control,
                        improving response and reducing steady-state error.

    Note: In mode=1 (trapezoidal), speed and accel must both be > 0; in mode=0 speed has no effect.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in step_angle---"

    """
    factor = 0.01
    try:
        if mode == 0:
            f_angle = angle
            s16_speed = int((abs(speed)) / factor)
            if param > 300:
                print("input_filter_width = " + str(param) + ", which is too big and resized to 300")
                param = 300
            s16_width = int(abs(param / factor))
            data = format_data([f_angle, s16_speed, s16_width], 'f s16 s16', 'encode')
            send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0)
            order_num = 0x10
            data = format_data([order_num, 1], 'u32 u16', 'encode')  # # Convert order_num into 4 bytes of data,
            send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        elif mode == 1:
            if speed > 0 and param > 0:
                f_angle = angle
                s16_speed = int((abs(speed)) / factor)
                s16_accel = int((abs(param)) / factor)
                data = format_data([f_angle, s16_speed, s16_accel], 'f s16 s16', 'encode')
                send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0)
                order_num = 0x11
                data = format_data([order_num, 1], 'u32 u16', 'encode')  # # Convert order_num into 4 bytes of data,
                send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
            else:
                print("speed or accel <= 0")
        elif mode == 2:
            f_angle = angle
            s16_speed_ff = int((speed) / factor)
            s16_torque_ff = int((param) / factor)
            data = format_data([f_angle, s16_speed_ff, s16_torque_ff], 'f s16 s16', 'encode')
            send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0)
            order_num = 0x12
            data = format_data([order_num, 1], 'u32 u16', 'encode')  # # Convert order_num into 4 bytes of data,
            send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in step_angle---：", e)
        return False
    return True

# Synchronized relative angle control of multiple joints
def step_angles(id_list=[1, 2, 3], angle_list=[0, 0, 0], speed=10, param=10, mode=1):
    """Control of multiple joints.

    Rotates multiple joints by the given relative angles within the given time.

    Args:
           id_list: List of joint IDs.
        angle_list: List of relative target angles.
             speed: Speed (r/min), depends on mode:
                    mode=1: speed of the fastest of the joints;
                    mode=0/2: feedforward speed.
             param: Motion parameter, depends on mode:
                    mode=0: angle input filter bandwidth (<300);
                    mode=1: acceleration during start and stop phases (r/min/s);
                    mode=2: feedforward torque (Nm).
              mode: Control mode. The joints support three angle control modes, selected by mode:
                    mode=0: multi-joint trajectory tracking mode, suited to smooth control over many input trajectory points; the angle input bandwidth should be set to half the command sending frequency;
                    mode=1: multi-joint trapezoidal trajectory mode; speed is the fastest speed of these joints (r/min), param is the target acceleration (r/min/s);
                    mode=2: feedforward control mode; speed and torque are feedforward terms. Feedforward adds speed and torque feedforward on top of the PID control,
                            improving response and reducing steady-state error.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        A message is shown if id_list and angle_list have different lengths.
        ---error in step_angles---

    """
    try:
        if len(id_list) == len(angle_list):
            if mode == 0:
                for i in range(len(id_list)):
                    preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param,
                                 mode=mode)  # Execute preset_angle for each angle in turn
                order_num = 0x10
                data = format_data([order_num, 1], 'u32 u16', 'encode')
                send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
            elif mode == 1:
                if speed > 0 and param > 0:
                    DETA_angle_list = [abs(x) for x in angle_list]  # Compute the absolute value of each relative angle
                    delta_angle = max(DETA_angle_list)  # Record the largest relative angle
                    if delta_angle <= (6 * speed * speed / abs(param)):  # Maximum relative angle is smaller than the angle covered by a full acceleration + deceleration
                        t = 2 * cm.sqrt(delta_angle / (6 * abs(param)))  # Time needed for an isosceles triangle (speed) trajectory
                    else:
                        t = speed / abs(param) + delta_angle / (6 * speed)  # Time needed to complete the full trapezoidal trajectory
                    for i in range(len(id_list)):
                        preset_angle(id_num=id_list[i], angle=angle_list[i], t=t, param=param,
                                     mode=mode)  # Send each relative angle and time to the corresponding joint
                    order_num = 0x11
                    data = format_data([order_num, 2], 'u32 u16', 'encode')  # # Convert order_num into 4 bytes of data,
                    send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
                else:
                    print("speed or accel <= 0")
            elif mode == 2:
                for i in range(len(id_list)):
                    preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param, mode=mode)
                order_num = 0x12
                data = format_data([order_num, 1], 'u32 u16', 'encode')
                send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        else:
            print("id_list 和 angle_list 长度不一致")
    except Exception as e:
        print("---error in step_angles---：", e)
        return False
    return True

# Adaptive absolute angle control of a single joint
def set_angle_adaptive(id_num=0, angle=0, speed=0, torque=0):
    """Adaptive angle control of a single joint.

    Moves the joint with the given ID to the given angle (absolute angle, relative to the user-defined zero point) within the given speed and torque limits.
    Note: If the speed is set too high relative to the torque, or the torque too low relative to the speed, the joint cannot decelerate to 0 speed quickly
    enough, so if it meets no resistance it will overshoot the target angle. This is physics; there is currently no good solution.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
         angle: Joint angle (deg).
         speed: Speed limit (r/min).
        torque: Torque limit (Nm).

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in set_angle_adaptive---"

    """
    factor = 0.01
    try:
        f_angle = angle
        s16_speed = int((abs(speed)) / factor)
        s16_torque = int(abs(torque / factor))
        data = format_data([f_angle, s16_speed, s16_torque], 'f s16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x0B, data=data, rtr=0)
    except Exception as e:
        print("---error in set_angle_adaptive---：", e)
        return False
    return True

# Adaptive absolute angle control of multiple joints
def set_angles_adaptive(id_list=[0, 0, 0], angle_list=[0, 0, 0], speed_list=[0, 0, 0], torque_list=[0, 0, 0]):
    """Adaptive angle control of multiple joints.

    Moves multiple joints to the given angles (absolute angles, relative to the user-defined zero point) within the given speed and torque limits.
    Note: If the speed is set too high relative to the torque, or the torque too low relative to the speed, the joint cannot decelerate to 0 speed quickly
    enough, so if it meets no resistance it will overshoot the target angle. This is physics; there is currently no good solution.

    Args:
            id_list: List of joint IDs.
         angle_list: List of joint angles (deg).
         speed_list: List of speed limits (r/min).
        torque_list: List of torque limits (Nm).

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        A message is shown if the number of joints and the number of parameters differ.
        "---error in set_angles_adaptive---"

    """
    try:
        length = len(id_list)
        if len(angle_list) != length or len(speed_list) != length or len(torque_list) != length:
            print("关节数量与参数数量不一致，请检查")
        for i in range(length):
            preset_angle(id_num=id_list[i], angle=angle_list[i], t=abs(speed_list[i]), param=abs(torque_list[i]), mode=1)
        order_num = 0x11
        data = format_data([order_num, 3], 'u32 u16', 'encode')  # # Convert order_num into 4 bytes of data,
        send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_angles_adaptive---：", e)
        return False
    return True

# Impedance control of a single joint
def impedance_control(id_num=0, angle=0, speed=0, tff=0, kp=0, kd=0, mode=1):
    """Impedance control of a single joint.

    Applies impedance control to the joint with the given ID. When done, the joint stays at the target angle and behaves with some compliance (springiness) to external forces.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
         angle: Target joint angle (deg).
         speed: Target joint speed (r/min).
           tff: Feedforward torque (Nm).
            kp: Angle stiffness coefficient (Nm/deg), must be > 0.
            kd: Speed damping coefficient (Nm/(r/min)), must be > 0.
          mode: Mode selection: 1 = angle is the control target, 0 = torque is the control target.

    Note: This function directly controls the joint output torque. The target output torque is:
          torque = kp*( angle – angle_) + tff + kd*(speed – speed_)
          where angle_ and speed_ are the actual current output shaft angle (deg) and speed (r/min), and kp and kd are the stiffness and damping coefficients.

    Returns:
         True: ran normally
        False: an error occurred

    Raises:
        "---error in impedance_control---"

    """

    factor = 0.001
    try:
        kp = abs(kp)
        kd = abs(kd)
        if kp > 20:
            kp = 20 # Limit the coefficients, otherwise the joint easily vibrates under load
        if kd > 20:
            kd = 20
        if mode == 1:
            if kp != 0:
                angle_set = (- kd * speed - tff) / kp + angle
            else:
                print("机器人中关节不允许不间断连续旋转")
                return False
        else:
            angle_set = angle
        preset_angle(id_num=id_num, angle=angle_set, t=speed, param=tff, mode=2)
        order_num = 0x15
        data = format_data([order_num, int(kp / factor), int(kd / factor)], 'u32 s16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in impedance_control---：", e)
        return False
    return True

# Impedance control of multiple joints
def impedance_control_multi(id_list=[1, 2, 3], angle_list=[0, 0, 0], speed_list=[0, 0, 0], tff_list=[0, 0, 0], kp_list=[0, 0, 0], kd_list=[0, 0, 0], mode=1):
    """Impedance control of multiple joints.

    Applies impedance control to multiple joints. When done, each specified joint stays at its target angle and behaves with some compliance (springiness) to external forces.

    Args:
           id_list: List of joint IDs.
        angle_list: List of target joint angles (deg).
        speed_list: List of target joint speeds (r/min).
          tff_list: List of feedforward torques (Nm).
           kp_list: List of angle stiffness coefficients (Nm/deg), each must be > 0.
           kd_list: List of speed damping coefficients (Nm/(r/min)), each must be > 0.
              mode: Mode selection: 1 = angle is the control target, 0 = torque is the control target.

    Note: This function directly controls the joint output torque. The target output torque is:
          torque = kp_list[i] * (angle_list[i] – angle_[i]) + tff_list[i] + kd_list[i] * (speed_list[i] – speed_[i])
          where angle_[i] and speed_[i] are the actual current output shaft angle (deg) and speed (r/min) of each joint, and kp_list[i] and kd_list[i]
          are the stiffness and damping coefficients.

    Returns:
        True: ran normally.
        False: an error occurred.

    Raises:
        "---error in impedance_control_multi---"

    """

    factor = 0.001
    try:
        lenth = len(id_list)
        if len(angle_list) != lenth or len(speed_list) != lenth or len(tff_list) != lenth or len(kp_list) != lenth or len(kd_list) != lenth:
            print("参数数目与关节数量不符，请检查")
            return False
        for i in range(lenth):
            kp_list[i] = abs(kp_list[i])
            kd_list[i] = abs(kd_list[i])
            if kp_list[i] > 10:
                kp_list[i] = 10  # Limit the coefficients, otherwise the joint easily vibrates under load
            if kd_list[i] > 10:
                kd_list[i] = 10
            if mode == 1:
                if kp_list[i] != 0:
                    angle_list[i] = (- kd_list[i] * speed_list[i] - tff_list[i]) / kp_list[i] + angle_list[i]
                else:
                    print("机器人中关节不允许不间断连续旋转")
                    return False
            else:
                angle_list[i] = angle_list[i]
            preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed_list[i], param=tff_list[i], mode=2)
            order_num = 0x16
            data = format_data([order_num, int(kp_list[i] / factor), int(kd_list[i] / factor)], 'u32 s16 s16', 'encode')
            send_command(id_num=id_list[i], cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
            time.sleep(0.00000001)
        order_num = 0x17
        data = format_data([order_num], 'u32', 'encode')
        send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in impedance_control_multi--：", e)
        return False
    return True

# Motion assist of a single joint
def motion_aid(id_num=0, angle=0, speed=0, angle_err=0, speed_err=0, torque=0):
    """Motion following and assist for a single joint.

    The joint with the given ID performs motion following and assist.
    When the joint is stationary and detects an angle difference angle_err and a speed difference speed_err, it provides an assist torque of size torque
    towards the target angle, then stops and holds position once it reaches angle.
    Notes:
      a. If assist + external driving force is greater than the resistance, the joint keeps rotating;
      b. If assist + external driving force is smaller than the resistance, the joint starts to slow down; when the speed drops below 2x speed_err, the joint stops giving assist;
      c. This function is normally used to assist a person. It is strongly recommended to keep the assist torque within what a person can resist, i.e. a person can stop the joint;
      d. If a torque beyond human strength is required, solid mechanical end stops MUST be fitted at sensible positions, to avoid injuring people or damaging objects by exceeding the range of motion.

    Args:
           id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
            angle: Target angle (deg), i.e. the assist target angle. This value minus the current joint angle is the assist stroke. Range -300~300 deg.
            speed: Speed limit (r/min) of the assist, prevents the assist torque from accelerating the joint to an excessive speed.
        angle_err: Angle difference (deg), the angle sensitivity of motion following and assist.
        speed_err: Speed difference (r/min), the speed sensitivity of motion following and assist.
           torque: Assist torque (Nm).

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in motion_follow_aid---"

    """
    factor = 0.01
    try:
        if angle < -300 or angle > 300:
            print("助力角度超出范围，允许范围为 -300°~300°，请检查")
            return False
        data = format_data([int(angle / factor), int(angle_err / factor), int(speed_err / factor), int(torque / factor)], 's16 u16 u16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x0D, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        set_speed_adaptive(id_num=id_num, speed_adaptive=speed)
    except Exception as e:
        print("---error in motion_follow_aid---：", e)
        return False
    return True

# Motion assist of multiple joints
def motion_aid_multi(id_list=[1, 2, 3], angle_list=[0, 0, 0], speed_list=[0, 0, 0], angle_err_list=[1, 1, 1], speed_err_list=[1, 1, 1], torque_list=[1, 1, 1]):
    """Motion following and assist for multiple joints.

    The specified joints perform motion following and assist.
    When a joint is stationary and detects an angle difference angle_er_list[i] and a speed difference speed_err_list[i], it provides an assist torque of size
    torque_list[i] towards the target angle angle_list[i], then stops and holds position once it reaches angle_list[i].
    Notes:
      a. If assist + external driving force is greater than the resistance, the joint keeps rotating;
      b. If assist + external driving force is smaller than the resistance, the joint starts to slow down; when the speed drops below 2x speed_err_list[i], the joint stops giving assist;
      c. This function is normally used to assist a person. It is strongly recommended to keep the assist torque within what a person can resist, i.e. a person can stop the joint;
      d. If a torque beyond human strength is required, solid mechanical end stops MUST be fitted at sensible positions, to avoid injuring people or damaging objects by exceeding the range of motion.

    Args:
               id_list: List of joint IDs.
            angle_list: List of target angles (deg), i.e. the assist target angles. Each value minus the current joint angle is the assist stroke.
            speed_list: List of speed limits (r/min) of the assist, prevents the assist torque from accelerating the joint to an excessive speed.
        angle_err_list: List of angle differences (deg), the angle sensitivity of motion following and assist.
        speed_err_list: List of speed differences (r/min), the speed sensitivity of motion following and assist.
           torque_list: List of assist torques (Nm).

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in motion_follow_aid_multi---"

    """
    factor = 0.01
    try:
        length = len(id_list)
        for i in range(length):
            if angle_list[i] < -300 or angle_list[i] > 300:
                print("ID 号为", i, "的关节助力角度超出范围，允许范围为 -300°~300°，请检查")
                return False
        if len(angle_err_list) != length or len(speed_list)!=length or len(speed_err_list) != length or len(torque_list) != length:
            print("关节数量与参数数量不一致，请检查")
        for i in range(length):
            data = format_data([int(angle_list[i] / factor), int(angle_err_list[i] / factor), int(speed_err_list[i] / factor), int(torque_list[i] / factor)], 's16 u16 u16 s16',
                               'encode')
            send_command(id_num=id_list[i], cmd=0x06, data=data, rtr=0)  # MSG_PRESET_FOUR must be sent as a standard frame (data frame), not a remote frame
        order_num = 0x11
        data = format_data([order_num, 4], 'u32 u16', 'encode')  # # Convert order_num into 4 bytes of data,
        send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        time.sleep(0.1)
        for i in range(length):
            # set_speed_limit(id_list[i], speed_limit=speed_list[i])
            set_speed_adaptive(id_list[i], speed_adaptive=speed_list[i])
    except Exception as e:
        print("---error in motion_follow_aid_multi---：", e)
        return False
    return True

# Check and wait until a single joint reaches its target
def position_done(id_num=0):
    """Check and wait until a single joint has reached its target (trapezoidal trajectory mode).

    Checks and waits until a single joint has rotated to the specified angle.
    Supports the following three angle control modes:
                             trapezoidal trajectory mode, mode=1
                             feedforward mode, mode=2
                             adaptive mode, set_angle_adaptive()

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
        True: reached the specified angle.
        None: has not reached the specified angle yet.

    Raises:
        "---error in position_done---"

    """
    kk = 0
    try:
        while kk == 0:
            kk = read_property(id_num, property='dr.controller.position_done')
            # print(kk)
            # print(id_num)
    except Exception as e:
        print("---error in position_done---：", e)
        return False
    if kk == 1:
        return True

# Check and wait until multiple joints reach their targets
def positions_done(id_list=[1, 2, 3]):
    """Check and wait until multiple joints have reached their targets (trapezoidal trajectory mode).

    Checks and waits until all of the joints have rotated to their specified angles.
    Supports the following three angle control modes:
                             trapezoidal trajectory mode, mode=1
                             feedforward mode, mode=2
                             adaptive mode, set_angle_adaptive()

    Args:
        id_num: List of joint IDs.

    Returns:
        True: all reached their specified angles.
        None: not all have reached their specified angles yet.

    Raises:
        "---error in positions_done---"

    """
    k = 1
    try:
        while (k == 1):
            kk = 1
            for i in range(len(id_list)):
                kk = kk & position_done(id_list[i])
            if kk == 1:
                k = 0
    except Exception as e:
        print("---error in positions_done---：", e)
        return False
    return True

# Speed control
def set_speed(id_num=0, speed=10, param=0, mode=1):
    """Speed control of a single joint.

    Makes the joint with the given ID rotate continuously at the given speed (stops automatically at the joint's limit angle).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
         speed: Target speed (r/min).
         param: Motion parameter, depends on mode:
                mode=0: feedforward torque (Nm);
                mode!=0: target acceleration (r/min/s).
          mode: Control mode, selected by mode:
                mode=0: direct speed control, the joint target speed is set directly to speed;
                mode!=0: constant acceleration control, the joint ramps to speed at the target acceleration.

    Note:
        In mode!=0 (constant acceleration), if the target acceleration is set to 0, the joint keeps its current speed.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in set_speed---"

    """
    factor = 0.01
    try:
        f_speed = speed
        if mode == 0:
            s16_torque = int((param) / factor)
            if f_speed == 0:
                s16_torque = 0
            u16_input_mode = 1
            data = format_data([f_speed, s16_torque, u16_input_mode], 'f s16 u16', 'encode')
        else:
            s16_ramp_rate = int((param) / factor)
            u16_input_mode = 2
            data = format_data([f_speed, s16_ramp_rate, u16_input_mode], 'f s16 u16', 'encode')
        send_command(id_num=id_num, cmd=0x1C, data=data, rtr=0)
    except Exception as e:
        print("---error in set_speed---：", e)
        return False
    return True

# Speed control of multiple joints
def set_speeds(id_list=[1, 2, 3], speed_list=[10.0, 20.0, 30.0], param=0, mode=1):
    """Speed control of multiple joints.

    Makes multiple joints rotate continuously at the given speeds (stops automatically at the joint's limit angle).

    Args:
           id_list: List of joint IDs.
        speed_list: List of target joint speeds (r/min).
             param: Motion parameter, depends on mode:
                    mode=0: feedforward torque (Nm);
                    mode!=0: target acceleration (r/min/s).
              mode: Control mode, selected by mode:
                    mode=0: direct speed control, the joint target speed is set directly to speed;
                    mode!=0: constant acceleration control, the joint ramps to speed at the target acceleration.

    Note:
        In mode!=0 (constant acceleration), if the target acceleration is set to 0, the joint keeps its current speed.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in set_speeds---"

    """
    try:
        if len(id_list) == len(speed_list):
            for i in range(len(id_list)):
                preset_speed(id_num=id_list[i], speed=speed_list[i], param=param, mode=mode)
            order_num = 0x13
            data = format_data([order_num], 'u32', 'encode')
            send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_speeds---：", e)
        return False
    return True

# Torque control
def set_torque(id_num=0, torque=0.1, param=0, mode=1):
    """Closed-loop torque (current) control of a single joint.

    Makes the joint with the given ID output the given torque (Nm). If the resistance is not enough to counter this torque, the joint keeps rotating (stops automatically at the joint's limit angle).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        torque: Target torque (Nm).
         param: Motion parameter, depends on mode:
                mode=0: parameter has no effect;
                mode!=0: torque increase per unit time (Nm/s).
          mode: Control mode, selected by mode:
                mode=0: direct torque control, the joint target is set directly to torque;
                mode!=0: constant torque ramp, the joint ramps to torque at the specified increase per unit time.

    Note:
        In mode!=0 (torque ramp), if the increase per unit time is set to 0, the joint keeps its current output torque.

    Returns:
        True: ran normally.
        False: an error occurred.

    Raises:
        "---error in set_torque---"

    """
    factor = 0.01
    try:
        f_torque = torque
        if mode == 0:
            u16_input_mode = 1
            s16_ramp_rate = 0
        else:
            u16_input_mode = 6
            s16_ramp_rate = int((param) / factor)
        data = format_data([f_torque, s16_ramp_rate, u16_input_mode], 'f s16 u16', 'encode')
        send_command(id_num=id_num, cmd=0x1D, data=data, rtr=0)
    except Exception as e:
        print("---error in set_torque---：", e)
        return False
    return True

# Torque control of multiple joints
def set_torques(id_list=[1, 2, 3], torque_list=[3.0, 4.0, 5.0], param=0, mode=1):
    """Torque control of multiple joints.

    Makes multiple joints output the given torques. If the resistance is not enough to counter the torque, the joint keeps rotating (stops automatically at the joint's limit angle).

    Args:
            id_list: List of joint IDs.
        torque_list: List of target joint torques (Nm).
              param: Motion parameter, depends on mode:
                    mode=0: parameter has no effect;
                    mode!=0: torque increase per unit time (Nm/s).
               mode: Control mode, selected by mode:
                     mode=0: direct torque control, the joint target is set directly to torque;
                     mode!=0: constant torque ramp, the joint ramps to torque at the specified increase per unit time.

    Note:
        In mode!=1 (torque ramp), if the increase per unit time is set to 0, the joint keeps its current output torque.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in set_torques---"

    """
    try:
        if len(id_list) == len(torque_list):
            for i in range(len(id_list)):
                preset_torque(id_num=id_list[i], torque=torque_list[i], param=param, mode=mode)
            order_num = 0x14
            data = format_data([order_num], 'u32', 'encode')
            send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_torques---：", e)
        return False
    return True

# Emergency stop
def estop(id_num=0):
    """Emergency stop.

    Makes the joint stop immediately. To stop several joints at the same time, use id_num=0.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: ran normally.
        False: an error occurred.

    Raises:
        "---error in estop---"

    """
    global estop_flag
    try:
        order_num = 0x06
        data = format_data([order_num], 'u32', 'encode')
        send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        estop_flag = 1
    except Exception as e:
        print("---error in estop---：", e)
        return False
    return True

# Read the joint ID number
def get_id():
    """Read the joint ID number.

    Reads the joint ID number. Note: only 1 joint may be connected to the bus when using this function.

    Args:
        None.

    Returns:
           id: Joint ID number.
        False: an error occurred.

    Raises:
        "error in get_id"

    """
    try:
        id = read_property(id_num=0, property='dr.config.can_id')
        print(id)
        return id
    except Exception as e:
        print("---error in get_id---：", e)
        return False

def get_ids():
    """Read the ID numbers of multiple joints on the bus.

        Reads the ID numbers of multiple joints on the bus.

        Args:
            None.

        Returns:
               id: List of joint ID numbers.
            False: an error occurred.

        Raises:
            "error in get_ids"

        """
    try:
        data_types = {'f': 0, 'u16': 1, 's16': 2, 'u32': 3, 's32': 4}
        property = 'dr.config.can_id'
        address = property
        if type(property) == str:
            address = key_find_value(property)
            if address > 0:
                pass
            else:
                print('invalid property: ' + str(property))
                return False
        data_type = property_type.get(value_find_key(address))
        data = format_data([address, data_types.get(data_type)], 'u16 u16', 'encode')
        send_command(id_num=0, cmd=0x1E, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        id_list = read_data_id()
        id_list.sort()
        print("总线中存在的不同 ID 号为", id_list)
        return id_list
    except Exception as e:
        print("---error in get_id---：", e)
        return False

# Read the joint's current angle
def get_angle(id_num=0):
    """Read the joint's current angle (deg).

    Reads the joint's current angle, in degrees (deg).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                but if there are several joints on the bus this disturbs bus communication, so do not broadcast with 0 then.

    Returns:
        angle: Current joint angle (deg).
        False: an error occurred.

    Raises:
        "---error in get_angle---"

    """
    try:
        return read_property(id_num=id_num, property='dr.output_shaft.angle')
    except Exception as e:
        print("---error in get_angle---：", e)
        return False

# Read the current speed
def get_speed(id_num=0):
    """Read the joint's current speed (r/min).

    Reads the joint's current speed, in revolutions per minute (r/min).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                but if there are several joints on the bus this disturbs bus communication, so do not broadcast with 0 then.

    Returns:
        speed: Current joint speed (r/min).
        False: an error occurred.

    Raises:
        "---error in get_speed---"

    """
    try:
        return read_property(id_num=id_num, property='dr.output_shaft.speed')
    except Exception as e:
        print("---error in get_speed---：", e)
        return False

# Read the current angle and speed at the same time
def get_state(id_num=0):
    """Read the joint's current angle (deg) and speed (r/min) at the same time.

        Reads the joint's current angle and speed.

        Args:
            id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                    but if there are several joints on the bus this disturbs bus communication, so do not broadcast with 0 then.

        Returns:
    [angle, speed]: List of the current joint angle and speed.
            False: an error occurred.

        Raises:
            "---error in get_state---"

        """
    try:
        return [read_property(id_num=id_num, property='dr.output_shaft.angle'), read_property(id_num=id_num, property='dr.output_shaft.speed')]
    except Exception as e:
        print("---error in get_state---：", e)
        return False

# Read the torque magnitude and direction
def get_torque(id_num=0):
    """Read the joint's current torque (Nm).

    Reads the joint's current output torque, in newton-metres (Nm).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                but if there are several joints on the bus this disturbs bus communication, so do not broadcast with 0 then.

    Returns:
        torque: Current joint output torque (Nm).
         False: an error occurred.

    Raises:
        "----error in get_torque---"

    """
    try:
        return read_property(id_num=id_num, property='dr.output_shaft.torque')
    except Exception as e:
        print("---error in get_torque---：", e)
        return False

# Read the joint's current voltage and current
def get_vol_cur(id_num=0):
    """Read the joint's current voltage and current.

    Reads the joint's current voltage and q-axis current, in volts (V) and amperes (A).

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                but if there are several joints on the bus this disturbs bus communication, so do not broadcast with 0 then.

    Returns:
        [vol, cur]: List of voltage and current.
             False: an error occurred.

    Raises:
        "---error in get_volcur--"

    """
    vol_cur = [0, 0]
    try:
        vol_cur[0] = read_property(id_num=id_num, property='dr.voltage')
        vol_cur[1] = read_property(id_num=id_num, property='dr.motor.Iq_measured')
        return vol_cur
    except Exception as e:
        print("---error in get_volcur--：", e)
        return False

# Enable real-time angle, speed and torque feedback
def enable_angle_speed_torque_state(id_num=0):
    """Enable real-time angle, speed and torque feedback.

        Enables real-time angle, speed and torque feedback.
        IMPORTANT: before running any other function that reads back parameters, first run disable_angle_speed_torque_state() to turn off the real-time feedback.

        Args:
            id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                    but if there are several joints on the bus, real-time feedback is enabled on all of them.

        Returns:
              True: enabled successfully.
             False: enabling failed.

        Raises:
            "---error in enable_angle_speed_torque_state--"

        """
    global angle_speed_torque_state_flag
    try:
        write_property(id_num, 'dr.can.enable_state_feedback', 1)
        angle_speed_torque_state_flag = 1
        return True
    except Exception as e:
        print("---error in enable_angle_speed_torque_state--：", e)
        return False

# Set the real-time angle/speed/torque feedback interval, in ms, default 2
def set_state_feedback_rate_ms(id_num=0, n_ms=2):
    """Set the real-time angle/speed/torque feedback interval, in ms, default 2.

        Sets the real-time angle/speed/torque feedback interval, in ms, default 2.
        IMPORTANT: when there are n joints with different IDs on the bus, it is recommended to set this value to 2n on all joints.

        Args:
            id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                    but if there are several joints on the bus, this applies to all of them.
              n_ms: Real-time angle/speed/torque feedback interval in ms. When there are n joints with different IDs on the bus, set this to 2n.

        Returns:
              True: set successfully.
             False: setting failed.

        Raises:
            "---error in set_state_feedback_rate_ms--"

        """

    try:
        for i in range(5):
            write_property(id_num, 'dr.config.state_feedback_rate_ms', n_ms)
        return True
    except Exception as e:
        print("---error in set_state_feedback_rate_ms--：", e)
        return False

# Real-time angle, speed and torque feedback
def angle_speed_torque_state(id_num=1, n=1):
    """Read the joint's real-time angle, speed and torque feedback.

        Reads the joint's real-time angle (angle, deg), speed (speed, r/min) and torque (torque, Nm) feedback.
        IMPORTANT:
                1. Because the bus sends data continuously, the first set of data read may be wrong; data is only guaranteed valid from the second consecutive call onwards.
                2. Before running any other function that reads back parameters, first run disable_angle_speed_torque_state() to turn off the real-time feedback.
        Args:
            id_num: Joint ID. When using this function, the joints on the bus should preferably already have IDs 1~63, with no duplicate IDs.
                 n: Number of joints on the bus.

        Returns:
angle_speed_torque: [angle, speed, torque]

        Raises:
            ---error in angle_speed_torque_state--

        """
    try:
        udata = read_data_state2(n)
        if udata == None:
            return None
        if READ_FLAG == 1:
            for i in range(n):
                jdata = udata[i * 16: (i + 1) * 16]
                cdata = uart_to_can_ID(data=jdata)
                if id_num == (cdata[1] * 256 + cdata[2] - 1) >> 5:
                    angle_speed_torque = format_data(data=cdata[3:], format='f s16 s16', type='decode')
                    return [round(angle_speed_torque[0], 3), round(angle_speed_torque[1] * 0.01, 3), round(angle_speed_torque[2] * 0.01, 3)]
            print("angle_speed_torque_state 函数中 ID 号有误")
            return None
    except Exception as e:
        print("---error in angle_speed_torque_state--：", e)
        return None

def angle_speed_torque_states(id_list=[1, 2, 3]):
    """Read the real-time angle, speed and torque feedback of multiple joints on the bus.

        Reads the joints' real-time angle (angle, deg), speed (speed, r/min) and torque (torque, Nm) feedback.
        IMPORTANT:
                1. Because the bus sends data continuously, the first set of data read may be wrong; data is only guaranteed valid from the second consecutive call onwards.
                2. Before running any other function that reads back parameters, first run disable_angle_speed_torque_state() to turn off the real-time feedback.
        Args:
            id_num_list: List of joint IDs. When using this function, the joints on the bus should preferably already have IDs 1~63, with no duplicate IDs.
                      n: Number of joints on the bus.

        Returns:
     angle_speed_torques: List of [angle, speed, torque], sorted by ID from low to high.
                    None: no data was read.

        Raises:
            ---error in angle_speed_torque_states--

        """
    try:
        n = len(id_list)
        id_list_sorted = sorted(id_list)
        # udata = read_data_state(n)
        udata = read_data_state2(n)
        if udata == None:
            return None
        angle_speed_torques = [None] * n # Create an empty array of length n
        if READ_FLAG == 1:
            id_num_list = []
            for i in range(n):
                jdata = udata[i * 16: (i + 1) * 16]
                cdata = uart_to_can_ID(data=jdata)
                id_num = ((cdata[1] * 256 + cdata[2] - 1) >> 5)
                if id_num in id_num_list:
                    print("ID 号重复，请调整状态反馈时间间隔")
                    return None
                else:
                    id_num_list.append(id_num)
                if id_num in id_list_sorted:
                    angle_speed_torque = format_data(data=cdata[3:], format='f s16 s16', type='decode')
                    if angle_speed_torque == None:
                        print("return None")
                        return None
                    else:
                        angle_speed_torques[id_list_sorted.index(id_num)] = [round(angle_speed_torque[0], 3), round(angle_speed_torque[1] * 0.01, 3), round(angle_speed_torque[2] * 0.01, 3)]
                else:
                    print("angle_speed_torque_state 函数中 ID 号有误")
                    return None
            return angle_speed_torques
    except Exception as e:
        print("---error in angle_speed_torque_state--：", e)
        return None

# Disable real-time angle, speed and torque feedback
def disable_angle_speed_torque_state(id_num=0):
    """Disable real-time angle, speed and torque feedback.

        Disables real-time angle, speed and torque feedback.
        IMPORTANT: before running any other function that reads back parameters, first run disable_angle_speed_torque_state() to turn off the real-time feedback.
        Args:
            id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                    but if there are several joints on the bus, real-time feedback is disabled on all of them.

        Returns:
              True: disabled successfully.
             False: disabling failed.

        Raises:
            "---error in disable_angle_speed_torque_state--"

        """
    global angle_speed_torque_state_flag
    try:
        for i in range(5):
            write_property(id_num, 'dr.can.enable_state_feedback', 0)
        time.sleep(0.5)
        angle_speed_torque_state_flag = 0
        while uart.inWaiting() > 0:
            uart.read(1) # Clear leftover data in the serial port
        return True
    except Exception as e:
        print("---error in disable_angle_speed_torque_state--：", e)
        return False

# Read the current angle, speed and torque of all joints on the bus at the same time
def get_angle_speed_torque_all(id_list=[1, 2, 3]):
    """Read the current angle, speed and torque (Nm) of all joints on the bus at the same time.

    Args:
        id_list: List of the IDs of all joints.

    Returns:
        [[angle, speed, torque]]: List of lists of the current angle (deg), speed (r/min) and torque (Nm) of all joints, sorted by ID from low to high.
         False: an error occurred.

    Raises:
        "----error in get_angle_speed_torque_all---"

    """
    try:
        order_num = 0x24
        data = format_data([order_num], 'u32', 'encode')
        send_command(id_num=0, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        n = len(id_list)
        id_list_sorted = sorted(id_list)
        # udata = read_data_state(n)
        udata = read_data_state2(n)
        if udata == None:
            return None
        angle_speed_torques = [None] * n # Create an empty array of length n
        if READ_FLAG == 1:
            id_num_list = []
            for i in range(n):
                jdata = udata[i * 16: (i + 1) * 16]
                cdata = uart_to_can_ID(data=jdata)
                id_num = ((cdata[1] * 256 + cdata[2] - 1) >> 5)
                if id_num in id_num_list:
                    print("ID 号重复，请调整状态反馈时间间隔")
                    return None
                else:
                    id_num_list.append(id_num)
                if id_num in id_list_sorted:
                    angle_speed_torque = format_data(data=cdata[3:], format='f s16 s16', type='decode')
                    if angle_speed_torque == None:
                        print("return None")
                        return None
                    else:
                        angle_speed_torques[id_list_sorted.index(id_num)] = [round(angle_speed_torque[0], 3), round(angle_speed_torque[1] * 0.01, 3), round(angle_speed_torque[2] * 0.01, 3)]
                else:
                    print("angle_speed_torque_state 函数中 ID 号有误")
                    return None
            return angle_speed_torques
    except Exception as e:
        print("---error in angle_speed_torque_state--：", e)
        return None

# Read PID
def get_pid(id_num=0):
    """Read the joint's control loop PID.

    Reads the control loop's position gain P, speed gain D and integral gain I.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                but if there are several joints on the bus this disturbs bus communication, so do not broadcast with 0 then.

    Returns:
     [P, I, I]: [position gain P, integral gain I, speed gain D].
         False: reading failed.

    Raises:
        "---error in get_pid--"

    """
    try:
        P = read_property(id_num=id_num, property='dr.controller.config.angle_gain')
        I = read_property(id_num=id_num, property='dr.controller.config.speed_integrator_gain')
        D = read_property(id_num=id_num, property='dr.controller.config.speed_gain')
        print('位置增益 P：%s' % P)
        print('积分增益 I：%s' % I)
        print('转速增益 D：%s' % D)
    except Exception as e:
        print("---error in get_pid--：", e)
        return False
    return [P, I, D]

# Read a parameter property
def read_property(id_num=0, property=''):
    """Read a joint property parameter.

    Reads a joint property parameter. The property parameters are the joint control parameters, stored in interface_enums.py.

    Args:
          id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast,
                  but if there are several joints on the bus this disturbs bus communication, so do not broadcast with 0 then.
        property: Name of the property parameter, e.g. "dr.voltage", "dr.config.can_id", etc.
                  See the keys of the property_address dictionary in parameter_interface.py for the full list.
    Returns:
        value: The value of the property parameter.
        False: wrong parameter, read failed, or an error occurred.

    Raises:
        "---error in read_property---"

    """
    try:
        data_types = {'f': 0, 'u16': 1, 's16': 2, 'u32': 3, 's32': 4}
        address = property
        if type(property) == str:
            address = key_find_value(property)
            if address > 0:
                pass
            else:
                print('invalid property: ' + str(property))
                return False
        data_type = property_type.get(value_find_key(address))
        data = format_data([address, data_types.get(data_type)], 'u16 u16', 'encode')
        send_command(id_num=id_num, cmd=0x1E, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
        cdata = receive_data()
        if READ_FLAG == 1:
            property = format_data(data=cdata, format='u16 u16 ' + data_type, type='decode')
            if len(property) > 0:
                return property[-1]
            else:
                print("参数错误")
                return False
        else:
            print("参数读取失败")
            return False
    except Exception as e:
        print("---error in read_property---：", e)
        return False

# Set the current angle as a temporary zero point, lost after power-off/restart. Do NOT call save_config() after this function, otherwise the original zero point will be changed
def set_zero_position_temp(id_num=0): # Not available on hollow-shaft joints
    """Set the joint zero-point angle, valid for this power-up only, lost after restart.

    Sets the current angle as the zero point of the joint output shaft. Afterwards the current angle is 0 degrees; lost after restart.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        "---error in set_zero_position_temp---"

    """
    try:
        order_num = 0x23
        data = format_data([order_num], 'u32', 'encode')
        send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_zero_position_temp---：", e)
        return False
    return True

# Set temporary angle limits
def set_angle_range(id_num=0, angle_min=-180, angle_max=180):
    """Set the joint's limit angles during operation. Once set, the controllable rotation angle is restricted to [angle_min, angle_max]. Lost after power-off/restart.

    Notes:
        a. The output shaft angle must be within [angle_min, angle_max] when calling this function, otherwise setting fails;
        b. This only applies to the current power-up and is lost when the joint restarts;
        c. The joint also has its own limit angle property, enabled by default, with range [-180.5 deg, 180.5 deg].
           That property is not affected by this function and applies after every power-up. To change or cancel it, use
           set_angle_range_config() and disable_angle_range_config(); see those functions.

    Args:
           id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        angle_min: Minimum limit angle (deg).
        angle_max: Maximum limit angle (deg).

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        ---error in set_angle_range---

    """
    try:
        angle = get_angle(id_num=id_num)
        if READ_FLAG == 1:
            if angle >= angle_min and angle <= angle_max:
                write_property(id_num=id_num, property='dr.output_shaft.angle_min', value=angle_min)
                write_property(id_num=id_num, property='dr.output_shaft.angle_max', value=angle_max)
                write_property(id_num=id_num, property='dr.output_shaft.enable_angle_limit', value=1)
            else:
                print("一体化关节输出轴当前角度不在" + str([angle_min, angle_max]) + "范围内，运行过程中软件限位范围设置失败！")
                return False
        else:
            print("一体化关节角度读取失败，软件限位范围设置失败！")
            return False
    except Exception as e:
        print("---error in set_angle_range---：", e)
        return False
    return True

# Cancel temporary angle limits
def disable_angle_range(id_num=0):
    """Cancel the joint's angle limits for the current run.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: cancelled successfully.
        False: cancelling failed.

    Raises:
        "---error in disable_angle_range---"

    """
    try:
        write_property(id_num=id_num, property='dr.output_shaft.enable_angle_limit', value=0)
        bol = read_property(id_num=id_num, property='dr.output_shaft.enable_angle_limit')
        if bol == 0:
            print("取消运行过程中的角度限位成功")
        else:
            print("取消运行过程中的角度限位失败")
            return False
    except Exception as e:
        print("---error in disable_angle_range---：", e)
        return False
    return True

# Set angle limit properties (persistent)
def set_angle_range_config(id_num=0, angle_min=-180, angle_max=180):
    """Set the joint's limit angle property. Once set, the controllable rotation angle is restricted to [angle_min, angle_max], by default after every power-up/restart.

    Notes:
        a. The output shaft angle must be within [angle_min, angle_max] when calling this function, otherwise setting fails;
        b. After the limit range is set, run save_config() so it applies after every power-up/restart;
        c. To cancel this property, use disable_angle_range_config() to turn it off; it then has no effect for this power-up.
           If save_config() is then called, the property is lost permanently; to get it back, simply call this function again.

    Args:
           id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        angle_min: Minimum limit angle (deg)
        angle_max: Maximum limit angle (deg)

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        "---error in set_angle_range_config---"

    """
    try:
        angle = get_angle(id_num=id_num)
        if READ_FLAG == 1:
            if angle >= angle_min and angle <= angle_max:
                write_property(id_num=id_num, property='dr.config.angle_min',
                               value=angle_min)
                write_property(id_num=id_num, property='dr.config.angle_max',
                               value=angle_max)
                write_property(id_num=id_num, property='dr.config.enable_angle_limit',
                               value=1)
                # save_config(id_num=id_num)
            else:
                print("一体化关节输出轴当前角度不在" + str([angle_min, angle_max]) + "范围内，运行过程中软件限位范围设置失败！")
                return False
        else:
            print("一体化关节角度读取失败，限位属性设置失败！")
            return False
    except Exception as e:
        print("---error in set_angle_range_config---：", e)
        return False
    return True

# Cancel angle limit properties (persistent)
def disable_angle_range_config(id_num=0):
    """Cancel the joint's angle limit property.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: cancelled successfully.
        False: cancelling failed.

    Raises:
        "---error in disable_angle_range_config---"

    """
    try:
        write_property(id_num=id_num, property='dr.config.enable_angle_limit', value=0)
        bol = read_property(id_num=id_num, property='dr.config.enable_angle_limit')
        # save_config(id_num=id_num)
        if bol == 0:
            print("取消角度限位属性成功")
        else:
            print("取消角度限位属性失败")
            return False
    except Exception as e:
        print("---error in disable_angle_range_config---：", e)
        return False
    return True

# Set output shaft speed limit
def set_speed_limit(id_num=0, speed_limit=1):
    """Set the joint speed limit.

        Sets the joint speed limit speed_limit (r/min); afterwards the absolute joint speed will not exceed speed_limit.
        Notes:
            1. The speed limit set by this function applies to the current power-up only and is lost after power-off or restart.
            2. To cancel the limit before restarting, simply set a very large value, e.g. speed_limit = 100000.
            3. If you want to keep speed_limit permanently, call save_config right after (use with care).
        Args:
                 id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
            speed_limit: Speed limit (r/min) (must be > 0).

        Returns:
             True: set successfully.
            False: setting failed.

        Raises:
            ---error in set_speed_limit---

        """
    try:
        if speed_limit <= 0:
            print("speed_limit 必须大于 0，请重新输入")
            return False
        else:
            preset_angle(id_num=id_num, angle=abs(speed_limit), t=0, param=0, mode=1)
            order_num = 0x18
            data = format_data([order_num], 'u32', 'encode')  # # Convert order_num into 4 bytes of data,
            send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_speed_limit---：", e)
        return False
    return True

# Set output shaft torque limit
def set_torque_limit(id_num=0, torque_limit=1):
    """Set the joint torque limit.

        Sets the joint torque limit torque_limit (Nm); afterwards the absolute joint torque will not exceed torque_limit.
        Notes:
            1. The torque limit set by this function applies to the current power-up only and is lost after power-off or restart.
            2. To cancel the limit before restarting, simply set a very large value, e.g. torque_limit = 100000.
            3. If you want to keep torque_limit permanently, call save_config right after (use with care).
        Args:
                  id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
            torque_limit: Torque limit (Nm) (must be > 0).

        Returns:
             True: set successfully.
            False: setting failed.

        Raises:
            ---error in set_toque_limit---

        """
    try:
        if torque_limit <= 0:
            print("torque_limit 必须大于 0，请重新输入")
            return False
        else:
            preset_angle(id_num=id_num, angle=abs(torque_limit), t=0, param=0, mode=1)
            order_num = 0x19
            data = format_data([order_num], 'u32', 'encode')  # # Convert order_num into 4 bytes of data,
            send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_toque_limit---：", e)
        return False
    return True

# Set output shaft adaptive speed
def set_speed_adaptive(id_num=0, speed_adaptive=1):
    """Set the joint adaptive speed.

        Sets the joint speed limit speed_adaptive (r/min); afterwards the absolute adaptive joint speed will not exceed speed_adaptive.
        Notes:
            1. The speed limit set by this function applies to the current power-up only and is lost after power-off or restart.
            2. To cancel the limit before restarting, simply set a very large value, e.g. speed_adaptive = 100000.
            3. This function must be run after a motion command.
        Args:
                    id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
            speed_adaptive: Adaptive speed limit (r/min) (must be > 0).

        Returns:
             True: set successfully.
            False: setting failed.

        Raises:
            ---error in set_speed_adaptive---

        """
    try:
        if speed_adaptive <= 0:
            print("speed_adaptive 必须大于 0，请重新输入")
            return False
        else:
            preset_angle(id_num=id_num, angle=abs(speed_adaptive), t=0, param=0, mode=1)
            order_num = 0x20
            data = format_data([order_num], 'u32', 'encode')  # # Convert order_num into 4 bytes of data,
            send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_speed_adaptive---：", e)
        return False
    return True

# Set output shaft adaptive torque
def set_torque_adaptive(id_num=0, torque_adaptive=1):
    """Set the joint adaptive torque.

        Sets the joint torque limit torque_adaptive (Nm); afterwards the absolute adaptive joint torque will not exceed torque_adaptive.
        Notes:
            1. The torque limit set by this function applies to the current power-up only and is lost after power-off or restart.
            2. To cancel the limit before restarting, simply set a very large value, e.g. torque_adaptive = 100000.
            3. This function must be run after a motion command.
        Args:
                  id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
            torque_adaptive: Torque limit (Nm) (must be > 0).

        Returns:
             True: set successfully.
            False: setting failed.

        Raises:
            ---error in set_toque_adaptive---

        """
    try:
        if torque_adaptive <= 0:
            print("torque_adaptive 必须大于 0，请重新输入")
            return False
        else:
            preset_angle(id_num=id_num, angle=abs(torque_adaptive), t=0, param=0, mode=1)
            order_num = 0x21
            data = format_data([order_num], 'u32', 'encode')  # # Convert order_num into 4 bytes of data,
            send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_toque_adaptive---：", e)
        return False
    return True

# Set temporary PID
def set_pid(id_num=0, P=20, I=25, D=20):
    """Set the joint control loop PID.

    Sets the control loop's position gain P, speed gain D and integral gain I, to tune the joint's control performance.
    The PID values set by this function apply to the current power-up only and are lost after power-off or restart.
    To use a PID set permanently, call save_config right after setting the PID with this function.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
             P: Position gain (must be > 0).
             I: Integral gain (must be > 0).
             D: Speed gain (must be > 0).

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        ---error in set_pid---

    """
    try:
        if P <= 0 or D <= 0 or I <= 0:
            print("请输出大于 0 的 PID 数值")
            return False
        write_property(id_num=id_num, property='dr.controller.config.angle_gain', value=P)
        write_property(id_num=id_num, property='dr.controller.config.speed_integrator_gain', value=I)
        write_property(id_num=id_num, property='dr.controller.config.speed_gain', value=D)
    except Exception as e:
        print("---error in set_pid---：", e)
        return False
    return True

# Mode setting
def set_mode(id_num=0, mode=2):
    """Set the joint mode.

    Puts the joint into a different control mode.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
          mode: Joint mode number:
                mode=1: idle mode, the joint is unloaded (no torque)
                mode=2: closed-loop control mode; motion control functions only work in closed-loop mode. (Default mode after power-up)

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        ---error in set_mode---

    """
    try:
        if mode == 1:
            write_property(id_num=id_num, property='dr.requested_state', value=1)
        elif mode == 2:
            write_property(id_num=id_num, property='dr.requested_state', value=2)
    except Exception as e:
        print("---error in set_mode---：", e)
        return False
    return True

# Set CAN baud rate
def set_can_baud_rate(id_num=0, baud_rate=500000):
    """Set the joint CAN baud rate.

    Sets the CAN baud rate (kept after power-off).
    Note: recommended to run with no load, otherwise the joint may briefly unload.

    Args:
           id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        baud_rate: CAN baud rate, one of 125k, 250k, 500k or 1M. After changing it, the host CAN baud rate must be manually set to the same value.

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        ---error in set_can_baud_rate---

    """
    try:
        write_property(id_num=id_num, property='dr.can.config.baud_rate', value=baud_rate)
        save_config(id_num=id_num)
    except Exception as e:
        print("---error in set_can_baud_rate---：", e)
        return False
    return True

def write_property(id_num=0, property='', value=0):
    """Modify a joint property parameter.

    Modifies a joint property parameter. The property parameters are the joint control parameters, stored in interface_enums.py.

    Args:
          id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        property: Name of the property parameter, e.g. "dr.voltage", "dr.config.can_id", etc.
                  See the keys of the property_address dictionary in parameter_interface.py for the full list.
           value: Target value of the parameter.

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        "---error in write_property---"

    """
    try:
        data_types = {'f': 0, 'u16': 1, 's16': 2, 'u32': 3, 's32': 4} # Dictionary
        address = property
        if type(property) == str: # Check whether the property name is a string
            address = key_find_value(property) # Return the code corresponding to the property
            if address > 0:
                pass
                # print(str(value_find_key(address)) + ' address = ' + str(address))
            else:
                print('invalid property: ' + str(property))
                return False
        data_type = property_type.get(value_find_key(address)) # Determine the data type of the property
        # print(data_type)
        data = format_data([address, data_types.get(data_type), value], 'u16 u16 ' + data_type, 'encode')
        send_command(id_num=id_num, cmd=0x1F, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in write_property---：", e)
        return False
    return True

# Save the joint configuration
def save_config(id_num=0):
    """Save the configuration.

    Saves the joint property parameters changed by the user. The property parameters are the joint control parameters, stored in interface_enums.py.
    Normally, properties changed with write_property revert to their previous values after the joint is powered off or restarted;
    to keep them permanently, use save_config to save them to flash, so they are not lost after power-off or restart.
    Note: recommended to run with no load, otherwise the joint may briefly unload.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: set successfully.
        False: setting failed.

    Raises:
        "---error in save_config---"

    """
    try:
        order_num = 0x01
        data = format_data([order_num], 'u32', 'encode')
        send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in save_config---：", e)
        return False
    time.sleep(2)
    return True

"""
Other system helper functions, normally not needed
"""

# Reboot
def reboot(id_num=0):
    """Reboot the joint.

    Performs a software restart of the joint, similar to power-cycling it.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: restart started.
        False: restart failed.

    Raises:
        "---error in reboot---"

    """
    try:
        order_num = 0x03
        data = format_data([order_num], 'u32', 'encode')
        send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in reboot---：", e)
        return False
    return True

# Restore factory settings, without changing the ID number
def init_config(id_num=0):
    """Restore factory settings.

    Restores the factory parameter configuration, without changing the user-set ID number.
    Recommended to run with no load, otherwise the joint may briefly unload.

    Args:
        id_num: Joint ID. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.

    Returns:
         True: factory settings restored.
        False: restoring factory settings failed.

    Raises:
        "---error in init_config---"

    """
    try:
        data = format_data([id_num], 'u32', 'encode')
        send_command(id_num=id_num, cmd=0x0E, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in init_config---：", e)
        return False
    return True

def clear_uart():  # Clear leftover data in the serial port
    uart.flushInput()























































































































































































"""
Internal helper functions, not for users
"""

# Serial send function
def write_data(data=[]):
    global uart
    try:
        result = uart.write(data)  # Write data
        # print("write_data: ", data)
        return result
    except Exception as e:
        print("---error in write_data--：", e)
        print("重启串口")
        uart.close()
        uart.open()
        result = uart.write(data)  # Write data
        return result


# Serial receive function
def read_data(num=16): # 16 bytes
    global READ_FLAG
    READ_FLAG = -1
    byte_list = []
    byte_list_head = 0
    i = 500  # Testing showed that normally receiving 16 bytes takes about 500 (loop iterations)
    while uart.inWaiting() == 0 and i > 0:  # To do:
        i -= 1
        time.sleep(0.001)
        if i == 0:
            print("数据为空，接收数据超时，程序退出")
            raise TimeoutError('DrEmpower: no reply from motor')  # [delta_motors] was sys.exit()
    while byte_list_head != 170:
        byte_list_head = uart.read(1)[0]
    byte_list.append(byte_list_head)
    while uart.inWaiting() > 0 and len(byte_list) < num:
        byte_list.append(list(uart.read(1))[0])
    if len(byte_list) == num:
        READ_FLAG = 1
        return byte_list
    elif len(byte_list) > num:
        byte_list_length = len(byte_list)
        READ_FLAG = 1
        for i in range(byte_list_length - num):
            byte_list.pop(byte_list_length - i - 1)
        return byte_list
    else:
        print("Received data error in read_data(): " + str(byte_list))
        READ_FLAG = -1
        byte_list = []
        return

def read_data_state(n): # 16*n bytes
    global READ_FLAG
    READ_FLAG = -1
    byte_list = []
    byte_list_head = 0
    i = 500  # Testing showed that normally receiving 16 bytes takes about 500 (loop iterations)
    while uart.inWaiting() == 0 and i > 0:  # To do:
        i -= 1
        time.sleep(0.001)
        if i == 0:
            print("数据为空，接收数据超时，程序退出")
            raise TimeoutError('DrEmpower: no reply from motor')  # [delta_motors] was sys.exit()
    while byte_list_head != 170:
        byte_list_head = uart.read(1)[0]
    byte_list.append(byte_list_head)
    while uart.inWaiting() > 0 or len(byte_list) < (n * 16):
        byte_list.append(list(uart.read(1))[0])
    if len(byte_list) == (n * 16):
        READ_FLAG = 1
        return byte_list
    elif len(byte_list) > (n * 16):
        byte_list_length = len(byte_list)
        READ_FLAG = 1
        for i in range(byte_list_length - (n * 16)):
            byte_list.pop(byte_list_length - i - 1)
        return byte_list
    else:
        print("Received data error in read_data_state(): " + str(byte_list))
        READ_FLAG = -1
        return

def read_data_state2(n): # 16*n bytes
    global READ_FLAG
    READ_FLAG = -1
    byte_list = []
    byte_list_head = 0
    while uart.inWaiting() == 0:
        pass
    while byte_list_head != 170:
        byte_list_head = uart.read(1)[0]
    byte_list.append(byte_list_head)
    while uart.inWaiting() > 0 or len(byte_list) < (n * 16):
        if uart.inWaiting() == 0 and len(byte_list) < (n * 16):
            break
        byte_list.append(list(uart.read(1))[0])
    if len(byte_list) == (n * 16):
        READ_FLAG = 1
        return byte_list
    elif len(byte_list) > (n * 16):
        byte_list_length = len(byte_list)
        READ_FLAG = 1
        for i in range(byte_list_length - (n * 16)):
            byte_list.pop(byte_list_length - i - 1)
        return byte_list
    else:
        print("Received data error in read_data_state(): " + str(byte_list))
        READ_FLAG = -1
        return None

def read_data_id(): # 16*n bytes
    byte_list = []
    id_list = []
    i = 500  # Testing showed that normally receiving 16 bytes takes about 500 (loop iterations)
    while uart.inWaiting() == 0 and i > 0:  # To do:
        i -= 1
        time.sleep(0.001)
        if i == 0:
            print("数据为空，接收数据超时，程序退出")
            raise TimeoutError('DrEmpower: no reply from motor')  # [delta_motors] was sys.exit()
    while uart.inWaiting() > 0:
        byte_list.append(list(uart.read(1))[0])
    if len(byte_list) % 16 == 0 and len(byte_list) >= 16:
        for i in range(len(byte_list) // 16):
            jdata = byte_list[i * 16: (i + 1) * 16]
            cdata = uart_to_can_ID(data=jdata)
            id_list.append((cdata[1] * 256 + cdata[2] - 1) >> 5)
        return id_list
    else:
        print("Received data error in read_data_id(): " + str(byte_list))
        return

# USB-to-CAN module packet mode: CAN message -> serial frame
def can_to_uart(data=[], rtr=0):
    udata = [0xAA, 0, 0, 0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] # where 0x08 is the DLC (message length) in the CAN header
    # udata[1] is the IDE in the CAN header, udata[2] the RTR, udata[3] the DLC, udata[4~7] the id
    # udata is defined by the USB-to-CAN module's serial packet mode
    if len(data) == 11 and data[0] == 0x08: # 0x08 is a preset check byte
        if rtr == 1:
            udata[2] = 0x01 # rtr flag
        for i in range(10):
            udata[6 + i] = data[i + 1]
        return udata
    else:
        return []


# USB-to-CAN module packet mode: serial frame -> CAN message
def uart_to_can(data=[]):
    global READ_FLAG
    cdata = [0x08, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    # if len(data) == 16 and data[3] == 0x08:
    if (len(data) % 16 == 0) and data[3] == 0x08:
        for i in range(10):
            cdata[1 + i] = data[i + 6]
        return cdata
    else:
        READ_FLAG = -1
        return []

# USB-to-CAN module packet mode: serial frame -> CAN message, also computes the CAN node ID
def uart_to_can_ID(data=[]):
    global READ_FLAG
    cdata = [0x08, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    if len(data) == 16 and data[3] == 0x08:
        for i in range(10):
            cdata[1 + i] = data[i + 6]
        return cdata
    else:
        READ_FLAG = -1
        # print(READ_FLAG)
        return [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


# CAN send function
def send_command(id_num=0, cmd=0x09, data=[], rtr=0):
    global set_angles_mode_1_flag
    global estop_flag
    cdata = [0x08, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    id_list = (id_num << 5) + cmd # id number shifted left by 5 bits + cmd, determined by the relation between the odrive CAN node_id and cmd_id
    cdata[1] = id_list >> 8
    cdata[2] = id_list & 0xFF
    for i in range(8):
        cdata[3 + i] = data[i] # data[] holds the command content, e.g. angle, speed, torque
    # print("cdata: ", cdata)
    data = can_to_uart(data=cdata, rtr=rtr)
    # print("send_command: ", [hex(i) for i in data])
    write_data(data)
    set_angles_mode_1_flag = 0
    estop_flag = 0
    # write_data(data=can_to_uart(data=cdata, rtr=rtr))


# CAN receive function
def receive_data():
    udata = read_data(16) # A CAN message is 16 bytes in total
    if READ_FLAG == 1:
        cdata = uart_to_can(data=udata)
        return cdata[3:]

def format_data(data=[], format="f f", type='decode'):
    # print("format_data data:", data)
    # print("format_data format:", format)
    format_list = format.split() # Convert the data types into a list
    rdata = []
    if type == 'decode' and len(data) == 8:
        p = 0
        for f in format_list:
            s_f = []
            if f == 'f':
                s_f = [4, 'f']
            elif f == 'u16':
                s_f = [2, 'H']
            elif f == 's16':
                s_f = [2, 'h']
            elif f == 'u32':
                s_f = [4, 'I']
            elif f == 's32':
                s_f = [4, 'i']
            ba = bytearray()
            if len(s_f) == 2:
                for i in range(s_f[0]):
                    ba.append(data[p])
                    p = p + 1
                rdata.append(struct.unpack(s_f[1], ba)[0])
            else:
                print('unkown format in format_data(): ' + f)
                return []
        return rdata
    elif type == 'encode' and len(format_list) == len(data): # Check that the number of format types equals the number of data items
        for i in range(len(format_list)):
            f = format_list[i]
            s_f = []
            if f == 'f':
                s_f = [4, 'f'] # f means float, 4 bytes; angle is a float
            elif f == 'u16':
                s_f = [2, 'H'] # H means unsigned short, 2 bytes
            elif f == 's16':
                s_f = [2, 'h'] # h means short, 2 bytes; speed and acceleration are short integers
            elif f == 'u32':
                s_f = [4, 'I'] # I means unsigned int, 4 bytes
            elif f == 's32':
                s_f = [4, 'i'] # i means int, 4 bytes
            if len(s_f) == 2:
                bs = struct.pack(s_f[1], data[i]) # Convert the data to binary
                for j in range(s_f[0]):
                    rdata.append(bs[j]) # Pack the data byte by byte into rdara
                    # print(i, ":", rdata)
            else:
                print('unkown format in format_data(): ' + f)
                return []
        if len(rdata) < 8:
            for i in range(8 - len(rdata)):
                rdata.append(0x00)
        return rdata


# Preset angle
def preset_angle(id_num=1, angle=0, t=0, param=0, mode=0):
    """Preset the angle of a single joint.

    Presets the target angle of the joint with the given ID; the motion must then be started with an mt or mq command.

    Args:
        id_num: ID of the joint to set. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        angle: Joint angle (-360~360)*n, large rotations are supported.
        t: mode=0: no effect, just pass 0; mode=1: motion time (s); mode=2: feedforward speed (r/min).
        param: mode=0: angle input filter bandwidth (<300); mode=1: acceleration during start and stop phases ((r/min)/s); mode=2: feedforward torque (Nm).
        mode: Angle control mode. The joint supports three angle control modes:
              mode = 0: multi-joint trajectory tracking mode, for general rotation, especially suited to many input trajectory points; the angle input bandwidth should be set to half the command sending frequency.
              mode = 1: multi-joint trapezoidal trajectory mode; speed is expressed as the motion time t (s), param is the target acceleration ((r/min)/s).
              mode = 2: feedforward control mode; t is the feedforward speed and param the feedforward torque. Feedforward adds speed and torque feedforward on top of the PID control, improving response and reducing steady-state error.

    Note: In mode=1 (trapezoidal), speed and accel must both be > 0. speed=0 causes the joint to report a motor error and leave closed-loop control, so in this mode speed=0 is replaced by 0.01.
          Also, if accel=0 in this mode, the joint moves to angle at maximum speed and the speed parameter has no effect.

    Returns:
        True: ran normally
        False: an error occurred during operation

    Raises:
        "---error in set_angle---"

    """
    factor = 0.01
    if mode == 0:
        f_angle = angle
        s16_time = int(abs(t) / factor)
        if param > 300:
            print("input_filter_width = " + str(param) + ", which is too big and resized to 300")
            param = 300
        s16_width = int(abs(param / factor))
        data = format_data([f_angle, s16_time, s16_width], 'f s16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0)
    elif mode == 1:
        f_angle = angle
        s16_time = int(abs(t) / factor) # Run time
        s16_accel = int((abs(param)) / factor)
        data = format_data([f_angle, s16_time, s16_accel], 'f s16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0) # Send angle, time and acceleration to the joint
    elif mode == 2:
        f_angle = angle
        s16_speed_ff = int((t) / factor)
        s16_torque_ff = int((param) / factor)
        data = format_data([f_angle, s16_speed_ff, s16_torque_ff], 'f s16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0)


# Preset speed
def preset_speed(id_num=0, speed=10, param=0, mode=1):
    """Preset the speed of a single joint.

    Presets the target speed of the joint with the given ID; the motion must then be started with an mv command.

    Args:
        id_num: ID of the joint to set. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        speed:  Target speed (r/min)
        param:  mode=0: feedforward torque (Nm); mode!=0: target acceleration ((r/min)/s)
        mode:   Control mode:
                mode=0: speed feedforward control, the joint target speed is set directly to speed
                mode!=0: speed ramp control, the joint ramps to speed at the target acceleration.

    Note:
        In speed ramp mode, if the target acceleration is set to 0, the joint keeps its current speed.

    Returns:
        True: ran normally
        False: an error occurred during operation

    Raises:
        "---error in set_speed---"

    """
    factor = 0.01
    try:
        f_speed = speed
        if mode == 0:
            s16_torque = int((param) / factor)
            if f_speed == 0:
                s16_torque = 0
            s16_input_mode = int(1 / factor)
            data = format_data([f_speed, s16_torque, s16_input_mode], 'f s16 s16', 'encode')
        else:
            s16_ramp_rate = int((param) / factor)
            s16_input_mode = int(2 / factor)
            data = format_data([f_speed, s16_ramp_rate, s16_input_mode], 'f s16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0)
    except Exception as e:
        print("---error in preset_speed---：", e)
        return False
    return True


# Preset torque
def preset_torque(id_num=0, torque=0.1, param=0, mode=1):
    """Preset the torque of a single joint.

    Presets the target torque (Nm) of the joint with the given ID.

    Args:
        id_num: ID of the joint to set. If the current ID is unknown, 0 can be used to broadcast; if there are several joints on the bus, all of them will perform the operation.
        torque: Joint output torque (Nm)
        param: mode=0: parameter has no meaning; mode!=0: torque rise rate (Nm/s)
        mode:   Control mode:
                mode=0: direct torque control, the joint target torque is set directly to torque
                mode!=0: torque ramp control, the joint ramps to torque at the torque rise rate (Nm/s).

    Note;
        If the joint speed exceeds the speed_limit you set, the joint's output torque will be reduced.
        Set dr.controller.config.speed_limit = False to prevent the torque reduction.
        Also, in torque ramp mode, if the torque rise rate is 0, the torque stays at its current value.

    Returns:
        True: ran normally
        False: an error occurred during operation

    Raises:
        "---error in set_torque---"

    """
    factor = 0.01
    try:
        f_torque = torque
        if mode == 0:
            s16_input_mode = int(1 / factor)
            s16_ramp_rate = 0
        else:
            s16_input_mode = int(6 / factor)
            s16_ramp_rate = int((param) / factor)
        data = format_data([f_torque, s16_ramp_rate, s16_input_mode], 'f s16 s16', 'encode')
        send_command(id_num=id_num, cmd=0x0C, data=data, rtr=0)
    except Exception as e:
        print("---error in preset_torque---：", e)
        return False
    return True

def output_cpr(id_num=0):

    try:
        order_num = 0x22
        data = format_data([order_num], 'u32', 'encode')  # # Convert order_num into 4 bytes of data,
        send_command(id_num=id_num, cmd=0x08, data=data, rtr=0)  # Must be sent as a standard frame (data frame), not a remote frame
    except Exception as e:
        print("---error in set_toque_adaptive---：", e)
        return False
    return True
