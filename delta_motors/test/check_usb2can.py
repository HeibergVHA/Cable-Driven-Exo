#!/usr/bin/env python3
"""Step-by-step check of the USB2CAN adapter -> CAN bus -> DrEmpower motors chain.

Needs the real hardware. Each step prints PASS / WARN / FAIL plus a hint, and it
stops as soon as a later step could not work anyway.

    python3 test/check_usb2can.py                  # read-only checks, nothing moves
    python3 test/check_usb2can.py --move 1         # also jog motor 1 by +5 deg and back
    python3 test/check_usb2can.py --scan-bitrates  # try 1M/500k/250k/125k (uses sudo)

Steps:
  1. USB adapter is plugged in and the gs_usb driver has it
  2. can0 exists, is UP, has a bitrate, and the bus is healthy (ERROR-ACTIVE)
  3. python-can can open can0
  4. Bus is quiet (motors are not streaming state feedback, which confuses DrEmpower)
  5. Ping every CAN id 1..63 directly and list which motors answer
  6. Health of each motor: supply voltage, state (idle / closed loop), error code
  7. Read the angle through DrEmpower + can_shim (the path motor_node uses)
  8. (--move) jog a motor and check that the angle changes
"""

import argparse
import glob
import json
import os
import struct
import subprocess
import sys
import time

try:
    import can
except ImportError:
    print('python-can is missing: sudo apt install python3-can')
    sys.exit(1)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from delta_motors.parameter_interface import key_find_value, property_type  # noqa: E402

TYPE_CODE = {'f': 0, 'u16': 1, 's16': 2, 'u32': 3, 's32': 4}
TYPE_FMT = {'f': 'f', 'u16': 'H', 's16': 'h', 'u32': 'I', 's32': 'i'}
CMD_READ_PROP = 0x1E
STATE_NAMES = {1: 'IDLE (no torque - will NOT move)', 2: 'CLOSED LOOP (ready)'}
BITRATES = [1000000, 500000, 250000, 125000]

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'motors.yaml')

failed = False


def report(status, msg, hint=None):
    global failed
    colour = {'PASS': '\033[32m', 'WARN': '\033[33m', 'FAIL': '\033[31m', 'INFO': ''}[status]
    print(f'  {colour}[{status}]\033[0m {msg}')
    if hint:
        for line in hint.splitlines():
            print(f'         -> {line}')
    if status == 'FAIL':
        failed = True


def step(n, title):
    print(f'\n{n}. {title}')


def give_up():
    print('\nStopped at the first failure. Fix that and run again.')
    sys.exit(1)


def link_info(iface):
    out = subprocess.run(['ip', '-details', '-statistics', '-json', 'link', 'show', iface],
                         capture_output=True, text=True)
    if out.returncode != 0:
        return None
    return json.loads(out.stdout)[0]


def configured_ids():
    try:
        import yaml
        with open(CONFIG) as f:
            return list(yaml.safe_load(f)['delta_motors']['ros__parameters']['motor_ids'])
    except Exception:
        return None


# ---------------- raw protocol (independent of DrEmpower) ----------------

def read_request(id_num, prop):
    addr = key_find_value(prop)
    data = struct.pack('<HH', addr, TYPE_CODE[property_type[prop]]).ljust(8, b'\x00')
    return can.Message(arbitration_id=(id_num << 5) + CMD_READ_PROP, is_extended_id=False, data=data)


def raw_read(bus, id_num, prop, timeout=0.3):
    """Read one property straight over CAN; returns the value or None."""
    while bus.recv(0) is not None:  # drop anything stale
        pass
    bus.send(read_request(id_num, prop))
    addr = key_find_value(prop)
    end = time.time() + timeout
    while time.time() < end:
        m = bus.recv(end - time.time())
        if m is None:
            break
        d = bytes(m.data)
        if m.arbitration_id >> 5 == id_num and len(d) == 8 and struct.unpack('<H', d[:2])[0] == addr:
            fmt = TYPE_FMT[property_type[prop]]
            return struct.unpack('<' + fmt, d[4:4 + struct.calcsize(fmt)])[0]
    return None


def ping_all(bus, ids):
    found = set()
    for i in ids:
        try:
            bus.send(read_request(i, 'dr.config.can_id'), timeout=0.05)
        except can.CanError:
            break  # tx queue full: nobody is ACKing our frames
        time.sleep(0.002)
    end = time.time() + 0.5
    while time.time() < end:
        m = bus.recv(0.05)
        if m is not None and (m.arbitration_id & 0x1F) == CMD_READ_PROP:
            found.add(m.arbitration_id >> 5)
    return sorted(found)


# ---------------- steps ----------------

def check_usb():
    step(1, 'USB adapter + driver')
    bound = [os.path.basename(p) for p in glob.glob('/sys/bus/usb/drivers/gs_usb/*') if ':' in os.path.basename(p)]
    if bound:
        report('PASS', f'gs_usb driver is bound to {len(bound)} USB interface(s)')
    elif os.path.isdir('/sys/module/gs_usb'):
        report('FAIL', 'gs_usb driver is loaded but no adapter is attached to it',
               'Plug in the USB2CAN (try another USB port/cable), then check `dmesg | tail`.')
    else:
        report('FAIL', 'gs_usb driver not loaded',
               'Run `sudo modprobe gs_usb` (setup_can.sh does this) and replug the adapter.')


def check_interface(iface):
    step(2, f'{iface} interface')
    info = link_info(iface)
    if info is None:
        report('FAIL', f'{iface} does not exist', 'Run `bash setup_can.sh` after plugging in the adapter.')
        give_up()
    if 'UP' not in info.get('flags', []):
        report('FAIL', f'{iface} is DOWN', 'Run `bash setup_can.sh` (needed after every replug/reboot).')
        give_up()
    data = info.get('linkinfo', {}).get('info_data', {})
    bitrate = data.get('bittiming', {}).get('bitrate')
    report('PASS', f'{iface} is UP, bitrate {bitrate} bit/s')
    state = data.get('state', '?')
    berr = data.get('berr_counter', {})
    if state == 'ERROR-ACTIVE':
        report('PASS', f'bus state {state} (healthy)')
    else:
        report('WARN' if state == 'ERROR-WARNING' else 'FAIL',
               f'bus state {state}, tx errors {berr.get("tx")}, rx errors {berr.get("rx")}',
               'Frames are not being acknowledged. Usual causes:\n'
               '- motors not powered (24 V)\n'
               '- bitrate does not match the motors -> try --scan-bitrates\n'
               '- CAN_H/CAN_L swapped or loose, or missing 120 ohm termination\n'
               f'Reset the interface with `bash setup_can.sh {bitrate}` after fixing.')
    return bitrate


def open_bus(iface):
    step(3, 'python-can')
    try:
        bus = can.Bus(interface='socketcan', channel=iface)
    except Exception as e:
        report('FAIL', f'could not open {iface}: {e}')
        give_up()
    report('PASS', f'opened {iface}')
    return bus


def check_quiet(bus):
    step(4, 'Idle traffic')
    end, n, ids = time.time() + 0.5, 0, set()
    while time.time() < end:
        m = bus.recv(0.05)
        if m is not None:
            n += 1
            ids.add(m.arbitration_id >> 5)
    if n == 0:
        report('PASS', 'bus is quiet when nothing is sent')
    else:
        report('WARN', f'{n} frames in 0.5 s from id(s) {sorted(ids)} without being asked',
               'A motor is streaming real-time state feedback. DrEmpower reads the first frame it\n'
               'sees, so normal reads return wrong values. Turn it off with\n'
               'DrEmpower.disable_angle_speed_torque_state(id_num=0).')


def check_ping(bus, iface):
    step(5, 'Find motors (ping ids 1..63)')
    found = ping_all(bus, range(1, 64))
    wanted = configured_ids()
    if not found:
        info = link_info(iface) or {}
        state = info.get('linkinfo', {}).get('info_data', {}).get('state', '?')
        report('FAIL', f'no motor answered (bus state now {state})',
               'Check: motor power on? same bitrate (--scan-bitrates)? CAN_H/CAN_L wiring?\n'
               'Watch `candump can0` in another terminal while running this.')
        give_up()
    report('PASS', f'motors answering: {found}')
    if wanted is not None:
        missing = [i for i in wanted if i not in found]
        extra = [i for i in found if i not in wanted]
        if missing:
            report('FAIL', f'config/motors.yaml expects {wanted}, but {missing} did not answer',
                   'Fix motor_ids in config/motors.yaml (then rebuild) or check that motor.')
        else:
            report('PASS', f'all motor_ids from config/motors.yaml are present: {wanted}')
        if extra:
            report('INFO', f'also on the bus but not in motor_ids: {extra}')
    return found


def check_health(bus, ids):
    step(6, 'Motor health')
    for i in ids:
        v = raw_read(bus, i, 'dr.voltage')
        state = raw_read(bus, i, 'dr.current_state')
        err = raw_read(bus, i, 'dr.error')
        angle = raw_read(bus, i, 'dr.output_shaft.angle')
        print(f'  motor {i}: voltage={v if v is None else round(v, 1)} V  state={state}  '
              f'error={err if err is None else hex(err)}  angle={angle if angle is None else round(angle, 2)} deg')
        if v is not None and v < 18:
            report('FAIL', f'motor {i}: supply voltage {v:.1f} V is too low', 'Motors need 24 V.')
        if state != 2:
            report('FAIL', f'motor {i}: state {state} = {STATE_NAMES.get(state, "unknown")}',
                   f'Put it in closed loop: DrEmpower.set_mode(id_num={i}, mode=2)\n'
                   '(it drops to idle after an error, an estop or a set_mode(mode=1)).')
        else:
            report('PASS', f'motor {i}: {STATE_NAMES[2]}')
        if err:
            report('FAIL', f'motor {i}: error code {hex(err)}',
                   f'Motion is blocked while an error is latched. Clear it with\n'
                   f'DrEmpower.reboot(id_num={i}) or by power-cycling the motors.')
        elif err == 0:
            report('PASS', f'motor {i}: no error')


def check_drempower(iface, ids):
    step(7, 'DrEmpower + can_shim (what motor_node uses)')
    os.environ['DREMPOWER_CAN'] = iface
    try:
        from delta_motors import DrEmpower as dr
    except Exception as e:
        report('FAIL', f'importing DrEmpower failed: {e}')
        give_up()
    for i in ids:
        a = dr.get_angle(id_num=i)
        if a is False or a is None:
            report('FAIL', f'motor {i}: DrEmpower.get_angle failed although the raw read worked',
                   'Likely leftover/streaming frames on the bus (see step 4).')
        else:
            report('PASS', f'motor {i}: DrEmpower.get_angle = {a:.2f} deg')
    return dr


def check_move(dr, id_num, step_deg=5.0):
    step(8, f'Jog motor {id_num} by +{step_deg} deg and back')
    start = dr.get_angle(id_num=id_num)
    if start is False:
        report('FAIL', 'could not read start angle')
        return
    target = start + step_deg
    if input(f'  Motor {id_num} will move from {start:.1f} to {target:.1f} deg at 5 r/min. '
             'Is the joint free to move? [y/N] ').strip().lower() != 'y':
        report('INFO', 'skipped')
        return
    try:
        dr.set_angle(id_num=id_num, angle=target, speed=5, param=5, mode=1)
        end, now = time.time() + 5, start
        while time.time() < end:
            time.sleep(0.2)
            now = dr.get_angle(id_num=id_num)
            if now is not False and abs(now - target) < 0.5:
                break
        if now is not False and abs(now - target) < 0.5:
            report('PASS', f'reached {now:.2f} deg')
        elif now is not False and abs(now - start) > 0.5:
            report('WARN', f'moved, but only to {now:.2f} deg (target {target:.2f})',
                   'Load too high, speed limit, or angle limit in the motor config.')
        else:
            report('FAIL', f'did not move (still {now} deg)',
                   'Commands arrive but the motor ignores them: check state/error in step 6\n'
                   'and the motor angle limits (dr.config.angle_min/max).')
    finally:
        dr.set_angle(id_num=id_num, angle=start, speed=5, param=5, mode=1)
        time.sleep(2)


def scan_bitrates(iface):
    print('Trying each bitrate (needs sudo to reconfigure the interface)...')
    hit = None
    for br in BITRATES:
        for cmd in (['ip', 'link', 'set', iface, 'down'],
                    ['ip', 'link', 'set', iface, 'type', 'can', 'bitrate', str(br)],
                    ['ip', 'link', 'set', iface, 'up']):
            subprocess.run(['sudo'] + cmd, check=True)
        with can.Bus(interface='socketcan', channel=iface) as bus:
            found = ping_all(bus, range(1, 64))
        print(f'  {br:>8} bit/s: {found if found else "no answer"}')
        if found and hit is None:
            hit = br
    if hit:
        subprocess.run(['sudo', 'ip', 'link', 'set', iface, 'down'], check=True)
        subprocess.run(['sudo', 'ip', 'link', 'set', iface, 'type', 'can', 'bitrate', str(hit)], check=True)
        subprocess.run(['sudo', 'ip', 'link', 'set', iface, 'up'], check=True)
        print(f'\nMotors answer at {hit} bit/s; {iface} is left at that rate. '
              f'Use `bash setup_can.sh {hit}` from now on.')
    else:
        print('\nNo motor answered at any bitrate: check power and wiring.')
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--move', type=int, metavar='ID', help='jog this motor id by +5 deg and back')
    ap.add_argument('--scan-bitrates', action='store_true', help='find the bitrate the motors use')
    args = ap.parse_args()

    if args.scan_bitrates:
        scan_bitrates(args.iface)
        return

    check_usb()
    check_interface(args.iface)
    bus = open_bus(args.iface)
    try:
        check_quiet(bus)
        ids = check_ping(bus, args.iface)
        check_health(bus, ids)
    finally:
        bus.shutdown()
    dr = check_drempower(args.iface, ids)
    if args.move is not None:
        if args.move in ids:
            check_move(dr, args.move)
        else:
            report('FAIL', f'--move {args.move}: that motor did not answer')

    print('\n' + ('\033[31mSome checks FAILED - see hints above.\033[0m' if failed
                  else '\033[32mAll checks passed.\033[0m'))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
