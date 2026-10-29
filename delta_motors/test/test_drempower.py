"""Offline tests for DrEmpower.py + can_shim.py -- no adapter or motors needed.

DrEmpower talks to a simulated motor over a python-can *virtual* bus instead of
can0, so every test goes through the real code path:

    DrEmpower -> 16-byte serial packet -> CanSerialShim -> CAN frame -> FakeMotor
    FakeMotor -> CAN reply -> CanSerialShim -> 16-byte packet -> DrEmpower

Run:   cd delta_motors && python3 -m pytest test/test_drempower.py -v
"""

import os
import struct
import sys
import threading
import time

import can
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from delta_motors.can_shim import CanSerialShim  # noqa: E402
from delta_motors.parameter_interface import value_find_key  # noqa: E402

CHANNEL = 'drempower_test'

# DrEmpower's type codes (data_types in read_property/write_property) -> struct format
TYPE_FMT = {0: 'f', 1: 'H', 2: 'h', 3: 'I', 4: 'i'}

CMD_ORDER = 0x08          # order_num commands (estop, set_angles trigger, ...)
CMD_PRESET_ANGLE = 0x0C   # preset_angle, executed by a later 0x08 order
CMD_SET_ANGLE_TRAP = 0x1A # set_angle mode=1 (trapezoidal)
CMD_READ_PROP = 0x1E
CMD_WRITE_PROP = 0x1F
ORDER_ESTOP = 0x06
ORDER_RUN_PRESET_TRAP = 0x11


class FakeMotor(threading.Thread):
    """Simulated DrEmpower joints on the virtual bus. Moves happen instantly."""

    def __init__(self, ids):
        super().__init__(daemon=True)
        self.bus = can.Bus(interface='virtual', channel=CHANNEL)
        self.props = {i: {'dr.output_shaft.angle': 0.0,
                          'dr.current_state': 2,  # closed loop
                          'dr.error': 0,
                          'dr.voltage': 24.0,
                          'dr.controller.position_done': 1,
                          'dr.config.can_id': i} for i in ids}
        self.presets = {}
        self.estopped = set()
        self.frames = []          # every frame received, for assertions
        self.silent = False       # True = act like an unpowered motor
        self._halt = threading.Event()

    def run(self):
        while not self._halt.is_set():
            msg = self.bus.recv(0.02)
            if msg is not None:
                self.frames.append(msg)
                if not self.silent:
                    self._handle(msg)

    def stop(self):
        self._halt.set()
        self.join()
        self.bus.shutdown()

    def _targets(self, id_num):
        return list(self.props) if id_num == 0 else [id_num] if id_num in self.props else []

    def _handle(self, msg):
        id_num, cmd, d = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F, bytes(msg.data)
        for i in self._targets(id_num):
            p = self.props[i]
            if cmd == CMD_READ_PROP:
                addr, t = struct.unpack('<HH', d[:4])
                value = p.get(value_find_key(addr), 0)
                reply = struct.pack('<HH' + TYPE_FMT[t], addr, t, value)
                self.bus.send(can.Message(arbitration_id=(i << 5) + cmd, is_extended_id=False,
                                          data=reply.ljust(8, b'\x00')))
            elif cmd == CMD_WRITE_PROP:
                addr, t = struct.unpack('<HH', d[:4])
                p[value_find_key(addr)] = struct.unpack('<' + TYPE_FMT[t], d[4:4 + struct.calcsize(TYPE_FMT[t])])[0]
            elif cmd == CMD_SET_ANGLE_TRAP:
                p['dr.output_shaft.angle'] = struct.unpack('<fhh', d)[0]
            elif cmd == CMD_PRESET_ANGLE:
                self.presets[i] = struct.unpack('<fhh', d)[0]
            elif cmd == CMD_ORDER:
                order = struct.unpack('<I', d[:4])[0]
                if order == ORDER_ESTOP:
                    self.estopped.add(i)
                elif order == ORDER_RUN_PRESET_TRAP and i in self.presets:
                    p['dr.output_shaft.angle'] = self.presets.pop(i)


@pytest.fixture(scope='module')
def motor():
    m = FakeMotor(ids=[1, 2])
    m.start()
    yield m
    m.stop()


@pytest.fixture(scope='module')
def dr(motor):
    # DrEmpower opens its bus at import time; point the shim at the virtual bus.
    os.environ['DREMPOWER_CAN'] = CHANNEL
    real_bus = can.interface.Bus
    can.interface.Bus = lambda channel, interface=None, **kw: real_bus(channel=channel, interface='virtual', **kw)
    try:
        from delta_motors import DrEmpower
    finally:
        can.interface.Bus = real_bus
    yield DrEmpower
    DrEmpower.uart.bus.shutdown()


@pytest.fixture(autouse=True)
def clean(request):
    if 'motor' in request.fixturenames:
        m = request.getfixturevalue('motor')
        m.frames.clear()
        m.silent = False
    yield


def wait_for(cond, timeout=1.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


# ---------------- pure encoding (no bus) ----------------

@pytest.mark.parametrize('fmt,values', [
    ('f s16 s16', [12.5, -300, 1000]),
    ('u16 u16 u32', [38001, 3, 123456]),
    ('u32', [6]),
    ('s32', [-42]),
])
def test_format_data_roundtrip(dr, fmt, values):
    encoded = dr.format_data(values, fmt, 'encode')
    assert len(encoded) == 8
    decoded = dr.format_data(encoded, fmt, 'decode')
    assert decoded[:len(values)] == pytest.approx(values)


def test_can_to_uart_packet_layout(dr):
    # id 3, cmd 0x1A -> arbitration id (3 << 5) + 0x1A = 0x7A
    cdata = [0x08, 0x00, 0x7A, 1, 2, 3, 4, 5, 6, 7, 8]
    p = dr.can_to_uart(cdata, rtr=0)
    assert p[:4] == [0xAA, 0, 0, 8]           # header, standard id, data frame, dlc 8
    assert p[4:8] == [0, 0, 0x00, 0x7A]        # arbitration id, big-endian
    assert p[8:] == [1, 2, 3, 4, 5, 6, 7, 8]
    assert dr.can_to_uart(cdata, rtr=1)[2] == 1


def test_shim_packet_frame_roundtrip():
    packet = bytes([0xAA, 0, 0, 8, 0, 0, 0x00, 0x7A, 1, 2, 3, 4, 5, 6, 7, 8])
    frame = CanSerialShim._packet_to_frame(list(packet))
    assert frame.arbitration_id == 0x7A
    assert not frame.is_extended_id and not frame.is_remote_frame
    assert bytes(frame.data) == packet[8:]
    assert CanSerialShim._frame_to_packet(frame) == packet


# ---------------- DrEmpower <-> shim <-> simulated motor ----------------

def test_set_angle_sends_correct_frame(dr, motor):
    assert dr.set_angle(id_num=1, angle=45.0, speed=10, param=10, mode=1) is True
    assert wait_for(lambda: motor.frames)
    f = motor.frames[0]
    assert f.arbitration_id == (1 << 5) + CMD_SET_ANGLE_TRAP
    angle, speed, accel = struct.unpack('<fhh', bytes(f.data))
    assert (angle, speed, accel) == (45.0, 1000, 1000)  # speed/accel are sent x100


def test_set_angle_then_get_angle(dr, motor):
    dr.set_angle(id_num=2, angle=-30.0, speed=10, param=10, mode=1)
    assert wait_for(lambda: motor.props[2]['dr.output_shaft.angle'] == -30.0)
    assert dr.get_angle(id_num=2) == pytest.approx(-30.0)


def test_read_property_u32(dr, motor):
    motor.props[1]['dr.error'] = 0x42
    assert dr.read_property(id_num=1, property='dr.error') == 0x42
    assert dr.read_property(id_num=1, property='dr.current_state') == 2
    motor.props[1]['dr.error'] = 0


def test_read_voltage(dr):
    assert dr.read_property(id_num=1, property='dr.voltage') == pytest.approx(24.0)


def test_write_property(dr, motor):
    dr.write_property(id_num=1, property='dr.requested_state', value=1)
    assert wait_for(lambda: motor.props[1].get('dr.requested_state') == 1)


def test_unknown_property_rejected(dr, motor):
    assert dr.read_property(id_num=1, property='dr.does_not_exist') is False
    time.sleep(0.05)
    assert not motor.frames  # nothing was sent


def test_estop_broadcast(dr, motor):
    motor.estopped.clear()
    assert dr.estop(id_num=0) is True
    assert wait_for(lambda: motor.estopped == {1, 2})


def test_set_angles_unsorted_ids(dr, motor):
    # Each id must get its own target; uses the get_angle path (state feedback off)
    motor.props[1]['dr.output_shaft.angle'] = 0.0
    motor.props[2]['dr.output_shaft.angle'] = 0.0
    assert dr.set_angles(id_list=[2, 1], angle_list=[20.0, 10.0], speed=10, param=10, mode=1) is True
    assert wait_for(lambda: motor.props[1]['dr.output_shaft.angle'] == 10.0
                    and motor.props[2]['dr.output_shaft.angle'] == 20.0)


def test_position_done(dr):
    assert dr.position_done(id_num=1) is True


def test_no_reply_times_out_instead_of_hanging(dr, motor):
    motor.silent = True
    t0 = time.time()
    assert dr.get_angle(id_num=1) is False
    assert time.time() - t0 < 2.0
