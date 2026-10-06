"""Serial-port lookalike on top of SocketCAN (python-can).

DrEmpower.py was written for Daran's USB-CAN module, which appears as a serial
port and wraps every CAN frame in a 16-byte serial packet:

    [0xAA, IDE, RTR, DLC, id3, id2, id1, id0, d0, d1, d2, d3, d4, d5, d6, d7]

Adapters like the INNO-MAKER USB2CAN are SocketCAN devices (can0) instead.
CanSerialShim implements the handful of serial methods DrEmpower uses
(write, read, inWaiting, flushInput, close, open) and converts between those
16-byte packets and real CAN frames, so the rest of DrEmpower.py is unchanged.
"""

import can

PACKET_LEN = 16
HEADER = 0xAA


class CanSerialShim:
    def __init__(self, channel='can0', timeout=0.5):
        self.bus = can.interface.Bus(channel=channel, interface='socketcan')
        self.timeout = timeout
        self._rx = bytearray()  # received frames, already encoded as 16-byte packets

    # ---- packet <-> frame conversion ----
    @staticmethod
    def _packet_to_frame(p):
        arb_id = (p[4] << 24) | (p[5] << 16) | (p[6] << 8) | p[7]
        is_ext = p[1] == 1
        is_rtr = p[2] == 1
        dlc = p[3]
        return can.Message(arbitration_id=arb_id, is_extended_id=is_ext,
                           is_remote_frame=is_rtr, dlc=dlc,
                           data=None if is_rtr else bytes(p[8:8 + dlc]))

    @staticmethod
    def _frame_to_packet(m):
        a = m.arbitration_id
        data = bytes(m.data).ljust(8, b'\x00')[:8]
        return bytes([HEADER, int(m.is_extended_id), int(m.is_remote_frame), 8,
                      (a >> 24) & 0xFF, (a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF]) + data

    def _pump(self, timeout=0.0):
        """Move any frames waiting on the bus into the receive buffer."""
        msg = self.bus.recv(timeout)
        while msg is not None:
            if not msg.is_error_frame:
                self._rx += self._frame_to_packet(msg)
            msg = self.bus.recv(0)

    # ---- the serial.Serial API that DrEmpower.py uses ----
    def write(self, data):
        p = list(data)
        if len(p) != PACKET_LEN or p[0] != HEADER:
            return 0
        self.bus.send(self._packet_to_frame(p))
        return PACKET_LEN

    def inWaiting(self):
        self._pump(0.0)
        return len(self._rx)

    in_waiting = property(inWaiting)

    def read(self, n=1):
        if len(self._rx) < n:
            self._pump(self.timeout)
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def flushInput(self):
        self._pump(0.0)
        self._rx.clear()

    reset_input_buffer = flushInput

    def close(self):
        pass  # keep the bus open; DrEmpower only closes to immediately reopen

    def open(self):
        pass
