"""Measure how fast the admittance control loop can run.

One cycle = read every motor, run the controller, send the commands.

    python3 control/bench_loop.py                 # fake CAN bus, no hardware
    python3 control/bench_loop.py --real          # real motors, READ-ONLY
    python3 control/bench_loop.py --real --ids 7 8

--real never sends a motion command
"""

import argparse
import os
import sys
import time
from collections import deque
from pathlib import Path

import can
import numpy as np

from admittance import AdmittanceController, AdmittanceParams

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "delta_motors"))

CONTROL_HZ = 200          # target rate of the control loop [Hz]
WIRE_TIME = 130e-6        # one 8-byte CAN frame at 1 Mbit/s [s]
CMD_READ_PROPERTY = 0x1E
MAX_FAILED_READS = 5


class FakeBus:
    """Stands in for can.interface.Bus: motors answer read requests with zeros."""

    delay = 0.0           # motor + USB adapter delay per reply [s]

    def __init__(self, **_):
        self.queue = deque()

    def send(self, msg):
        if msg.arbitration_id & 0x1F == CMD_READ_PROPERTY:
            # Like the real firmware, replies leave the address bytes at zero.
            reply = can.Message(arbitration_id=msg.arbitration_id, is_extended_id=False,
                                data=bytes(2) + bytes(msg.data[2:4]) + bytes(4))
            self.queue.append((time.perf_counter() + 2 * WIRE_TIME + self.delay, reply))

    def recv(self, timeout=None):
        end = time.perf_counter() + (timeout or 0.0)
        while True:
            if self.queue and self.queue[0][0] <= time.perf_counter():
                return self.queue.popleft()[1]
            if time.perf_counter() >= end:
                return None
            time.sleep(20e-6)


def read_motors(dr, ids):
    """Angle [deg], speed [r/min] and torque [Nm] per motor, or None on a failed read."""
    states = []
    for id_num in ids:
        state = dr.get_state(id_num)
        torque = dr.get_torque(id_num)
        # get_state returns [False, False] on a failed read. Compare with `is`,
        # because a real reading of 0.0 also equals False.
        if state is False or torque is False or any(v is False for v in state):
            return None
        states.append((state[0], state[1], torque))
    return np.array(states, dtype=float)


def run_controller(ctrl, states):
    """One controller step. Missing motors read as zero, so a single motor can be tested."""
    theta_m = np.zeros(2)
    tau_m = np.zeros(2)
    n = min(len(states), 2)
    theta_m[:n] = np.radians(states[:n, 0])
    tau_m[:n] = states[:n, 2]
    return ctrl.step(theta_m, tau_m)


def send_commands(dr, ids, theta_ref):
    for id_num, angle in zip(ids, np.degrees(theta_ref)):
        dr.set_angle(id_num=id_num, angle=float(angle), speed=0.0, param=0.0, mode=2)


def summary(seconds):
    ms = np.array(seconds) * 1e3
    return (f"median {np.median(ms):6.2f} ms   p99 {np.percentile(ms, 99):6.2f} ms   "
            f"max {ms.max():6.2f} ms")


def measure(dr, ids, cycles, real):
    """Run the loop flat out, then paced at CONTROL_HZ. Returns False if reads fail."""
    dt = 1.0 / CONTROL_HZ
    ctrl = AdmittanceController(AdmittanceParams(), dt, np.zeros(2))
    reads, steps, totals = [], [], []
    overruns = 0
    failed = 0
    failed_total = 0
    start = time.perf_counter()

    for k in range(2 * cycles):
        paced = k >= cycles
        t0 = time.perf_counter()
        states = read_motors(dr, ids)
        if states is None:
            failed += 1
            failed_total += 1
            if failed >= MAX_FAILED_READS:
                print(f"Stopped: {failed} failed reads in a row. Are the motor IDs {ids} right?")
                return False
            continue
        failed = 0
        t1 = time.perf_counter()
        theta_ref = run_controller(ctrl, states)
        t2 = time.perf_counter()
        if not real:
            send_commands(dr, ids, theta_ref)
        t3 = time.perf_counter()

        if not paced:
            reads.append(t1 - t0)
            steps.append(t2 - t1)
            totals.append(t3 - t0)
            start = t3
            continue
        slack = start + (k - cycles + 1) * dt - time.perf_counter()
        if slack > 0:
            time.sleep(slack)
        else:
            overruns += 1

    print(f"  read {len(ids)} motor(s)  {summary(reads)}")
    print(f"  controller      {summary(steps)}")
    print(f"  full cycle      {summary(totals)}")
    print(f"  flat-out rate   {1.0 / np.median(totals):.0f} Hz")
    print(f"  paced at {CONTROL_HZ} Hz   missed {overruns}/{cycles} deadlines")
    print(f"  failed reads    {failed_total}")
    if len(ids) < 2:
        two_motors = 2 * np.median(reads) + np.median(steps)
        print(f"  estimate for 2 motors: {1.0 / two_motors:.0f} Hz")
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or "").partition("\n")[0])
    parser.add_argument("--real", action="store_true", help="use the real CAN bus, read-only")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface for --real")
    parser.add_argument("--ids", type=int, nargs="+", default=None,
                        help="motor CAN IDs (default: 7 with --real, 7 8 on the fake bus)")
    parser.add_argument("--cycles", type=int, default=1000, help="cycles per measurement")
    parser.add_argument("--delay", type=float, nargs="+", default=[0.0, 0.5, 1.0],
                        help="fake bus only: delays per reply to try [ms]")
    args = parser.parse_args(argv)

    if args.real:
        os.environ["DREMPOWER_CAN"] = args.interface
    else:
        os.environ["DREMPOWER_CAN"] = "fake"
        can.interface.Bus = FakeBus

    try:
        # DrEmpower opens the bus on import.
        from delta_motors import DrEmpower as dr
    except OSError as e:
        print(f"Could not open {args.interface}: {e}\n"
              "Is the adapter plugged in and the bus up? See delta_motors/setup_can.sh")
        return 1

    if args.real:
        ids = args.ids or [7]
        print(f"REAL bus {args.interface}, motors {ids}, read-only: nothing will move")
        return 0 if measure(dr, ids, args.cycles, real=True) else 1

    ids = args.ids or [7, 8]
    print(f"FAKE bus, motors {ids}: the delay per reply is a guess, use --real for true figures")
    for delay in args.delay:
        FakeBus.delay = delay * 1e-3
        print(f"\n+{delay:.1f} ms per reply")
        measure(dr, ids, args.cycles, real=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
