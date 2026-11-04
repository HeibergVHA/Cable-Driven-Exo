"""Run the admittance controller on the real exo, so it follows the arm.

    python3 control/run_exo.py --ids 7 8            # dry run: no motion commands
    python3 control/run_exo.py --ids 7 8 --enable   # moves the motors
    python3 control/run_exo.py --ids 7 --enable     # one motor, see below

With one motor, it drives q1 and q2 must be locked at 0. Add --gravity if the
arm link is attached and you want its weight compensated. Leave it out for a
motor without the link.

Start with the arm hanging straight down (q = 0) and nobody pushing: the start
position becomes the zero and the torque read at start is treated as bias.

Any fault holds the motors at their measured position and ends the program.
This is not an emergency stop: keep a power switch within reach.
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

from admittance import AdmittanceController, AdmittanceParams, SingleMotorController

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "delta_motors"))

CALIBRATION_CYCLES = 50
PRINT_PERIOD = 0.5                      # [s]
MAX_CYCLE_TIME = 0.03                   # longest allowed cycle [s]
MAX_TRACKING_ERROR = np.radians(10.0)   # reference vs measured joint angle [rad]
MAX_HUMAN_TORQUE = 5.0                  # estimated human torque [Nm]
LIMIT_MARGIN = np.radians(5.0)          # measured angle past the limits [rad]
CLOSED_LOOP_STATES = (2, 8)             # dr.current_state: firmware reads 8, library docs say 2


class Fault(Exception):
    """A safety check failed: hold the motors and stop."""


class ExoRunner:
    def __init__(self, dr, ids, params, enable, rate_hz=200.0,
                 clock=time.perf_counter, sleep=time.sleep, gravity=False, log_path=None):
        self.dr = dr
        self.ids = ids
        self.params = params
        self.enable = enable
        self.period = 1.0 / rate_hz
        self.clock = clock
        self.sleep = sleep
        if len(ids) == 1:
            self.ctrl = SingleMotorController(params, self.period, gravity)
        else:
            self.ctrl = AdmittanceController(params, self.period, np.zeros(len(ids)))
        self.theta_zero = np.zeros(len(ids))
        self.tau_bias = np.zeros(len(ids))
        self.theta_m = None                 # last measured angle, zeroed [rad]
        self.log_path = log_path
        self.log_rows = []

    def read(self):
        """Motor angles [rad, not zeroed] and torques [Nm]. A failed read is a fault."""
        angles, torques = [], []
        for id_num in self.ids:
            angle = self.dr.get_angle(id_num)
            torque = self.dr.get_torque(id_num)
            # Compare with `is`: a real reading of 0.0 also equals False.
            if angle is False or torque is False or angle is None or torque is None:
                raise Fault(f"failed to read motor {id_num}")
            angles.append(angle)
            torques.append(torque)
        return np.radians(angles), np.array(torques, dtype=float)

    def calibrate(self):
        """Store the start position as q = 0 and the resting torque as bias."""
        for id_num in self.ids:
            state = self.dr.read_property(id_num, "dr.current_state")
            if self.enable and state not in CLOSED_LOOP_STATES:
                raise Fault(f"motor {id_num} is not in closed loop (state {state})")
        readings = [self.read() for _ in range(CALIBRATION_CYCLES)]
        self.theta_zero = np.mean([r[0] for r in readings], axis=0)
        self.tau_bias = np.mean([r[1] for r in readings], axis=0)
        self.theta_m = np.zeros(len(self.ids))
        print("start angles [deg]:", np.round(np.degrees(self.theta_zero), 2),
              " torque bias [Nm]:", np.round(self.tau_bias, 3))

    def send(self, theta_ref, theta_dot_ref):
        """Position plus speed feedforward. Only ever called with --enable."""
        angles = np.degrees(theta_ref + self.theta_zero)
        speeds = theta_dot_ref * 60.0 / (2.0 * np.pi)
        for id_num, angle, speed in zip(self.ids, angles, speeds):
            if self.dr.set_angle(id_num=id_num, angle=float(angle), speed=float(speed),
                                 param=0.0, mode=2) is False:
                raise Fault(f"failed to send to motor {id_num}")

    def hold(self):
        """Hold the last measured position with zero speed."""
        self.ctrl.hold()
        if self.enable and self.theta_m is not None:
            try:
                self.send(self.theta_m, np.zeros(len(self.ids)))
            except Fault as e:
                print(f"COULD NOT HOLD ({e}): use the power switch")

    def check(self, q, tau_human):
        p = self.params
        if np.any(q < p.q_min - LIMIT_MARGIN) or np.any(q > p.q_max + LIMIT_MARGIN):
            raise Fault(f"joint angle {np.round(np.degrees(q), 1)} deg is outside the limits")
        if np.any(np.abs(tau_human) > MAX_HUMAN_TORQUE):
            raise Fault(f"human torque estimate {np.round(tau_human, 2)} Nm is too large")
        # In a dry run nothing tracks the reference, so this check would always trip.
        if self.enable and np.any(np.abs(self.ctrl.q_ref - q) > MAX_TRACKING_ERROR):
            raise Fault(f"tracking error: reference {np.round(np.degrees(self.ctrl.q_ref), 1)}"
                        f" deg, measured {np.round(np.degrees(q), 1)} deg")

    def cycle(self, dt):
        theta, tau = self.read()
        self.theta_m = theta - self.theta_zero
        theta_ref = self.ctrl.step(self.theta_m, tau - self.tau_bias, dt)
        if self.log_path:
            self.log_rows.append([self.clock(), *np.degrees(self.theta_m), *np.degrees(theta_ref),
                                  *(tau - self.tau_bias), *self.ctrl.tau_human, *self.ctrl.qd_ref])
        self.check(self.ctrl.joint_angles(self.theta_m), self.ctrl.tau_human)
        if self.enable:
            self.send(theta_ref, self.ctrl.motor_speeds())

    def run(self, duration=None):
        """Run until a fault, Returns 0 on a clean end."""
        code = 0
        cycles = 0
        start = last = 0.0
        try:
            self.calibrate()
            start = last = next_time = next_print = self.clock()
            while duration is None or last - start < duration:
                next_time += self.period
                remaining = next_time - self.clock()
                if remaining > 0:
                    self.sleep(remaining)
                else:
                    next_time = self.clock()    # running late: do not try to catch up
                now = self.clock()
                dt, last = now - last, now
                if dt > MAX_CYCLE_TIME:
                    raise Fault(f"cycle took {dt * 1e3:.1f} ms")
                self.cycle(dt)
                cycles += 1
                if now >= next_print:
                    next_print = now + PRINT_PERIOD
                    self.print_status(cycles / max(now - start, 1e-9))
        except KeyboardInterrupt:
            print("\nStopped by Ctrl+C")
        except Fault as e:
            print(f"\nFAULT: {e}")
            code = 1
        finally:
            self.hold()
            self.write_log()
        if cycles:
            print(f"{cycles} cycles, average {cycles / max(last - start, 1e-9):.0f} Hz")
        return code

    def write_log(self):
        """Save after the run, so writing the file cannot slow the control loop."""
        if not self.log_path or not self.log_rows:
            return
        n, joints = len(self.ids), len(self.ctrl.q_ref)
        header = (["time_s"] + [f"theta{i}_deg" for i in range(n)]
                  + [f"theta_ref{i}_deg" for i in range(n)] + [f"torque{i}_Nm" for i in range(n)]
                  + [f"human{i}_Nm" for i in range(joints)]
                  + [f"qd_ref{i}_rad_s" for i in range(joints)])
        rows = np.array(self.log_rows)
        rows[:, 0] -= rows[0, 0]
        np.savetxt(self.log_path, rows, delimiter=",", header=",".join(header), comments="",
                   fmt="%.6f")
        print(f"log saved to {self.log_path}")

    def print_status(self, rate):
        q = self.ctrl.joint_angles(self.theta_m)
        print(f"q {np.round(np.degrees(q), 1)} deg   "
              f"q_ref {np.round(np.degrees(self.ctrl.q_ref), 1)} deg   "
              f"human {np.round(self.ctrl.tau_human, 2)} Nm   {rate:.0f} Hz")


def make_params(max_speed, range_deg, damping=None, mass=None, deadband=None):
    """Default controller settings with tighter limits and optional tuning, per joint."""
    params = AdmittanceParams()
    if damping is not None:
        params.damping = np.full(2, damping)
    if mass is not None:
        params.mass = np.full(2, mass)
    if deadband is not None:
        params.deadband = np.full(2, deadband)
    limit = np.radians(range_deg)
    params.qd_max = np.minimum(params.qd_max, max_speed)
    params.q_min = np.maximum(params.q_min, -limit)
    params.q_max = np.minimum(params.q_max, limit)
    return params


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or "").partition("\n")[0])
    parser.add_argument("--ids", type=int, nargs="+", required=True, metavar="ID",
                        help="CAN IDs of the motors driving loop 1 and loop 2, "
                             "or one ID to run a single motor on q1")
    parser.add_argument("--gravity", action="store_true",
                        help="one motor only: compensate the weight of the attached arm link")
    parser.add_argument("--enable", action="store_true", help="send motion commands")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument("--rate", type=float, default=200.0, help="target loop rate [Hz]")
    parser.add_argument("--max-speed", type=float, default=0.5, help="joint speed limit [rad/s]")
    parser.add_argument("--range-deg", type=float, default=20.0, help="joint angle limit [deg]")
    parser.add_argument("--damping", type=float, default=None,
                        help="joint damping [Nm s/rad], lower feels lighter but can shake")
    parser.add_argument("--mass", type=float, default=None,
                        help="virtual joint inertia [kg m^2]")
    parser.add_argument("--deadband", type=float, default=None,
                        help="joint torque ignored around zero [Nm], higher shakes less")
    parser.add_argument("--log", default=None, metavar="FILE",
                        help="save every control cycle to this CSV file")
    parser.add_argument("--duration", type=float, default=None, help="stop after this time [s]")
    args = parser.parse_args(argv)

    if len(args.ids) not in (1, 2) or len(set(args.ids)) != len(args.ids):
        parser.error("--ids needs one motor or two different motors")
    if args.gravity and len(args.ids) != 1:
        parser.error("--gravity is only for one motor: two motors always use the exo model")

    for name in ("damping", "mass"):
        value = getattr(args, name)
        if value is not None and value <= 0:
            parser.error(f"--{name} must be above 0")
    if args.deadband is not None and args.deadband < 0:
        parser.error("--deadband cannot be negative")

    os.environ["DREMPOWER_CAN"] = args.interface
    try:
        from delta_motors import DrEmpower as dr
    except OSError as e:
        print(f"Could not open {args.interface}: {e}\n"
              "Is the adapter plugged in and the bus up? See delta_motors/setup_can.sh")
        return 1

    if args.enable:
        print("MOTORS WILL MOVE. Arm hanging at q = 0.")
        if input("Type yes to start: ").strip().lower() != "yes":
            print("Not started")
            return 1
    else:
        print("DRY RUN: no motion commands are sent. Use --enable to move the motors.")

    runner = ExoRunner(dr, args.ids, make_params(args.max_speed, args.range_deg, args.damping, args.mass, args.deadband),
                       args.enable,
                       args.rate, gravity=args.gravity, log_path=args.log)
    return runner.run(args.duration)


if __name__ == "__main__":
    sys.exit(main())
