"""Tests for run_exo.py against a simulated motor driver, no hardware needed.

    cd control && python3 -m pytest -q test_run_exo.py
"""

import numpy as np
import pytest

from admittance import AdmittanceParams, SingleMotorController, kin
from run_exo import CALIBRATION_CYCLES, FF_SIGN, ExoRunner, Fault, main, make_params

IDS = [7, 8]
START_DEG = np.array([100.0, -40.0])
DT = 0.005


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class FakeDriver:
    """Motors follow set_angle instantly; torque is what they need to hold the exo."""

    def __init__(self, push=(0.0, 0.0), bias=(0.0, 0.0), state=8):
        self.angle = dict(zip(IDS, START_DEG))
        self.push = np.array(push)
        self.bias = np.array(bias)
        self.state = state
        self.commands = []
        self.reads = 0
        self.fail_after: int | None = None
        self.interrupt_after: int | None = None
        self.move_on_command = True

    def read_property(self, id_num, name):
        return self.state

    def get_angle(self, id_num):
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after:
            return False
        if self.interrupt_after is not None and self.reads > self.interrupt_after:
            raise KeyboardInterrupt
        return self.angle[id_num]

    def get_torque(self, id_num):
        theta = np.radians([self.angle[i] for i in IDS] - START_DEG)
        q = kin.joint_angles_from_motors(theta)
        calibrating = self.reads <= CALIBRATION_CYCLES * len(IDS)
        push = np.zeros(2) if calibrating else self.push
        tau = kin.loop_forces_for_joint_torque(kin.gravity_torque(q) - push) * kin.SPOOL_RADIUS
        return (tau + self.bias)[IDS.index(id_num)]

    def set_angle(self, id_num, angle, speed, param, mode):
        assert mode == 2
        self.commands.append((id_num, angle, speed))
        if self.move_on_command:
            self.angle[id_num] = angle
        return True


def run(driver, enable=True, duration: float | None = 0.5, params=None):
    clock = Clock()
    runner = ExoRunner(driver, IDS, params or make_params(0.5, 20.0), enable,
                       clock=clock, sleep=clock.sleep)
    return runner, runner.run(duration)


def test_dry_run_sends_no_motion_commands():
    driver = FakeDriver(push=(0.6, 0.0))
    _, code = run(driver, enable=False)
    assert code == 0
    assert driver.commands == []


def test_first_command_is_the_current_position():
    # Nobody pushing: the exo must not jump when control starts.
    driver = FakeDriver()
    run(driver)
    for (id_num, angle, speed), start in zip(driver.commands[:2], START_DEG):
        assert np.isclose(angle, start)
        assert speed == 0.0


def test_push_moves_the_exo_at_the_expected_speed():
    params = AdmittanceParams()
    driver = FakeDriver(push=(0.4, 0.0))
    _, code = run(driver, duration=1.0, params=make_params(2.0, 90.0))
    assert code == 0
    # Check the speed in the units the motor is sent: r/min at each motor. The
    # last two commands are the final hold, so look at the two before them.
    expected = np.array([(0.4 - params.deadband[0]) / params.damping[0], 0.0])
    rpm = kin.tendon_jacobian() @ expected / kin.SPOOL_RADIUS * 60 / (2 * np.pi)
    rpm *= [FF_SIGN.get(id_num, 0.0) for id_num in IDS]
    moving = driver.commands[-4:-2]
    assert [c[0] for c in moving] == IDS
    assert np.allclose([c[2] for c in moving], rpm, rtol=0.05)


def test_torque_bias_at_rest_is_removed():
    driver = FakeDriver(bias=(0.5, -0.3))
    runner, code = run(driver)
    assert code == 0
    assert np.allclose(runner.ctrl.q_ref, 0.0, atol=1e-6)


def test_failed_read_stops_and_holds_last_position():
    driver = FakeDriver(push=(0.4, 0.0))
    driver.fail_after = 300
    _, code = run(driver, duration=None)
    assert code == 1
    last_two = driver.commands[-2:]
    assert [c[2] for c in last_two] == [0.0, 0.0]


def test_ctrl_c_holds():
    driver = FakeDriver(push=(0.4, 0.0))
    driver.interrupt_after = 300
    _, code = run(driver, duration=None)
    assert code == 0
    assert [c[2] for c in driver.commands[-2:]] == [0.0, 0.0]


def test_motor_not_following_stops_on_tracking_error():
    driver = FakeDriver(push=(2.0, 0.0))
    driver.move_on_command = False
    _, code = run(driver, duration=5.0, params=make_params(2.0, 90.0))
    assert code == 1
    # Hold is at the measured position, which never left the start.
    assert np.allclose([c[1] for c in driver.commands[-2:]], START_DEG)


def test_large_torque_estimate_stops():
    driver = FakeDriver(push=(8.0, 0.0))
    _, code = run(driver, enable=False, duration=2.0)
    assert code == 1


def test_measured_angle_outside_limits_stops():
    driver = FakeDriver()
    clock = Clock()
    runner = ExoRunner(driver, IDS, make_params(0.5, 20.0), False, clock=clock, sleep=clock.sleep)
    runner.calibrate()
    driver.angle[IDS[0]] = START_DEG[0] + 200.0
    driver.angle[IDS[1]] = START_DEG[1] + 200.0
    with pytest.raises(Fault, match="outside the limits"):
        runner.cycle(DT)


def test_slow_cycle_stops():
    driver = FakeDriver()
    clock = Clock()
    original = driver.get_angle

    def slow(id_num):
        if driver.reads > CALIBRATION_CYCLES * len(IDS) + 4:
            clock.t += 0.05
        return original(id_num)

    driver.get_angle = slow
    runner = ExoRunner(driver, IDS, make_params(0.5, 20.0), True, clock=clock, sleep=clock.sleep)
    assert runner.run(5.0) == 1


def test_motor_not_in_closed_loop_refuses_to_start():
    driver = FakeDriver(state=1)
    _, code = run(driver)
    assert code == 1
    assert driver.commands == []


RATIO = kin.PULLEY_RADII[0] / kin.SPOOL_RADIUS


class OneMotorDriver:
    """A single motor on q1 with q2 locked. Torque is what it takes to resist the push."""

    def __init__(self, push=0.0, state=8):
        self.angle = 100.0
        self.push = push
        self.state = state
        self.commands = []
        self.reads = 0

    def read_property(self, id_num, name):
        return self.state

    def get_angle(self, id_num):
        self.reads += 1
        return self.angle

    def get_torque(self, id_num):
        return 0.0 if self.reads <= CALIBRATION_CYCLES else -self.push / RATIO

    def set_angle(self, id_num, angle, speed, param, mode):
        assert id_num == 7 and mode == 2
        self.commands.append((angle, speed))
        self.angle = angle
        return True


def run_one(driver, enable=True, duration=1.0):
    clock = Clock()
    runner = ExoRunner(driver, [7], make_params(2.0, 90.0), enable, clock=clock, sleep=clock.sleep)
    return runner, runner.run(duration)


def test_one_motor_push_moves_at_the_expected_speed():
    params = AdmittanceParams()
    driver = OneMotorDriver(push=0.4)
    runner, code = run_one(driver)
    assert code == 0
    qd = (0.4 - params.deadband[0]) / params.damping[0]
    # The last command is the final hold, so look at the one before it.
    assert np.isclose(driver.commands[-2][1], FF_SIGN[7] * qd * RATIO * 60 / (2 * np.pi), rtol=0.05)
    assert driver.commands[-1][1] == 0.0
    assert runner.ctrl.q_ref[0] > 0.0
    assert runner.ctrl.q_ref[1] == 0.0


def test_one_motor_stays_still_without_a_push():
    driver = OneMotorDriver()
    _, code = run_one(driver)
    assert code == 0
    assert np.allclose([c[0] for c in driver.commands], 100.0)


def test_one_motor_dry_run_sends_nothing():
    driver = OneMotorDriver(push=0.4)
    _, code = run_one(driver, enable=False)
    assert code == 0
    assert driver.commands == []


def test_one_motor_not_in_closed_loop_refuses_to_start():
    driver = OneMotorDriver(state=1)
    _, code = run_one(driver)
    assert code == 1
    assert driver.commands == []


def test_bare_motor_away_from_zero_does_not_run_away():
    ctrl = SingleMotorController(AdmittanceParams(), DT)
    ctrl.q_ref[0] = np.radians(40.0)
    theta = ctrl.motor_reference()
    for _ in range(200):
        ctrl.step(theta, np.zeros(1))
    assert ctrl.qd_ref[0] == 0.0


def test_gravity_option_cancels_the_weight_of_the_link():
    ctrl = SingleMotorController(AdmittanceParams(), DT, gravity=True)
    ctrl.q_ref[0] = np.radians(40.0)
    theta = ctrl.motor_reference()
    q = ctrl.joint_angles(theta)
    tau = np.array([kin.gravity_torque(q)[0] / RATIO])
    assert abs(tau[0]) > 0.05
    for _ in range(200):
        ctrl.step(theta, tau)
    assert np.isclose(ctrl.qd_ref[0], 0.0, atol=1e-6)
    # Same torque with the model left out reads as a push, which is why it matters.
    plain = SingleMotorController(AdmittanceParams(), DT)
    plain.step(theta, tau)
    assert plain.tau_human[0] != 0.0


@pytest.mark.parametrize("args", [["--ids", "7", "8", "9"], ["--ids", "7", "7"],
                                  ["--ids", "7", "8", "--gravity"],
                                  ["--ids", "7", "--damping", "0"],
                                  ["--ids", "7", "--mass", "-1"],
                                  ["--ids", "7", "--deadband", "-0.1"]])
def test_bad_arguments_are_refused(args):
    with pytest.raises(SystemExit):
        main(args)


def test_lower_damping_moves_faster_for_the_same_push():
    speeds = []
    for damping in (1.5, 0.5):
        clock = Clock()
        driver = OneMotorDriver(push=0.2)
        params = make_params(2.0, 90.0, damping=damping)
        ExoRunner(driver, [7], params, True, clock=clock, sleep=clock.sleep).run(1.0)
        speeds.append(driver.commands[-2][1])
    assert np.isclose(speeds[1], 3.0 * speeds[0], rtol=0.05)


def test_log_has_one_row_per_cycle_and_survives_a_fault(tmp_path):
    path = tmp_path / "run.csv"
    clock = Clock()
    driver = OneMotorDriver(push=0.2)
    real_get_angle = driver.get_angle
    calls = {"n": 0}

    def fail_later(id_num):
        calls["n"] += 1
        return False if calls["n"] > CALIBRATION_CYCLES + 20 else real_get_angle(id_num)

    driver.get_angle = fail_later
    runner = ExoRunner(driver, [7], make_params(2.0, 90.0), True, clock=clock, sleep=clock.sleep,
                       log_path=path)
    assert runner.run(None) == 1
    lines = path.read_text().splitlines()
    assert lines[0].split(",")[:3] == ["time_s", "theta0_deg", "theta_ref0_deg"]
    assert len(lines) == 21
    assert all(len(line.split(",")) == 8 for line in lines)


def test_feed_forward_follows_the_sign_of_each_motor(monkeypatch):
    monkeypatch.setitem(FF_SIGN, 7, -1.0)
    monkeypatch.setitem(FF_SIGN, 8, 1.0)
    driver = FakeDriver()
    clock = Clock()
    runner = ExoRunner(driver, IDS, make_params(0.5, 20.0), True, clock=clock, sleep=clock.sleep)
    runner.send(np.zeros(2), np.array([1.0, 1.0]))
    rpm = 60 / (2 * np.pi)
    assert np.allclose([c[2] for c in driver.commands], [-rpm, rpm])


def test_motor_without_a_measured_sign_gets_no_feed_forward(monkeypatch, capsys):
    monkeypatch.delitem(FF_SIGN, 8, raising=False)
    driver = FakeDriver()
    clock = Clock()
    runner = ExoRunner(driver, IDS, make_params(0.5, 20.0), True, clock=clock, sleep=clock.sleep)
    assert "motor(s) [8]" in capsys.readouterr().out
    runner.send(np.zeros(2), np.array([1.0, 1.0]))
    assert driver.commands[1][2] == 0.0


def test_tracking_error_above_three_degrees_stops():
    driver = OneMotorDriver()
    clock = Clock()
    runner = ExoRunner(driver, [7], make_params(0.5, 20.0), True, clock=clock, sleep=clock.sleep)
    runner.calibrate()
    driver.angle = 100.0 + 3.5 * RATIO      # 3.5 deg at the joint
    with pytest.raises(Fault, match="tracking error"):
        runner.cycle(0.005)
