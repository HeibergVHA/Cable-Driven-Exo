"""Tests for the dynamics and the admittance controller.

    cd control && python3 -m pytest -q
"""

import numpy as np

from admittance import AdmittanceController, AdmittanceParams, kin

DT = 0.005


def total_energy(q, qd):
    """Kinetic + potential energy, computed from Jacobians (not from RNEA)."""
    energy = 0.0
    frames = kin.forward_kinematics(q)
    for k, link in enumerate(kin.LINKS):
        J = kin.jacobian(q, link.com, up_to=k)
        v, w = J[:3] @ qd, J[3:] @ qd
        R, p = frames[k]
        I_w = R @ link.inertia @ R.T
        energy += 0.5 * link.mass * v @ v + 0.5 * w @ I_w @ w
        energy -= link.mass * kin.GRAVITY @ (p + R @ link.com)
    return energy


def test_free_swing_conserves_energy():
    q = np.radians([40.0, 60.0])
    qd = np.array([1.0, -0.5])
    e0 = total_energy(q, qd)
    dt = 1e-3
    zero = np.zeros(2)
    for _ in range(1000):  # RK4, 1 s
        def f(x):
            return np.concatenate([x[2:], kin.forward_dynamics(x[:2], x[2:], zero)])
        x = np.concatenate([q, qd])
        k1 = f(x)
        k2 = f(x + dt / 2 * k1)
        k3 = f(x + dt / 2 * k2)
        k4 = f(x + dt * k3)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        q, qd = x[:2], x[2:]
    assert abs(total_energy(q, qd) - e0) < 1e-6 * max(1.0, abs(e0))


def jacobian_mass_matrix(q):
    """M(q) = sum of m Jv^T Jv + Jw^T I Jw, independent of RNEA."""
    M = np.zeros((2, 2))
    frames = kin.forward_kinematics(q)
    for k, link in enumerate(kin.LINKS):
        J = kin.jacobian(q, link.com, up_to=k)
        R = frames[k][0]
        M += link.mass * J[:3].T @ J[:3] + J[3:].T @ (R @ link.inertia @ R.T) @ J[3:]
    return M + kin.reflected_motor_inertia()


def test_rnea_matches_lagrangian():
    # Energy conservation cannot see workless (gyroscopic) errors, so check
    # tau = M qdd + Mdot qd - 1/2 d/dq (qd^T M qd) + g term by term.
    q = np.radians([25.0, 55.0])
    qd = np.array([1.2, -0.7])
    qdd = np.array([0.4, 1.1])
    eps = 1e-6
    M = jacobian_mass_matrix(q)
    M_dot = (jacobian_mass_matrix(q + eps * qd) - jacobian_mass_matrix(q - eps * qd)) / (2 * eps)
    grad = np.array([
        qd @ (jacobian_mass_matrix(q + eps * e) - jacobian_mass_matrix(q - eps * e)) @ qd / (2 * eps)
        for e in np.eye(2)])
    tau = M @ qdd + M_dot @ qd - 0.5 * grad + kin.gravity_torque(q)
    assert np.allclose(kin.inverse_dynamics(q, qd, qdd), tau, atol=1e-7)


def test_rnea_matches_mass_matrix():
    q = np.radians([20.0, 70.0])
    qd = np.array([0.3, -0.8])
    qdd = np.array([1.5, 2.0])
    bias = kin.inverse_dynamics(q, qd, np.zeros(2))
    assert np.allclose(kin.inverse_dynamics(q, qd, qdd),
                       kin.mass_matrix(q) @ qdd + bias)


def motor_readings(q, tau_human_true):
    """Motor angles/torques of a static exo held against a human torque."""
    theta_m = kin.tendon_jacobian() @ q / kin.SPOOL_RADIUS
    tau_joint = kin.gravity_torque(q) - tau_human_true
    tau_m = kin.loop_forces_for_joint_torque(tau_joint) * kin.SPOOL_RADIUS
    return theta_m, tau_m


def test_no_human_torque_stays_still():
    q = np.radians([30.0, 45.0])
    ctrl = AdmittanceController(AdmittanceParams(), DT, q)
    for _ in range(400):
        ctrl.step(*motor_readings(q, np.zeros(2)))
    assert np.allclose(ctrl.q_ref, q, atol=1e-9)


def test_constant_push_gives_velocity_torque_over_damping():
    params = AdmittanceParams(deadband=np.zeros(2))
    q = np.radians([30.0, 45.0])
    push = np.array([0.0, 0.4])
    ctrl = AdmittanceController(params, DT, q)
    for _ in range(400):  # 2 s, many virtual time constants
        ctrl.step(*motor_readings(q, push))
    assert np.allclose(ctrl.qd_ref, push / params.damping, atol=1e-3)


def test_push_below_deadband_is_ignored():
    q = np.radians([30.0, 45.0])
    ctrl = AdmittanceController(AdmittanceParams(), DT, q)
    for _ in range(400):
        ctrl.step(*motor_readings(q, np.array([0.03, -0.03])))
    assert np.allclose(ctrl.q_ref, q)


def test_reference_stops_at_joint_limit():
    params = AdmittanceParams()
    q = np.radians([30.0, 45.0])
    ctrl = AdmittanceController(params, DT, q)
    for _ in range(2000):
        ctrl.step(*motor_readings(q, np.array([2.0, 0.0])))
    assert ctrl.q_ref[0] == params.q_max[0]
    assert ctrl.qd_ref[0] == 0.0


def test_hold_freezes_reference():
    q = np.radians([30.0, 45.0])
    ctrl = AdmittanceController(AdmittanceParams(), DT, q)
    for _ in range(50):
        ctrl.step(*motor_readings(q, np.array([0.5, 0.5])))
    before = ctrl.q_ref.copy()
    theta_ref = ctrl.hold()
    assert np.all(ctrl.qd_ref == 0.0)
    assert np.allclose(kin.joint_angles_from_motors(theta_ref), before)
