"""Joint-space admittance controller for the cable-driven shoulder.

Each control cycle:
    1. motor angles  -> joint angles             q = r_s J_a^-1 theta_m
    2. motor torques -> joint torques            tau_motor = J_a^T tau_m / r_s
    3. human torque  = model - motor torque      (inverse dynamics + friction)
    4. virtual model M_v qdd + B_v qd + K_v (q - q_eq) = tau_human
    5. reference joint angles -> motor angles    theta_ref = J_a q_ref / r_s

The controller is hardware-agnostic: feed it motor angles and torques in
radians and Nm (zeroed so that theta_m = 0 at q = 0) and send the returned
motor angles to the PDA-04s in position mode.
"""

import importlib.util
from dataclasses import dataclass, field
from pathlib import Path
    
import numpy as np

# fr-kinamtics.py has a hyphen in its name, so load it by path.
_spec = importlib.util.spec_from_file_location(
    "kinematics", Path(__file__).with_name("fr-kinamtics.py"))
kin = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(kin)


def _vec(*values):
    return field(default_factory=lambda: np.array(values, dtype=float))


@dataclass
class AdmittanceParams:
    # Virtual model, per joint. Lower mass/damping feels lighter but is
    # less stable: tune down slowly on the real exo.
    mass: np.ndarray = _vec(0.05, 0.05)          # [kg m^2]
    damping: np.ndarray = _vec(0.5, 0.5)         # [Nm s/rad]
    stiffness: np.ndarray = _vec(0.0, 0.0)       # [Nm/rad], 0 = transparent
    q_eq: np.ndarray = _vec(0.0, 0.0)            # spring rest angle [rad]

    # Safety limits on the reference.
    q_min: np.ndarray = _vec(np.radians(-10), np.radians(-30))   # TODO
    q_max: np.ndarray = _vec(np.radians(120), np.radians(150))   # TODO
    qd_max: np.ndarray = _vec(2.0, 2.0)          # [rad/s]

    # Human torque estimate.
    cutoff_hz: float = 15.0                      # low-pass on the estimate
    deadband: np.ndarray = _vec(0.05, 0.05)      # [Nm], hides model bias
    # Lumped joint-side friction tau_f = b qd + tau_c tanh(qd / 0.05). TODO identify.
    friction_viscous: np.ndarray = _vec(0.0, 0.0)   # [Nm s/rad]
    friction_coulomb: np.ndarray = _vec(0.0, 0.0)   # [Nm]


def friction_torque(qd, params):
    """Joint-side friction model [Nm] (cables, pulleys, gearbox)."""
    return (params.friction_viscous * qd
            + params.friction_coulomb * np.tanh(np.asarray(qd) / 0.05))


def apply_deadband(x, width):
    """Zero inside +-width, continuous outside it."""
    return np.sign(x) * np.maximum(np.abs(x) - width, 0.0)


class AdmittanceController:
    def __init__(self, params, dt, q_start):
        self.p = params
        self.dt = dt
        self.q_ref = np.array(q_start, dtype=float)
        self.qd_ref = np.zeros_like(self.q_ref)
        self.qdd_ref = np.zeros_like(self.q_ref)
        self.tau_human = np.zeros_like(self.q_ref)
        rc = 1.0 / (2.0 * np.pi * params.cutoff_hz)
        self._alpha = dt / (dt + rc)

    def estimate_human_torque(self, q, tau_motor):
        """Human torque [Nm] from the measured joint angle and motor torque.

        Exo model: M qdd + C qd + g + friction = tau_motor + tau_human.
        The reference velocity/acceleration is used instead of
        differentiating the encoders: the position loop tracks it, and it
        is noise free.
        """
        model = (kin.inverse_dynamics(q, self.qd_ref, self.qdd_ref)
                 + friction_torque(self.qd_ref, self.p))
        return model - tau_motor

    def step(self, theta_m, tau_m):
        """Run one control cycle. Returns the motor angle references [rad]."""
        q = kin.joint_angles_from_motors(theta_m)
        tau_motor = kin.joint_torque_from_motors(tau_m)

        raw = self.estimate_human_torque(q, tau_motor)
        self.tau_human += self._alpha * (raw - self.tau_human)
        tau_h = apply_deadband(self.tau_human, self.p.deadband)

        p = self.p
        self.qdd_ref = (tau_h - p.damping * self.qd_ref
                        - p.stiffness * (self.q_ref - p.q_eq)) / p.mass
        # Semi-implicit Euler: update velocity first, then position.
        self.qd_ref = np.clip(self.qd_ref + self.qdd_ref * self.dt, -p.qd_max, p.qd_max)
        self.q_ref = self.q_ref + self.qd_ref * self.dt

        # Joint limits: stop at the wall and drop velocity into it.
        low, high = self.q_ref < p.q_min, self.q_ref > p.q_max
        self.q_ref = np.clip(self.q_ref, p.q_min, p.q_max)
        into_wall = (low & (self.qd_ref < 0)) | (high & (self.qd_ref > 0))
        self.qd_ref[into_wall] = 0.0
        self.qdd_ref[into_wall] = 0.0

        return self.motor_reference()

    def hold(self):
        """Freeze the reference, e.g. when motor feedback goes stale."""
        self.qd_ref[:] = 0.0
        self.qdd_ref[:] = 0.0
        self.tau_human[:] = 0.0
        return self.motor_reference()

    def motor_reference(self):
        return kin.tendon_jacobian() @ self.q_ref / kin.SPOOL_RADIUS
