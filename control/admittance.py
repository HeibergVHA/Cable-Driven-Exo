import importlib.util
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import numpy.typing as npt

# fr-kinamtics.py has a hyphen in its name, so load it by path.
_spec = importlib.util.spec_from_file_location(
    "kinematics", Path(__file__).with_name("fr-kinamtics.py"))
if _spec is None or _spec.loader is None:
    raise ImportError("could not load fr-kinamtics.py")
kin = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(kin)

def _vec(*values: float) -> Callable[[], npt.NDArray[np.float64]]:
    return lambda: np.array(values, dtype=float)


@dataclass
class AdmittanceParams:
    mass: npt.NDArray[np.float64] = field(default_factory=_vec(0.05, 0.05))          # [kg m^2]
    damping: npt.NDArray[np.float64] = field(default_factory=_vec(0.5, 0.5))         # [Nm s/rad]
    stiffness: npt.NDArray[np.float64] = field(default_factory=_vec(0.0, 0.0))       # [Nm/rad], 0 = transparent
    q_eq: npt.NDArray[np.float64] = field(default_factory=_vec(0.0, 0.0))            # spring rest angle [rad]

    q_min: npt.NDArray[np.float64] = field(default_factory=_vec(np.radians(-30), np.radians(-10))) # MIN
    q_max: npt.NDArray[np.float64] = field(default_factory=_vec(np.radians(150), np.radians(120))) # MAX
    qd_max: npt.NDArray[np.float64] = field(default_factory=_vec(2.0, 2.0))          # [rad/s]

    cutoff_hz: float = 15.0                      # low-pass on the estimate
    deadband: npt.NDArray[np.float64] = field(default_factory=_vec(0.15, 0.15))      # [Nm], hides model bias; too small shakes at damping 0.5

    friction_viscous: npt.NDArray[np.float64] = field(default_factory=_vec(0.0, 0.0))   # [Nm s/rad]
    friction_coulomb: npt.NDArray[np.float64] = field(default_factory=_vec(0.0, 0.0))   # [Nm]


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
        self._rc = 1.0 / (2.0 * np.pi * params.cutoff_hz)

    def joint_angles(self, theta_m):
        return kin.joint_angles_from_motors(theta_m)

    def joint_torques(self, tau_m):
        return kin.joint_torque_from_motors(tau_m)

    def motor_speeds(self):
        return kin.tendon_jacobian() @ self.qd_ref / kin.SPOOL_RADIUS

    def estimate_human_torque(self, q, tau_motor):
        """Exo model: M qdd + C qd + g + friction = tau_motor + tau_human."""
        model = (kin.inverse_dynamics(q, self.qd_ref, self.qdd_ref)
                 + friction_torque(self.qd_ref, self.p))
        return model - tau_motor

    def step(self, theta_m, tau_m, dt=None):
        """Run one control cycle. Returns the motor angle references [rad].

        Pass the measured cycle time as dt when the loop rate is not constant.
        """
        dt = self.dt if dt is None else dt
        q = self.joint_angles(theta_m)
        tau_motor = self.joint_torques(tau_m)

        raw = self.estimate_human_torque(q, tau_motor)
        self.tau_human += dt / (dt + self._rc) * (raw - self.tau_human)
        tau_h = apply_deadband(self.tau_human, self.p.deadband)

        p = self.p
        self.qdd_ref = (tau_h - p.damping * self.qd_ref
                        - p.stiffness * (self.q_ref - p.q_eq)) / p.mass
        # Semi-implicit Euler: update velocity first, then position.
        self.qd_ref = np.clip(self.qd_ref + self.qdd_ref * dt, -p.qd_max, p.qd_max)
        self.q_ref = self.q_ref + self.qd_ref * dt

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


class SingleMotorController(AdmittanceController):
    """One motor drives q1 through its loop, with q2 locked at 0."""

    def __init__(self, params, dt, gravity=False):
        super().__init__(params, dt, np.zeros(2))
        self.gravity = gravity
        self.ratio = kin.PULLEY_RADII[0] / kin.SPOOL_RADIUS     # motor rad per joint rad

    def joint_angles(self, theta_m):
        return np.array([theta_m[0] / self.ratio, 0.0])

    def joint_torques(self, tau_m):
        return np.array([tau_m[0] * self.ratio, 0.0])

    def motor_speeds(self):
        return self.qd_ref[:1] * self.ratio

    def motor_reference(self):
        return self.q_ref[:1] * self.ratio

    def estimate_human_torque(self, q, tau_motor):
        if self.gravity:
            raw = super().estimate_human_torque(q, tau_motor)
        else:
            raw = friction_torque(self.qd_ref, self.p) - tau_motor
        raw[1] = 0.0    # q2 is locked, so nothing can push it
        return raw
