"""Simulate the admittance controller on a model of the exo.

Plant:   rigid-body exo dynamics (fr-kinamtics.py) + joint friction.
Motors:  PDA-04 position mode modelled as a joint-space PD loop, with one
         control cycle of CAN delay and noise on the torque reading.
Human:   applies a torque profile to the joints.

    python3 control/sim_admittance.py              # opens a plot window
    python3 control/sim_admittance.py --save sim.png
    python3 control/sim_admittance.py --follow     # human moves, exo follows
"""

import argparse

import numpy as np

from admittance import AdmittanceController, AdmittanceParams, friction_torque, kin

CONTROL_HZ = 200          # outer admittance loop (target for the CAN loop)
PLANT_HZ = 2000           # physics integration
DURATION = 6.0            # [s]

# PDA-04 position loop, as seen at the joints. TODO: identify on the bench.
KP = np.array([50.0, 50.0])     # [Nm/rad]
KD = np.array([1.0, 1.0])       # [Nm s/rad]
TORQUE_NOISE = 0.02             # std of motor torque reading [Nm]


# Cuff between human arm and exo, used in --follow mode. TODO: measure.
CUFF_STIFFNESS = 30.0     # [Nm/rad]
CUFF_DAMPING = 0.3        # [Nm s/rad]


def human_arm(t):
    """Human arm motion for --follow: smooth swings in both joints [rad]."""
    amp = np.radians([40.0, 60.0])
    freq = np.array([0.3, 0.2])
    w = 2 * np.pi * freq
    return amp / 2 * (1 - np.cos(w * t)), amp / 2 * w * np.sin(w * t)


def follow_torque(t, q, qd):
    """Torque the moving human arm puts on the exo through the cuff."""
    q_h, qd_h = human_arm(t)
    return CUFF_STIFFNESS * (q_h - q) + CUFF_DAMPING * (qd_h - qd)


def human_torque(t, q=None, qd=None):
    """Test profile: push flexion, release, push abduction, push both."""
    tau = np.zeros(2)
    if 0.5 <= t < 1.5:
        tau[1] = 0.6
    if 2.5 <= t < 3.5:
        tau[0] = 0.6
    if 4.5 <= t < 5.0:
        tau[:] = [-0.5, -0.8]
    return tau


def simulate(params, plant_friction=None, seed=0, human=human_torque,
             duration=DURATION):
    """Run the closed loop. `human(t, q, qd)` gives the human joint torque."""
    rng = np.random.default_rng(seed)
    plant_friction = params if plant_friction is None else plant_friction
    dt_c = 1.0 / CONTROL_HZ
    substeps = PLANT_HZ // CONTROL_HZ
    dt_p = dt_c / substeps

    q = np.zeros(2)
    qd = np.zeros(2)
    ctrl = AdmittanceController(params, dt_c, q)
    q_cmd = ctrl.q_ref.copy()
    qd_cmd = np.zeros(2)
    tau_motor = kin.gravity_torque(q)

    log = {k: [] for k in ("t", "q", "q_ref", "q_human", "tau_h", "tau_h_est",
                           "strand_min")}
    for k in range(int(duration * CONTROL_HZ)):
        t = k * dt_c

        # Sensor readings in motor space, like the PDA-04 reports them.
        theta_m = kin.tendon_jacobian() @ q / kin.SPOOL_RADIUS
        F = kin.loop_forces_for_joint_torque(tau_motor)
        tau_m = F * kin.SPOOL_RADIUS + rng.normal(0, TORQUE_NOISE, 2)

        # Controller output arrives one cycle later (CAN delay).
        theta_ref = ctrl.step(theta_m, tau_m)
        q_next = kin.joint_angles_from_motors(theta_ref)
        qd_next = ctrl.qd_ref.copy()

        for i in range(substeps):
            # The motor interpolates the setpoint with the commanded speed
            # (holding it constant makes the torque reading velocity biased).
            target = q_cmd + qd_cmd * i * dt_p
            tau_motor = KP * (target - q) + KD * (qd_cmd - qd)
            tau_h = human(t, q, qd)
            tau = tau_motor + tau_h - friction_torque(qd, plant_friction)
            qdd = kin.forward_dynamics(q, qd, tau)
            qd = qd + qdd * dt_p
            q = q + qd * dt_p
        q_cmd, qd_cmd = q_next, qd_next

        log["t"].append(t)
        log["q"].append(q.copy())
        log["q_ref"].append(ctrl.q_ref.copy())
        log["q_human"].append(human_arm(t)[0] if human is follow_torque
                              else np.full(2, np.nan))
        log["tau_h"].append(tau_h.copy())
        log["tau_h_est"].append(ctrl.tau_human.copy())
        log["strand_min"].append(kin.strand_tensions(F).min())
    return {k: np.array(v) for k, v in log.items()}


def report(log):
    err = np.degrees(np.abs(log["q_ref"] - log["q"]).max(axis=0))
    est = np.sqrt(np.mean((log["tau_h_est"] - log["tau_h"]) ** 2, axis=0))
    print(f"max tracking error [deg] : q0 {err[0]:.2f}   q1 {err[1]:.2f}")
    print(f"torque estimate RMS [Nm] : q0 {est[0]:.3f}  q1 {est[1]:.3f}")
    print(f"final angles [deg]       : {np.round(np.degrees(log['q'][-1]), 1)}")
    print(f"lowest strand tension [N]: {log['strand_min'].min():.1f}",
          "OK" if log["strand_min"].min() > 0 else "-> SLACK")


def plot(log, save=None):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    surface, text, muted, grid = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
    blue, orange = "#2a78d6", "#eb6834"
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), sharex=True, facecolor=surface)
    fig.suptitle("Admittance control, simulated", color=text, x=0.06, ha="left")
    t = log["t"]
    for j, name in enumerate(["q0 abduction", "q1 flexion"]):
        top, bottom = axes[0, j], axes[1, j]
        if np.isnan(log["q_human"][:, j]).all():
            top.plot(t, np.degrees(log["q_ref"][:, j]), color=blue, lw=2,
                     label="reference")
        else:
            top.plot(t, np.degrees(log["q_human"][:, j]), color=blue, lw=2,
                     label="human arm")
        top.plot(t, np.degrees(log["q"][:, j]), color=orange, lw=2, ls="--",
                 label="exo")
        top.set_ylabel("angle [deg]", color=muted)
        top.set_title(name, color=text, loc="left", fontsize=11)
        bottom.plot(t, log["tau_h"][:, j], color=blue, lw=2, label="human (true)")
        bottom.plot(t, log["tau_h_est"][:, j], color=orange, lw=1.5, label="estimated")
        bottom.set_ylabel("human torque [Nm]", color=muted)
        bottom.set_xlabel("time [s]", color=muted)
        for ax in (top, bottom):
            ax.set_facecolor(surface)
            ax.grid(color=grid, lw=0.8)
            ax.tick_params(colors=muted, labelsize=8)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color(grid)
            ax.legend(frameon=False, fontsize=8, labelcolor=text)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=100)
        print("saved", save)
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--save", help="save the plot instead of showing it")
    parser.add_argument("--follow", action="store_true",
                        help="human arm moves and the exo follows it")
    args = parser.parse_args()
    if args.follow:
        log = simulate(AdmittanceParams(), human=follow_torque, duration=10.0)
        lag = np.degrees(np.abs(log["q_human"] - log["q"]).max(axis=0))
        peak = np.abs(log["tau_h"]).max(axis=0)
        print(f"max exo lag behind arm [deg]: q0 {lag[0]:.1f}   q1 {lag[1]:.1f}")
        print(f"peak force felt by user [Nm]: q0 {peak[0]:.2f}  q1 {peak[1]:.2f}")
    else:
        log = simulate(AdmittanceParams())
    report(log)
    plot(log, args.save)
