"""Interactive 2D view of the motor -> joint angle mapping.

Drag the motor sliders; the joint angles are computed through the tendon
Jacobian and drawn as a simple planar two-link chain (q0 at the base,
q1 relative to link 1).

    python3 control/visualize.py
"""

import importlib.util
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

# fr-kinamtics.py has a hyphen in its name, so load it by path.
_spec = importlib.util.spec_from_file_location(
    "kinematics", Path(__file__).with_name("fr-kinamtics.py"))
kin = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(kin)

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_MUTED = "#52514e"
GRID = "#e4e3df"
LINK = "#2a78d6"
ZERO = "#b5b4ae"

# Display-only link lengths [m]; the angles do not depend on them.
L1, L2 = 1.0, 0.8
REACH = (L1 + L2) * 1.1


fig = plt.figure(figsize=(10, 6), facecolor=SURFACE)
fig.suptitle("Motor angles → joint angles", color=TEXT, x=0.06, ha="left",
             fontsize=13)
ax = fig.add_axes([0.05, 0.27, 0.55, 0.63])
ax_info = fig.add_axes([0.66, 0.27, 0.32, 0.63])
ax_info.axis("off")

ax.set_facecolor(SURFACE)
ax.set_xlim(-REACH, REACH)
ax.set_ylim(-REACH, REACH)
ax.set_aspect("equal")
ax.grid(color=GRID, linewidth=0.8)
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.tick_params(length=0)
for spine in ax.spines.values():
    spine.set_visible(False)

# Zero pose reference: both links straight along +x.
ax.plot([0, L1 + L2], [0, 0], color=ZERO, linewidth=1.5, linestyle="--", zorder=1)
links, = ax.plot([], [], color=LINK, linewidth=5, solid_capstyle="round", zorder=2)
joints, = ax.plot([], [], "o", color=LINK, markersize=11,
                  markeredgecolor=SURFACE, markeredgewidth=2, zorder=3)
arc0, = ax.plot([], [], color=TEXT_MUTED, linewidth=1, zorder=1)
arc1, = ax.plot([], [], color=TEXT_MUTED, linewidth=1, zorder=1)
ext1, = ax.plot([], [], color=ZERO, linewidth=1, linestyle="--", zorder=1)
lab0 = ax.text(0, 0, "", color=TEXT, fontsize=10)
lab1 = ax.text(0, 0, "", color=TEXT, fontsize=10)
info = ax_info.text(0, 1, "", va="top", family="monospace", fontsize=10.5,
                    color=TEXT)

s_m1 = Slider(fig.add_axes([0.12, 0.14, 0.55, 0.03]), "motor M1",
              -360, 360, valinit=0, valstep=1, color=LINK)
s_m2 = Slider(fig.add_axes([0.12, 0.07, 0.55, 0.03]), "motor M2",
              -360, 360, valinit=0, valstep=1, color=LINK)
for s in (s_m1, s_m2):
    s.label.set_color(TEXT)
    s.valtext.set_color(TEXT)


def arc(center, start, angle, radius):
    """Points of an arc from `start` sweeping `angle` [rad]."""
    t = np.linspace(start, start + angle, 40)
    return center[0] + radius * np.cos(t), center[1] + radius * np.sin(t)


def update(_=None):
    theta_m = np.radians([s_m1.val, s_m2.val])
    q0, q1 = kin.joint_angles_from_motors(theta_m)

    elbow = np.array([L1 * np.cos(q0), L1 * np.sin(q0)])
    tip = elbow + L2 * np.array([np.cos(q0 + q1), np.sin(q0 + q1)])
    links.set_data([0, elbow[0], tip[0]], [0, elbow[1], tip[1]])
    joints.set_data([0, elbow[0]], [0, elbow[1]])

    # q0 measured from +x, q1 measured from the extension of link 1.
    arc0.set_data(*arc((0, 0), 0, q0, 0.3))
    ext = elbow + 0.45 * np.array([np.cos(q0), np.sin(q0)])
    ext1.set_data([elbow[0], ext[0]], [elbow[1], ext[1]])
    arc1.set_data(*arc(elbow, q0, q1, 0.3))
    a0 = q0 / 2
    a1 = q0 + q1 / 2
    lab0.set_position((0.42 * np.cos(a0), 0.42 * np.sin(a0)))
    lab0.set_text(f"q0 = {np.degrees(q0):.1f}°")
    lab1.set_position(elbow + 0.42 * np.array([np.cos(a1), np.sin(a1)]))
    lab1.set_text(f"q1 = {np.degrees(q1):.1f}°")

    r0, r1 = kin.PULLEY_RADII
    info.set_text(
        "Motor angles [deg]\n"
        f"  M1 {s_m1.val:+7.1f}\n"
        f"  M2 {s_m2.val:+7.1f}\n\n"
        "Joint angles [deg]\n"
        f"  q0 {np.degrees(q0):+7.1f}\n"
        f"  q1 {np.degrees(q1):+7.1f}\n\n"
        "Mapping\n"
        "  r_s·θ_m = J_a·q\n"
        f"  J_a = [[{r0}, {r1}],\n"
        f"         [{r0}, {-r1}]]\n"
        f"  r_s = {kin.SPOOL_RADIUS} m\n\n"
        "Same direction → q0\n"
        "Opposite       → q1"
    )
    for s in (s_m1, s_m2):
        s.valtext.set_text(f"{s.val:.0f}°")
    fig.canvas.draw_idle()


s_m1.on_changed(update)
s_m2.on_changed(update)
update()

if __name__ == "__main__":
    plt.show()
