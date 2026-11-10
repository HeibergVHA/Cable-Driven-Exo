"""Kinematics, tendon transmission and dynamics for the exo arm."""

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]

GRAVITY = np.array([-9.81, 0.0, 0.0])  # X0 is up


@dataclass
class Joint:
    name: str
    axis: FloatArray    # unit rotation axis in the parent frame
    offset: FloatArray  # joint position in the parent frame [m]


@dataclass
class Link:
    mass: float         # [kg] exo structure moved by this joint (not the human arm)
    com: FloatArray     # centre of mass [m]
    inertia: FloatArray = field(default_factory=lambda: np.zeros((3, 3)))  # inertia tensor about the centre of mass, in this joint's frame [kg m^2]


# axes of q1, q2 and q3 all intersect in the shoulder centre.
D0 = -0.1005 # [m]
D1 = 0.0935
D2 = D0
#D3 = D1
D3 = 0.01
D4 = 0.200
D5 = 0.200
D6 = 0.250  # TODO: measure. Placeholder forearm length, elbow to cuff [m]


JOINTS = [
    Joint("q1_flexion", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -D0])),
    Joint("q2_abduction", np.array([0.0, -1.0, 0.0]), np.array([0.0, D1, D2])),
    #Joint("q3_rotation", np.array([-1.0, 0.0, 0.0]), np.array([-D4, -D3, 0.0])),
    #Joint("q4_elbow", np.array([0.0, -1.0, 0.0]), np.array([-D5, 0.0, 0.0])),
]
LINKS = [
    Link(0.0, np.zeros(3)),
    Link(0.5, np.array([-(D4 + D5) / 2, -D3, 0.0]), np.diag([0.0001, 0.00667, 0.00667])),
    # With q3/q4 active, replace the Link above with these three. TODO: real
    # values from CAD. Placeholders: rods along -X, the first two split the 0.5 kg
    # upper arm, the forearm is 0.3 kg and straight at q4 = 0.
    #Link(0.25, np.array([-D4 / 2, -D3, 0.0]), np.diag([0.00005, 0.00083, 0.00083])),
    #Link(0.25, np.array([-D5 / 2, 0.0, 0.0]), np.diag([0.00005, 0.00083, 0.00083])),
    #Link(0.3, np.array([-D6 / 2, 0.0, 0.0]), np.diag([0.00005, 0.00156, 0.00156])),
]

END_EFFECTOR = np.array([-(D4 + D5), -D3, 0.0])
# With q3/q4 active, use the cuff in the frame of q4 instead:
#END_EFFECTOR = np.array([-D6, 0.0, 0.0])

PULLEY_RADII = [0.05, 0.05]  # pulley radii for q1, q2 [m] [0.05, 0.05, 0.05, 0.05]
SPOOL_RADIUS = 0.025         # cable spool radius on the PDA-04 output [m]
PRETENSION = 50.0            # TODO: measure loop pretension [N], as for net just a guess
MOTOR_INERTIA = 0.0          # TODO: PDA-04 rotor inertia seen at the output [kg m^2], is not zero but who knows


def rot(axis, q):
    """Rotation matrix for angle q [rad] about its axis."""
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + np.sin(q) * k + (1.0 - np.cos(q)) * k @ k


def forward_kinematics(q):
    """
    Return a list of (R, p) for every joint frame plus the end effector.

    frames[i] is the world pose of joint i after its rotation,
    frames[-1] is the end effector (R of the last joint, p of the tip).
    """
    R = np.eye(3)
    p = np.zeros(3)
    frames = []
    for joint, qi in zip(JOINTS, q):
        p = p + R @ joint.offset
        R = R @ rot(joint.axis, qi)
        frames.append((R, p))
    frames.append((R, p + R @ END_EFFECTOR))
    return frames


def jacobian(q, point_in_last_frame=END_EFFECTOR, up_to=None):
    """
    j_12 =[[r0,r1],
          [r0,-r1]]

    J_12^-1 = [[1/2r0,1/2r1],
             [1/2r0,-1/2r1],

    and a 4-dof arm
    J_a = [[r0,r1,0,0],
          [r0,-r1,0,0],
          [r0,r1,r2,r3],
          [r0,r1,r2,-r3]]

    J_a^-1 = [[1/2r0,1/2r1,0,0],
             [1/2r0,-1/2r1,0,0],
             [1/2r0,1/2r1,1/2r2,1/2r3],
             [1/2r0,1/2r1,1/2r2,-1/2r3]]
    """
    n = len(JOINTS)
    up_to = n - 1 if up_to is None else up_to
    frames = forward_kinematics(q)
    R_k, p_k = frames[up_to]
    point = p_k + R_k @ point_in_last_frame

    J = np.zeros((6, n))
    for i in range(up_to + 1):
        R_i, p_i = frames[i]
        z = R_i @ JOINTS[i].axis  # joint axis is unchanged by its own rotation
        J[:3, i] = np.cross(z, point - p_i)
        J[3:, i] = z
    return J


def gravity_torque(q):
    """Joint torques [Nm] needed to hold the exo structure still at q. (at the pos)"""
    tau = np.zeros(len(JOINTS))
    for k, link in enumerate(LINKS):
        Jv = jacobian(q, link.com, up_to=k)[:3]
        tau -= Jv.T @ (link.mass * GRAVITY)
    return tau


def tendon_jacobian():
    """Tendon Jacobian J_a (loops x joints), constant for round pulleys."""
    r0, r1 = PULLEY_RADII
    return np.array([[r0, r1],
                     [r0, -r1]])
    #r0, r1, r2, r3 = PULLEY_RADII
    #return np.array([[r0, r1, 0.0, 0.0],
    #                 [r0, -r1, 0.0, 0.0],
    #                 [r0, r1, r2, r3],
    #                 [r0, r1, r2, -r3]])


def joint_angles_from_motors(theta_m):
    return np.linalg.solve(tendon_jacobian(), SPOOL_RADIUS * np.asarray(theta_m)) # Joint angles [rad] from motor angles [rad]


def joint_torque_from_motors(tau_m):
    loop_force = np.asarray(tau_m) / SPOOL_RADIUS
    return tendon_jacobian().T @ loop_force # The measured motor torques [Nm].


def loop_forces_for_joint_torque(tau):
    return np.linalg.solve(tendon_jacobian().T, np.asarray(tau))# Signed loop forces [N]


def strand_tensions(loop_force):
    """Tensions [N] of the two strands of each loop, shape (loops, 2)."""
    half = np.asarray(loop_force)[:, None] / 2.0
    return PRETENSION + np.hstack([half, -half])


def rnea(q, qd, qdd, gravity=True): # recursive Newton-Euler (rnea) dont know what to name it???
    n = len(JOINTS)
    omega = np.zeros(3)
    alpha = np.zeros(3)
    acc = -GRAVITY if gravity else np.zeros(3)  # acceleration of the origin
    R = np.eye(3)
    p = np.zeros(3)

    axes, origins, coms, forces, moments = [], [], [], [], []
    for joint, link, qi, qdi, qddi in zip(JOINTS, LINKS, q, qd, qdd):
        # The joint origin is fixed on body.
        d = R @ joint.offset
        acc = acc + np.cross(alpha, d) + np.cross(omega, np.cross(omega, d))
        p = p + d
        R = R @ rot(joint.axis, qi)
        z = R @ joint.axis

        alpha = alpha + z * qddi + np.cross(omega, z * qdi)
        omega = omega + z * qdi

        c = R @ link.com
        acc_c = acc + np.cross(alpha, c) + np.cross(omega, np.cross(omega, c))
        I_w = R @ link.inertia @ R.T
        axes.append(z)
        origins.append(p)
        coms.append(p + c)
        forces.append(link.mass * acc_c)
        moments.append(I_w @ alpha + np.cross(omega, I_w @ omega))

    tau = np.zeros(n)
    f = np.zeros(3)  # force from the child link
    m = np.zeros(3)  # moment from the child link, about the child's origin
    p_child = np.zeros(3)
    for i in reversed(range(n)):
        m = (moments[i] + np.cross(coms[i] - origins[i], forces[i])
             + m + np.cross(p_child - origins[i], f))
        f = forces[i] + f
        p_child = origins[i]
        tau[i] = axes[i] @ m
    return tau


def reflected_motor_inertia():
    """J_a^T (I_m / r_s^2) J_a."""
    J_a = tendon_jacobian()
    return MOTOR_INERTIA / SPOOL_RADIUS**2 * J_a.T @ J_a


def mass_matrix(q):
    """Joint-space inertia M(q)"""
    n = len(JOINTS)
    zero = np.zeros(n)
    M = np.column_stack([rnea(q, zero, np.eye(n)[j], gravity=False)
                         for j in range(n)])
    return M + reflected_motor_inertia()


def inverse_dynamics(q, qd, qdd):
    """Joint torques [Nm] follow (q, qd, qdd)."""
    return rnea(q, qd, qdd) + reflected_motor_inertia() @ np.asarray(qdd)


def forward_dynamics(q, qd, tau):
    """Joint accelerations [rad/s^2] """
    bias = inverse_dynamics(q, qd, np.zeros(len(JOINTS)))
    return np.linalg.solve(mass_matrix(q), np.asarray(tau) - bias)


if __name__ == "__main__":
    q = np.radians([30.0, 45.0])
    tip = forward_kinematics(q)[-1][1]
    print("q [deg]          :", np.degrees(q))
    print("end effector [m] :", np.round(tip, 4))
    print("gravity tau [Nm] :", np.round(gravity_torque(q), 4))

    eps = 1e-6
    J_num = np.zeros((3, len(q)))
    for i in range(len(q)):
        dq = np.zeros(len(q))
        dq[i] = eps
        J_num[:, i] = (forward_kinematics(q + dq)[-1][1] - tip) / eps
    err = np.abs(jacobian(q)[:3] - J_num).max()
    print("Jacobian check   :", "OK" if err < 1e-5 else f"FAIL ({err:.2e})")

    # Inverse dynamics: static RNEA must equal the Jacobian-based gravity.
    zero = np.zeros(len(q))
    err_g = np.abs(rnea(q, zero, zero) - gravity_torque(q)).max()
    print("RNEA gravity check:", "OK" if err_g < 1e-9 else f"FAIL ({err_g:.2e})")
    M = mass_matrix(q)
    sym_pd = np.allclose(M, M.T) and (np.linalg.eigvalsh(M) > 0).all()
    print("M(q) symmetric, positive definite:", "OK" if sym_pd else "FAIL")

    np.set_printoptions(precision=4, suppress=True)
    print("\nMass matrix M(q) [kg m^2]:\n", M)
    J_a = tendon_jacobian()
    print("\nTendon Jacobian J_a (loop length [m] per joint rad):\n", J_a)
    print("Inverse J_a^-1:\n", np.linalg.inv(J_a))
    print("Motor -> joint, q = r_s * J_a^-1 * theta_m:\n",
          SPOOL_RADIUS * np.linalg.inv(J_a))
    J_v = jacobian(q)[:3]
    print("\nGeometric Jacobian J_v at q (end-effector velocity [m/s] per joint rad/s):\n", J_v)
    print("Pseudo-inverse J_v^+ (3x2 is not square, so no true inverse):\n",
          np.linalg.pinv(J_v))
    print()

    # Motor <-> joint round trip through the tendon Jacobian.
    theta_m = J_a @ q / SPOOL_RADIUS
    print("motor angles [deg]:", np.round(np.degrees(theta_m), 2))
    print("back to q [deg]   :", np.round(np.degrees(joint_angles_from_motors(theta_m)), 2))
    F = loop_forces_for_joint_torque(gravity_torque(q))
    T = strand_tensions(F)
    print("loop forces for gravity [N]:", np.round(F, 2))
    print("strand tensions [N]        :", np.round(T, 2).tolist(),
          "OK" if (T > 0).all() else "-> a strand goes slack, raise PRETENSION")
    print("motor torques back to tau  :",
          np.round(joint_torque_from_motors(F * SPOOL_RADIUS), 4))
