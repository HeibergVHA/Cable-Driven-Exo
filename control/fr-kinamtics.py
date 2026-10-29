"""Forward kinematics, Jacobian and gravity torque for the exo arm.

The arm is a serial chain of revolute joints. Each joint is described by
its rotation axis and its position, both expressed in the frame of the
previous joint at zero angle. Adding the third shoulder rotation or the
elbow later only means adding entries to JOINTS / LINKS.

Base frame (fixed to the torso, origin at the shoulder centre, RIGHT arm):
    x = forward, y = left, z = up
Zero pose: arm hanging straight down along -z.
"""

from dataclasses import dataclass

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])


@dataclass
class Joint:
    name: str
    axis: np.ndarray    # unit rotation axis in the parent frame
    offset: np.ndarray  # joint position in the parent frame [m]


@dataclass
class Link:
    mass: float         # [kg] exo structure moved by this joint (not the human arm)
    com: np.ndarray     # centre of mass in this joint's frame [m]



# here abduction is proximal (closest to the torso) and flexion sits on it.
JOINTS = [
    Joint("shoulder_abduction", np.array([-1.0, 0.0, 0.0]), np.zeros(3)),
    Joint("shoulder_flexion", np.array([0.0, -1.0, 0.0]), np.zeros(3)),
]
LINKS = [
    Link(0.0, np.zeros(3)),
    Link(0.5, np.array([0.0, 0.0, -0.15])),
]
# Cuff / end point (e.g. elbow centre) in the frame of the last joint.
END_EFFECTOR = np.array([0.0, 0.0, -0.30])


def rot(axis, q):
    """Rotation matrix for angle q [rad] about a unit axis (Rodrigues)."""
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + np.sin(q) * k + (1.0 - np.cos(q)) * k @ k


def forward_kinematics(q):
    """Return a list of (R, p) for every joint frame plus the end effector.

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
    """ for a 2 dof arm 
    j_12 =[[r0,r1],  
          [r0,-r1]]

    and a 4-dof arm
    J_a = [[r0,r1,0,0],
          [r0,-r1,0,0],
          [r0,r1,r2,r3],  
          [r0,r1,r2,-r3]]

    Geometric Jacobian (6 x n) of a point fixed in joint `up_to`'s frame.

    Rows 0-2: linear velocity, rows 3-5: angular velocity.
    Joints after `up_to` do not move the point, so their columns are zero.
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
    """Joint torques [Nm] needed to hold the exo structure still at q."""
    tau = np.zeros(len(JOINTS))
    for k, link in enumerate(LINKS):
        Jv = jacobian(q, link.com, up_to=k)[:3]
        tau -= Jv.T @ (link.mass * GRAVITY)
    return tau

if __name__ == "__main__":
    q = np.radians([30.0, 45.0])
    tip = forward_kinematics(q)[-1][1]
    print("q [deg]          :", np.degrees(q))
    print("end effector [m] :", np.round(tip, 4))
    print("gravity tau [Nm] :", np.round(gravity_torque(q), 4))

    # Check the analytic Jacobian against finite differences.
    eps = 1e-6
    J_num = np.zeros((3, len(q)))
    for i in range(len(q)):
        dq = np.zeros(len(q))
        dq[i] = eps
        J_num[:, i] = (forward_kinematics(q + dq)[-1][1] - tip) / eps
    err = np.abs(jacobian(q)[:3] - J_num).max()
    print("Jacobian check   :", "OK" if err < 1e-5 else f"FAIL ({err:.2e})")
