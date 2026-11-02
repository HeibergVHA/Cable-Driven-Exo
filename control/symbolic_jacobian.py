"""Symbolic Jacobians for the 2-DOF shoulder, checked against fr-kinamtics.py.

    python3 control/symbolic_jacobian.py
"""

import importlib.util
from pathlib import Path

import numpy as np
import sympy as sp

q0, q1 = sp.symbols("q0 q1", real=True)
r0, r1, r_s = sp.symbols("r0 r1 r_s", positive=True)
d0, d1, d4, d5 = sp.symbols("d0 d1 d4 d5", real=True)
th1, th2 = sp.symbols("theta1 theta2", real=True)
q = sp.Matrix([q0, q1])


def show(title, expr):
    print(f"\n{title}")
    sp.pprint(expr, use_unicode=True)


# Tendon Jacobian: loop length per joint angle.
J_a = sp.Matrix([[r0, r1],
                 [r0, -r1]])
J_a_inv = sp.simplify(J_a.inv())
show("Tendon Jacobian J_a:", J_a)
show("Inverse J_a^-1:", J_a_inv)
show("Joint angles from motor angles, q = r_s J_a^-1 theta:",
     sp.simplify(r_s * J_a_inv * sp.Matrix([th1, th2])))

# Geometric Jacobian: same chain as JOINTS in fr-kinamtics.py (frame 0:
# X up, Z right). q1 = flexion about +Z0, q2 = abduction about -Y0,
# elbow d4 + d5 down the arm, d2 == d0 and d3 == d1 (arm on the centre line).
def rot_z(a):
    return sp.Matrix([[sp.cos(a), -sp.sin(a), 0], [sp.sin(a), sp.cos(a), 0], [0, 0, 1]])


def rot_minus_y(a):
    return sp.Matrix([[sp.cos(a), 0, -sp.sin(a)], [0, 1, 0], [sp.sin(a), 0, sp.cos(a)]])


offset1 = sp.Matrix([0, 0, -d0])
offset2 = sp.Matrix([0, d1, d0])
elbow = sp.Matrix([-(d4 + d5), -d1, 0])
p = sp.simplify(offset1 + rot_z(q0) * (offset2 + rot_minus_y(q1) * elbow))
J_v = sp.simplify(p.jacobian(q))
show("End-effector position p(q):", p)
show("Geometric Jacobian J_v = dp/dq:", J_v)
J_v_pinv = sp.simplify((J_v.T * J_v).inv() * J_v.T)
show("Pseudo-inverse J_v^+ = (J_v^T J_v)^-1 J_v^T:", J_v_pinv)
show("det(J_v^T J_v) (zero = singular pose):", sp.factor(sp.simplify((J_v.T * J_v).det())))

# Check the symbolic results against the numeric module.
spec = importlib.util.spec_from_file_location(
    "kinematics", Path(__file__).with_name("fr-kinamtics.py"))
if spec is None or spec.loader is None:
    raise ImportError("cannot load fr-kinamtics.py next to " + __file__)
kin = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kin)

q_num = np.radians([30.0, 45.0])
values = {q0: q_num[0], q1: q_num[1], d0: kin.D0, d1: kin.D1, d4: kin.D4,
          d5: kin.D5, r0: kin.PULLEY_RADII[0], r1: kin.PULLEY_RADII[1]}
assert np.isclose(kin.D2, kin.D0), "symbolic model assumes d2 == d0"
assert np.isclose(kin.D3, kin.D1), "symbolic model assumes d3 == d1"
p_num = np.array(p.subs(values), dtype=float).ravel()
err_p = np.abs(p_num - kin.forward_kinematics(q_num)[-1][1]).max()
err_v = np.abs(np.array(J_v.subs(values), dtype=float) - kin.jacobian(q_num)[:3]).max()
err_a = np.abs(np.array(J_a.subs(values), dtype=float) - kin.tendon_jacobian()).max()
print("\nCheck against fr-kinamtics.py:",
      "OK" if max(err_p, err_v, err_a) < 1e-9
      else f"FAIL ({max(err_p, err_v, err_a):.2e})")
