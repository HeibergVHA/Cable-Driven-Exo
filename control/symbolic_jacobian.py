"""Symbolic Jacobians for the 2-DOF shoulder, checked against fr-kinamtics.py.

    python3 control/symbolic_jacobian.py
"""

import importlib.util
from pathlib import Path

import numpy as np
import sympy as sp

q0, q1 = sp.symbols("q0 q1", real=True)
r0, r1, r_s, L = sp.symbols("r0 r1 r_s L", positive=True)
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

# Geometric Jacobian: same axes as JOINTS in fr-kinamtics.py
# (abduction about -x, then flexion about -y, arm along -z).
p = sp.rot_axis1(q0) * sp.rot_axis2(q1) * sp.Matrix([0, 0, -L])
# sympy's rot_axis* are passive rotations, i.e. rotation by -angle, which
# is exactly a rotation about the negative axis.
p = sp.simplify(p)
J_v = sp.simplify(p.jacobian(q))
show("End-effector position p(q):", p)
show("Geometric Jacobian J_v = dp/dq:", J_v)
J_v_pinv = sp.simplify((J_v.T * J_v).inv() * J_v.T)
show("Pseudo-inverse J_v^+ = (J_v^T J_v)^-1 J_v^T:", J_v_pinv)
show("det(J_v^T J_v) (zero = singular pose):", sp.factor(sp.simplify((J_v.T * J_v).det())))

# Check the symbolic results against the numeric module.
spec = importlib.util.spec_from_file_location(
    "kinematics", Path(__file__).with_name("fr-kinamtics.py"))
kin = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kin)

q_num = np.radians([30.0, 45.0])
values = {q0: q_num[0], q1: q_num[1], L: abs(kin.END_EFFECTOR[2]),
          r0: kin.PULLEY_RADII[0], r1: kin.PULLEY_RADII[1]}
err_v = np.abs(np.array(J_v.subs(values), dtype=float) - kin.jacobian(q_num)[:3]).max()
err_a = np.abs(np.array(J_a.subs(values), dtype=float) - kin.tendon_jacobian()).max()
print("\nCheck against fr-kinamtics.py:",
      "OK" if max(err_v, err_a) < 1e-9 else f"FAIL ({max(err_v, err_a):.2e})")
