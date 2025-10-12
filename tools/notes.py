import sympy as sp

# Define symbols
q0, q1, q2, q3, x, y, z = sp.symbols("q0 q1 q2 q3 x y z", real=True)

# Quaternion and vector-as-quaternion
q = sp.Matrix([q0, q1, q2, q3])
v = sp.Matrix([0, x, y, z])


# Quaternion multiplication function
def qmul(a, b):
    a0, a1, a2, a3 = a
    b0, b1, b2, b3 = b
    return sp.Matrix(
        [
            a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3,
            a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2,
            a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1,
            a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0,
        ]
    )


# Conjugate of q
q_conj = sp.Matrix([q0, -q1, -q2, -q3])

# Compute q * v * q_conj
v_rot = qmul(qmul(q, v), q_conj)

# Simplify under assumption that q is normalized (q0^2 + q1^2 + q2^2 + q3^2 = 1)
v_rot_simpl = sp.simplify(v_rot)

print("Rotated vector quaternion:")
sp.pprint(v_rot_simpl)

# Extract only the vector part (ignore scalar component)
v_rot_vec = v_rot_simpl[1:]

# Express as matrix multiplication R(q) * [x, y, z]
R = sp.simplify(
    sp.Matrix(
        [
            [sp.diff(v_rot_vec[0], x), sp.diff(v_rot_vec[0], y), sp.diff(v_rot_vec[0], z)],
            [sp.diff(v_rot_vec[1], x), sp.diff(v_rot_vec[1], y), sp.diff(v_rot_vec[1], z)],
            [sp.diff(v_rot_vec[2], x), sp.diff(v_rot_vec[2], y), sp.diff(v_rot_vec[2], z)],
        ]
    )
)

print("\nRotation matrix R(q):")
sp.pprint(R)


# rotation of 90° about z
theta = sp.pi / 2
q = sp.Matrix([sp.cos(theta / 2), 0, 0, sp.sin(theta / 2)])
q_conj = sp.Matrix([sp.cos(theta / 2), 0, 0, -sp.sin(theta / 2)])

# vector along x
v = sp.Matrix([0, 1, 0, 0])

# Extrinsic (active) rotation: q * v * q*
v_extrinsic = qmul(qmul(q, v), q_conj)

# Intrinsic (passive) rotation: q* * v * q
v_intrinsic = qmul(qmul(q_conj, v), q)

sp.pprint(v_extrinsic)
sp.pprint(v_intrinsic)
