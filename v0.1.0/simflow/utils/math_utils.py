# -*- coding: utf-8 -*-
"""Pure-Python vector / quaternion / matrix math for SIMFLOW.

This module has NO Maya dependency so that the solvers can be unit-tested
outside Maya.

Conventions
-----------
* Vectors are 3-tuples of floats.
* Quaternions are 4-tuples ``(x, y, z, w)`` (Hamilton product, the same
  layout as ``maya.api.OpenMaya.MQuaternion``).  ``rotate(q, v)`` rotates a
  vector actively (``q * v * q^-1``).  ``mul(a, b)`` means "apply ``b`` first,
  then ``a``".
* Matrices are flat 16-element lists in **Maya row-vector convention**:
  ``p' = p * M``; row ``i`` (elements ``4i..4i+2``) is the image of basis axis
  ``i`` and the translation lives in elements 12..14.
"""
from __future__ import annotations

import math

EPS = 1.0e-12

# ---------------------------------------------------------------------------
# Vectors
# ---------------------------------------------------------------------------


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def v_length(a):
    return math.sqrt(v_dot(a, a))


def v_normalize(a, fallback=(0.0, 1.0, 0.0)):
    length = v_length(a)
    if length < EPS:
        return tuple(fallback)
    return (a[0] / length, a[1] / length, a[2] / length)


def v_lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t)


def catmull_rom(p0, p1, p2, p3, t):
    """Uniform Catmull-Rom interpolation between ``p1`` and ``p2``."""
    t2 = t * t
    t3 = t2 * t
    out = []
    for i in range(3):
        out.append(0.5 * ((2.0 * p1[i]) +
                          (-p0[i] + p2[i]) * t +
                          (2.0 * p0[i] - 5.0 * p1[i] + 4.0 * p2[i] - p3[i]) * t2 +
                          (-p0[i] + 3.0 * p1[i] - 3.0 * p2[i] + p3[i]) * t3))
    return tuple(out)


# ---------------------------------------------------------------------------
# Quaternions
# ---------------------------------------------------------------------------

Q_IDENTITY = (0.0, 0.0, 0.0, 1.0)


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def q_normalize(q):
    n = math.sqrt(q_dot(q, q))
    if n < EPS:
        return Q_IDENTITY
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_neg(q):
    return (-q[0], -q[1], -q[2], -q[3])


def q_rotate(q, v):
    """Rotate vector ``v`` by unit quaternion ``q``."""
    qx, qy, qz, qw = q
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx))


def q_from_axis_angle(axis, angle):
    axis = v_normalize(axis, (1.0, 0.0, 0.0))
    s = math.sin(angle * 0.5)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle * 0.5))


def q_from_rotvec(rv):
    """Exponential map: rotation vector (axis * angle, radians) -> quaternion."""
    angle = v_length(rv)
    if angle < 1.0e-9:
        # second-order accurate small-angle form
        half = 0.5
        q = (rv[0] * half, rv[1] * half, rv[2] * half, 1.0)
        return q_normalize(q)
    s = math.sin(angle * 0.5) / angle
    return (rv[0] * s, rv[1] * s, rv[2] * s, math.cos(angle * 0.5))


def q_to_rotvec(q):
    """Logarithmic map along the SHORTEST arc (result angle in [0, pi])."""
    if q[3] < 0.0:
        q = q_neg(q)
    x, y, z, w = q
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < 1.0e-9:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    angle = 2.0 * math.atan2(sin_half, w)
    k = angle / sin_half
    return (x * k, y * k, z * k)


def q_angle(a, b):
    """Shortest angle (radians) between two orientations."""
    d = abs(q_dot(q_normalize(a), q_normalize(b)))
    d = min(1.0, d)
    return 2.0 * math.acos(d)


def q_slerp(a, b, t):
    """Shortest-path spherical interpolation."""
    d = q_dot(a, b)
    if d < 0.0:
        b = q_neg(b)
        d = -d
    if d > 0.9995:
        out = (a[0] + (b[0] - a[0]) * t,
               a[1] + (b[1] - a[1]) * t,
               a[2] + (b[2] - a[2]) * t,
               a[3] + (b[3] - a[3]) * t)
        return q_normalize(out)
    theta = math.acos(d)
    s = math.sin(theta)
    wa = math.sin((1.0 - t) * theta) / s
    wb = math.sin(t * theta) / s
    return (a[0] * wa + b[0] * wb, a[1] * wa + b[1] * wb,
            a[2] * wa + b[2] * wb, a[3] * wa + b[3] * wb)


def q_align(q, reference):
    """Return ``q`` or ``-q`` whichever lies in the hemisphere of ``reference``."""
    return q if q_dot(q, reference) >= 0.0 else q_neg(q)


# ---------------------------------------------------------------------------
# Matrices (Maya row-vector convention, flat 16 lists)
# ---------------------------------------------------------------------------

M_IDENTITY = (1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 1.0)


def m_mul(a, b):
    out = [0.0] * 16
    for r in range(4):
        a0, a1, a2, a3 = a[4 * r], a[4 * r + 1], a[4 * r + 2], a[4 * r + 3]
        for c in range(4):
            out[4 * r + c] = a0 * b[c] + a1 * b[4 + c] + a2 * b[8 + c] + a3 * b[12 + c]
    return out


def m_translation(m):
    return (m[12], m[13], m[14])


def m_from_translation(t):
    m = list(M_IDENTITY)
    m[12], m[13], m[14] = t
    return m


def m_transform_point(m, p):
    return (p[0] * m[0] + p[1] * m[4] + p[2] * m[8] + m[12],
            p[0] * m[1] + p[1] * m[5] + p[2] * m[9] + m[13],
            p[0] * m[2] + p[1] * m[6] + p[2] * m[10] + m[14])


def m_transform_vector(m, v):
    return (v[0] * m[0] + v[1] * m[4] + v[2] * m[8],
            v[0] * m[1] + v[1] * m[5] + v[2] * m[9],
            v[0] * m[2] + v[1] * m[6] + v[2] * m[10])


def m_from_quat(q, t=(0.0, 0.0, 0.0)):
    """Row-vector matrix whose rows are the rotated basis axes."""
    x_axis = q_rotate(q, (1.0, 0.0, 0.0))
    y_axis = q_rotate(q, (0.0, 1.0, 0.0))
    z_axis = q_rotate(q, (0.0, 0.0, 1.0))
    return [x_axis[0], x_axis[1], x_axis[2], 0.0,
            y_axis[0], y_axis[1], y_axis[2], 0.0,
            z_axis[0], z_axis[1], z_axis[2], 0.0,
            t[0], t[1], t[2], 1.0]


def q_from_rows(xr, yr, zr):
    """Quaternion from three ORTHONORMAL row axes (Maya row convention)."""
    # Column-convention rotation matrix R[i][j] = rows[j][i]
    r00, r01, r02 = xr[0], yr[0], zr[0]
    r10, r11, r12 = xr[1], yr[1], zr[1]
    r20, r21, r22 = xr[2], yr[2], zr[2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s, 0.25 * s)
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        q = (0.25 * s, (r01 + r10) / s, (r02 + r20) / s, (r21 - r12) / s)
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        q = ((r01 + r10) / s, 0.25 * s, (r12 + r21) / s, (r02 - r20) / s)
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        q = ((r02 + r20) / s, (r12 + r21) / s, 0.25 * s, (r10 - r01) / s)
    return q_normalize(q)


def m_rotation_quat(m):
    """Extract a proper rotation from a (possibly scaled / sheared / mirrored)
    matrix with Gram-Schmidt orthonormalisation (X axis is kept exact in
    direction).  A mirrored matrix (negative determinant) has its Z axis
    flipped so the result is always a proper rotation; callers must use this
    same function consistently (SIMFLOW only ever uses *delta* rotations)."""
    x = v_normalize((m[0], m[1], m[2]), (1.0, 0.0, 0.0))
    y = (m[4], m[5], m[6])
    y = v_normalize(v_sub(y, v_scale(x, v_dot(x, y))), (0.0, 1.0, 0.0))
    z = v_cross(x, y)
    return q_from_rows(x, y, z)


def m_inverse(m):
    """General 4x4 inverse (row-major)."""
    a = list(m)
    inv = [0.0] * 16
    inv[0] = (a[5] * a[10] * a[15] - a[5] * a[11] * a[14] - a[9] * a[6] * a[15] +
              a[9] * a[7] * a[14] + a[13] * a[6] * a[11] - a[13] * a[7] * a[10])
    inv[4] = (-a[4] * a[10] * a[15] + a[4] * a[11] * a[14] + a[8] * a[6] * a[15] -
              a[8] * a[7] * a[14] - a[12] * a[6] * a[11] + a[12] * a[7] * a[10])
    inv[8] = (a[4] * a[9] * a[15] - a[4] * a[11] * a[13] - a[8] * a[5] * a[15] +
              a[8] * a[7] * a[13] + a[12] * a[5] * a[11] - a[12] * a[7] * a[9])
    inv[12] = (-a[4] * a[9] * a[14] + a[4] * a[10] * a[13] + a[8] * a[5] * a[14] -
               a[8] * a[6] * a[13] - a[12] * a[5] * a[10] + a[12] * a[6] * a[9])
    inv[1] = (-a[1] * a[10] * a[15] + a[1] * a[11] * a[14] + a[9] * a[2] * a[15] -
              a[9] * a[3] * a[14] - a[13] * a[2] * a[11] + a[13] * a[3] * a[10])
    inv[5] = (a[0] * a[10] * a[15] - a[0] * a[11] * a[14] - a[8] * a[2] * a[15] +
              a[8] * a[3] * a[14] + a[12] * a[2] * a[11] - a[12] * a[3] * a[10])
    inv[9] = (-a[0] * a[9] * a[15] + a[0] * a[11] * a[13] + a[8] * a[1] * a[15] -
              a[8] * a[3] * a[13] - a[12] * a[1] * a[11] + a[12] * a[3] * a[9])
    inv[13] = (a[0] * a[9] * a[14] - a[0] * a[10] * a[13] - a[8] * a[1] * a[14] +
               a[8] * a[2] * a[13] + a[12] * a[1] * a[10] - a[12] * a[2] * a[9])
    inv[2] = (a[1] * a[6] * a[15] - a[1] * a[7] * a[14] - a[5] * a[2] * a[15] +
              a[5] * a[3] * a[14] + a[13] * a[2] * a[7] - a[13] * a[3] * a[6])
    inv[6] = (-a[0] * a[6] * a[15] + a[0] * a[7] * a[14] + a[4] * a[2] * a[15] -
              a[4] * a[3] * a[14] - a[12] * a[2] * a[7] + a[12] * a[3] * a[6])
    inv[10] = (a[0] * a[5] * a[15] - a[0] * a[7] * a[13] - a[4] * a[1] * a[15] +
               a[4] * a[3] * a[13] + a[12] * a[1] * a[7] - a[12] * a[3] * a[5])
    inv[14] = (-a[0] * a[5] * a[14] + a[0] * a[6] * a[13] + a[4] * a[1] * a[14] -
               a[4] * a[2] * a[13] - a[12] * a[1] * a[6] + a[12] * a[2] * a[5])
    inv[3] = (-a[1] * a[6] * a[11] + a[1] * a[7] * a[10] + a[5] * a[2] * a[11] -
              a[5] * a[3] * a[10] - a[9] * a[2] * a[7] + a[9] * a[3] * a[6])
    inv[7] = (a[0] * a[6] * a[11] - a[0] * a[7] * a[10] - a[4] * a[2] * a[11] +
              a[4] * a[3] * a[10] + a[8] * a[2] * a[7] - a[8] * a[3] * a[6])
    inv[11] = (-a[0] * a[5] * a[11] + a[0] * a[7] * a[9] + a[4] * a[1] * a[11] -
               a[4] * a[3] * a[9] - a[8] * a[1] * a[7] + a[8] * a[3] * a[5])
    inv[15] = (a[0] * a[5] * a[10] - a[0] * a[6] * a[9] - a[4] * a[1] * a[10] +
               a[4] * a[2] * a[9] + a[8] * a[1] * a[6] - a[8] * a[2] * a[5])
    det = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] + a[3] * inv[12]
    if abs(det) < 1.0e-20:
        raise ValueError("Matrix is singular and cannot be inverted")
    inv_det = 1.0 / det
    return [x * inv_det for x in inv]


def rotate_about_pivot(world, pivot_from, delta_q, pivot_to):
    """``world * T(-pivot_from) * R(delta_q) * T(pivot_to)``.

    Applies a world-space delta rotation about ``pivot_from`` and moves the
    pivot to ``pivot_to`` while preserving any scale / shear in ``world``.
    """
    m = m_mul(world, m_from_translation(v_scale(pivot_from, -1.0)))
    m = m_mul(m, m_from_quat(delta_q))
    return m_mul(m, m_from_translation(pivot_to))
