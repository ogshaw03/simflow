# -*- coding: utf-8 -*-
"""The rope's own curve (縄の当たり判定をカーブで見る).

Pure Python, no Maya dependency.

A rope rig draws the rope with a curve (usually the spline IK curve) whose
CVs are moved by the controllers (clusters, skin, parenting, ...).  The
controllers themselves need not lie on the curve, and the curve does not
pass through them, so colliding the controller points lets the curve cut
through the body.  This module rebuilds the curve from the simulated
controller positions:

    CV_k(p) = CV_k(animation) + sum_i J_ki (p_i - o_i)

``o_i`` is the animated (original) position of point ``i`` of the rope
(grips included), ``J_ki`` a 3x3 matrix: how CV ``k`` moves when point ``i``
moves in the world (measured on the rig in Maya, see core/rope_curve_probe.py).
Any rig whose CVs follow the controllers' translation linearly (clusters,
skin, constraints, direct connections) is described exactly; the rig is not
read, only measured.

A point on the curve is then linear in the controller positions::

    S_s(p) = c_s + sum_i W_si p_i,   W_si = sum_k B_k(u_s) J_ki

so a sample pushed out of a collider is a linear constraint on the
controllers, solved like any other position constraint (``push``).
"""
from __future__ import annotations

import math

from ..utils import math_utils as mu

SAMPLES_PER_SPAN = 4
MAX_SAMPLES = 240


def m3_apply(m, v):
    return (m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2])


def m3_apply_t(m, v):
    """Transposed matrix times v."""
    return (m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
            m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
            m[2] * v[0] + m[5] * v[1] + m[8] * v[2])


def m3_inverse(m):
    a, b, c, d, e, f, g, h, i = m
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1.0e-15:
        return None
    k = 1.0 / det
    return ((e * i - f * h) * k, (c * h - b * i) * k, (b * f - c * e) * k,
            (f * g - d * i) * k, (a * i - c * g) * k, (c * d - a * f) * k,
            (d * h - e * g) * k, (b * g - a * h) * k, (a * e - b * d) * k)


def m3_mul(a, b):
    return tuple(sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3)) for r in range(3) for c in range(3))


def full_knots(maya_knots):
    """Maya's knot vector has one knot less at each end than the textbook one."""
    k = list(maya_knots)
    if len(k) < 2:
        return k
    return [2.0 * k[0] - k[1]] + k + [2.0 * k[-1] - k[-2]]


def basis(degree, knots, n_cv, u):
    """B-spline basis values of every CV at ``u`` (textbook ``knots``)."""
    p = degree
    # the span: knots[j] <= u < knots[j + 1], p <= j <= n_cv - 1
    j = p
    while j < n_cv - 1 and u >= knots[j + 1]:
        j += 1
    N = [0.0] * (p + 1)
    N[0] = 1.0
    left = [0.0] * (p + 1)
    right = [0.0] * (p + 1)
    for r in range(1, p + 1):
        left[r] = u - knots[j + 1 - r]
        right[r] = knots[j + r] - u
        saved = 0.0
        for q in range(r):
            den = right[q + 1] + left[r - q]
            tmp = N[q] / den if abs(den) > 1.0e-15 else 0.0
            N[q] = saved + right[q + 1] * tmp
            saved = left[r - q] * tmp
        N[r] = saved
    out = [0.0] * n_cv
    for q in range(p + 1):
        k = j - p + q
        if 0 <= k < n_cv:
            out[k] = N[q]
    return out


def domain(degree, maya_knots, n_cv):
    k = list(maya_knots)
    return k[degree - 1], k[n_cv - 1]


def evaluate(degree, maya_knots, cvs, u):
    """Point of the curve at parameter ``u`` (Maya's parameter)."""
    b = basis(degree, full_knots(maya_knots), len(cvs), u)
    x = y = z = 0.0
    for w, c in zip(b, cvs):
        if w:
            x += w * c[0]
            y += w * c[1]
            z += w * c[2]
    return (x, y, z)


class RopeCurve(object):
    """What the rope solver needs to know about the rig's curve.

    ``cv_frames``: per frame, the curve's CVs in the world (the animation,
    SIMFLOW layers muted).  ``jac``: per CV, per rope point (grips included,
    rope order) a 3x3 row-major matrix or None.  ``name``: for messages."""

    def __init__(self, degree, knots, cv_frames, jac, name=""):
        self.degree = int(degree)
        self.knots = list(knots)
        self.cv_frames = [list(row) for row in cv_frames]
        self.jac = jac
        self.name = name
        self.n_cv = len(self.cv_frames[0]) if self.cv_frames else 0

    def signature(self):
        parts = [self.name, str(self.degree), ",".join("%.6g" % k for k in self.knots)]
        for row in self.jac:
            for m in row:
                parts.append("-" if m is None else ",".join("%.5g" % x for x in m))
        for row in self.cv_frames:
            for c in row:
                parts.append("%.5g,%.5g,%.5g" % tuple(c))
        return "|".join(parts)

    def samples(self, n=None):
        lo, hi = domain(self.degree, self.knots, self.n_cv)
        spans = max(1, self.n_cv - self.degree)
        n = n or min(MAX_SAMPLES, max(8, SAMPLES_PER_SPAN * spans))
        return [lo + (hi - lo) * s / float(n) for s in range(n + 1)]

    def prepare(self, orig, n_samples=None):
        """Precompute for the solver.  ``orig``: per rope point, its animated
        positions per frame (the positions ``jac`` is relative to)."""
        kn = full_knots(self.knots)
        us = self.samples(n_samples)
        n_pts = len(orig)
        W = []
        for u in us:
            b = basis(self.degree, kn, self.n_cv, u)
            row = []
            for i in range(n_pts):
                acc = None
                for k, w in enumerate(b):
                    m = self.jac[k][i] if w else None
                    if m is None:
                        continue
                    acc = tuple(w * x for x in m) if acc is None else tuple(a + w * x for a, x in zip(acc, m))
                if acc is not None and max(abs(x) for x in acc) > 1.0e-9:
                    row.append((i, acc))
            W.append((b, row))
        n_frames = len(self.cv_frames)
        base = []
        for f in range(n_frames):
            cvs = self.cv_frames[f]
            out = []
            for b, row in W:
                x = y = z = 0.0
                for w, c in zip(b, cvs):
                    if w:
                        x += w * c[0]
                        y += w * c[1]
                        z += w * c[2]
                for i, m in row:
                    q = m3_apply(m, orig[i][f])
                    x -= q[0]
                    y -= q[1]
                    z -= q[2]
                out.append((x, y, z))
            base.append(out)
        return PreparedCurve([row for _, row in W], base, us)


class PreparedCurve(object):
    """Sample ``s`` of the curve = base[s] + sum_i W[s][i] p_i."""

    def __init__(self, W, base_frames, params):
        self.W = W
        self.base_frames = base_frames
        self.params = params
        self.n = len(W)
        self.movable = [bool(row) for row in W]

    def base_at(self, f, t, n_frames):
        """The constant part between frames f and f + 1 (same Catmull-Rom as
        the controllers: it is linear in the animated positions)."""
        bf = self.base_frames
        a, b = bf[max(f - 1, 0)], bf[f]
        c, d = bf[min(f + 1, n_frames - 1)], bf[min(f + 2, n_frames - 1)]
        return [mu.catmull_rom(a[s], b[s], c[s], d[s], t) for s in range(self.n)]

    def point(self, s, p, base):
        x, y, z = base[s]
        for i, m in self.W[s]:
            q = m3_apply(m, p[i])
            x += q[0]
            y += q[1]
            z += q[2]
        return (x, y, z)

    def points(self, p, base):
        return [self.point(s, p, base) for s in range(self.n)]

    def push(self, s, p, normal, depth, inv_m):
        """Move the controllers so that sample ``s`` moves ``depth`` along
        ``normal`` (least change, weighted by ``inv_m`` per point; 0 = fixed).
        Returns the controllers moved with their share, or None."""
        grads = []
        tot = 0.0
        for i, m in self.W[s]:
            w = inv_m[i]
            if w <= 0.0:
                continue
            g = m3_apply_t(m, normal)
            gg = mu.v_dot(g, g)
            if gg <= 1.0e-18:
                continue
            grads.append((i, g, w, gg))
            tot += w * gg
        if tot <= 1.0e-18:
            return None
        lam = depth / tot
        moved = []
        for i, g, w, gg in grads:
            p[i] = mu.v_add(p[i], mu.v_scale(g, w * lam))
            moved.append((i, w * gg / tot))
        return moved
