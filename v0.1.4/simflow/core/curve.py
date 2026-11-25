# -*- coding: utf-8 -*-
"""Profile curve (root -> tip) used to taper a value along a chain.

Points are ``(x, y)`` with ``x`` in 0..1 (0 = root, 1 = tip) and ``y`` in
0..1 (a multiplier).  The first point is always at x = 0 and the last at
x = 1.  Values between points use monotone cubic interpolation
(Fritsch-Carlson), so the curve is smooth but never overshoots the points.

Pure Python, no Maya import.
"""
from __future__ import annotations

FLAT = ((0.0, 1.0), (1.0, 1.0))


def normalise(points):
    """Sorted, clamped copy with end points at x = 0 and x = 1."""
    pts = sorted((min(1.0, max(0.0, float(x))), min(1.0, max(0.0, float(y)))) for x, y in (points or FLAT))
    if not pts:
        return FLAT
    if pts[0][0] > 0.0:
        pts.insert(0, (0.0, pts[0][1]))
    else:
        pts[0] = (0.0, pts[0][1])
    if pts[-1][0] < 1.0:
        pts.append((1.0, pts[-1][1]))
    else:
        pts[-1] = (1.0, pts[-1][1])
    # drop points with the same x (keep the last one)
    out = []
    for p in pts:
        if out and abs(out[-1][0] - p[0]) < 1.0e-6:
            if len(out) > 1 or p[0] < 1.0:
                out[-1] = p
                continue
        out.append(p)
    if len(out) == 1:
        out.append((1.0, out[0][1]))
    return tuple(out)


def _tangents(xs, ys):
    n = len(xs)
    d = [(ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]) for i in range(n - 1)]
    m = [0.0] * n
    m[0], m[-1] = d[0], d[-1]
    for i in range(1, n - 1):
        if d[i - 1] * d[i] <= 0.0:
            m[i] = 0.0
        else:
            m[i] = (d[i - 1] + d[i]) / 2.0
    for i in range(n - 1):
        if d[i] == 0.0:
            m[i] = m[i + 1] = 0.0
            continue
        a, b = m[i] / d[i], m[i + 1] / d[i]
        s = a * a + b * b
        if s > 9.0:
            t = 3.0 / s ** 0.5
            m[i], m[i + 1] = t * a * d[i], t * b * d[i]
    return m


def evaluate(points, x):
    pts = normalise(points)
    x = min(1.0, max(0.0, float(x)))
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if len(pts) == 2:
        t = (x - xs[0]) / (xs[1] - xs[0])
        return ys[0] + (ys[1] - ys[0]) * t
    m = _tangents(xs, ys)
    for i in range(len(xs) - 1):
        if x <= xs[i + 1] or i == len(xs) - 2:
            h = xs[i + 1] - xs[i]
            t = (x - xs[i]) / h
            t2, t3 = t * t, t * t * t
            v = ((2 * t3 - 3 * t2 + 1) * ys[i] + (t3 - 2 * t2 + t) * h * m[i] +
                 (-2 * t3 + 3 * t2) * ys[i + 1] + (t3 - t2) * h * m[i + 1])
            return min(1.0, max(0.0, v))
    return ys[-1]
