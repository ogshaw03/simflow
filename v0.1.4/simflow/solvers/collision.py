# -*- coding: utf-8 -*-
"""Sphere / capsule colliders for the spring solvers (pure Python).

A collider at one instant is ``(center, axis, half_length, radius)`` in world
space: a sphere is a capsule with ``half_length = 0``.  Per frame the solver
gets a list of colliders; between frames centers and axes are interpolated.

Collision is resolved on the *tip* of every segment (the point mass): when
it is inside ``radius + thickness`` it is moved to the surface by rotating the
segment about its own pivot, and the angular velocity that moves the tip into
the collider is removed.  Pivots are never pushed, so a hair root that sits
on (or inside) a head sphere does not make the hair jump off.

A collider with a 5th element ``True`` is a *bone* of another swing group
("押し出す揺れもの"): thin, so the whole segment is tested against it (the
closest points of the two segments), not only the tip.

A collider with a 5th element ``"tri"`` and 6th element (a, b, c) is a
*panel*: a triangle of the cloth between two neighbouring chains of that
group (without it a strand slips through the gap between the bones).  A
segment that went through a panel is pushed back to the side it was on
before (``sides``: remembered per segment and panel by the solver, while the
segment is in front of the panel's face), or - before it has been seen in
front of the face - to the side of its own pivot.  The remembered side keeps
a fast strand (a character moved a lot in ONE frame) from being pushed out
on the wrong side when its pivot is briefly on the other side.

A collider with a 5th element ``"floor"`` is an infinite plane through
``center`` with the outward (up) normal ``axis``: a point must stay
``radius + thickness`` above it.

A collider with a 5th element ``"ell"`` is a sphere / capsule under a
NON-UNIFORM scale (made with :func:`ellipsoid`): an ellipsoid, or a capsule
with an elliptic body and ellipsoid ends, exactly as it is drawn.  It is
tested in its own unscaled space (local radius / half length, the linear map
``A`` and its inverse); the first 4 elements are a capsule that CONTAINS it
(for the broad phases).  Without it the shape was a sphere of the mean
radius: the cloth stood off the thin side and sank into the long side.
"""
from __future__ import annotations

import math

from ..utils import math_utils as mu

# アニメの側へ逃がす: the side's direction counts across the collider's axis when
# at least this part of it (sine of its angle to the axis) is across
SIDE_ACROSS = 0.2
# アニメの側へ逃がす: the way out on that side is taken only when the nearest way
# is at most this far (deg) round from it
SIDE_ALT_MAX_DEG = 150.0


def lerp_colliders(a, b, t, before=None, after=None):
    """Colliders between two frames (same order).  With the frames ``before``
    ``a`` and ``after`` ``b``, positions use the same Catmull-Rom curve as the
    solvers' pivots: a collider and a pivot that move together (a sleeve and
    the head on one body) stay together between frames, also when the body
    is moved a lot in one frame (a straight line here would leave the pivot,
    which overshoots on the curve, on the other side of the collider)."""
    if not a:
        return a
    curve = before is not None and after is not None and len(before) == len(a) and len(after) == len(b)

    def pos(k, pa, pb, pick):
        if curve:
            return mu.catmull_rom(pick(before[k]), pa, pb, pick(after[k]), t)
        return mu.v_lerp(pa, pb, t)
    out = []
    for k, (ca, cb) in enumerate(zip(a, b)):
        center = pos(k, ca[0], cb[0], lambda c: c[0])
        if (is_ell(ca) or is_ell(cb)) and not (is_floor(ca) or is_floor(cb)):
            ea, eb = as_ell(ca), as_ell(cb)          # the scale became (non-)uniform between the frames
            A = tuple(mu.v_lerp(x, y, t) for x, y in zip(ea[5], eb[5]))
            try:
                out.append(ellipsoid(center, A, ea[7] + (eb[7] - ea[7]) * t, ea[8] + (eb[8] - ea[8]) * t))
            except ValueError:                       # the two axis frames do not blend: the nearer frame's shape
                e = ea if t < 0.5 else eb
                out.append(ellipsoid(center, e[5], e[7], e[8]))
            continue
        axis = mu.v_normalize(mu.v_lerp(ca[1], cb[1], t), ca[1])
        if len(ca) > 5 and ca[4] == "tri":
            pts = tuple(pos(k, pa, pb, lambda c, j=j: c[5][j]) for j, (pa, pb) in enumerate(zip(ca[5], cb[5])))
            out.append((center, axis, ca[2], ca[3], "tri", pts))
            continue
        out.append((center, axis, ca[2] + (cb[2] - ca[2]) * t, ca[3] + (cb[3] - ca[3]) * t) + tuple(ca[4:]))
    return out


def _bone_points(col):
    if len(col) > 5 and col[4] == "tri":
        return list(col[5])
    center, axis, half = col[0], col[1], col[2]
    return [mu.v_sub(center, mu.v_scale(axis, half)), mu.v_add(center, mu.v_scale(axis, half))]


def bone_motion(col_frames, thickness):
    """For colliders made from other swing groups (bones / panels): per frame
    interval the largest distance a point of them moves, and the smallest
    reach (radius + ``thickness``).  (None, None) when there are none.

    A solver uses it to take enough sub steps that nothing moves more than
    half the reach in one sub step: a thin bone or a panel moving faster
    would jump over a segment between two sub steps (a character moved a lot
    in ONE frame)."""
    if not col_frames:
        return None, None
    idx = [k for k, c in enumerate(col_frames[0]) if is_bone(c)]
    if not idx:
        return None, None
    reach = min(col_frames[0][k][3] for k in idx) + max(0.0, thickness)
    disp = []
    for f in range(len(col_frames) - 1):
        a, b = col_frames[f], col_frames[f + 1]
        worst = 0.0
        for k in idx:
            if k >= len(a) or k >= len(b):
                continue
            for pa, pb in zip(_bone_points(a[k]), _bone_points(b[k])):
                worst = max(worst, mu.v_length(mu.v_sub(pb, pa)))
        disp.append(worst)
    return disp, max(reach, 1.0e-3)


def panel(a, b, c, radius):
    """Triangle collider (cloth between two chains)."""
    center = mu.v_scale(mu.v_add(mu.v_add(a, b), c), 1.0 / 3.0)
    return (center, (0.0, 1.0, 0.0), 0.0, radius, "tri", (a, b, c))


def closest_point_triangle(p, a, b, c):
    """Closest point to p on triangle abc (Ericson, Real-Time Collision Detection 5.1.5)."""
    ab, ac, ap = mu.v_sub(b, a), mu.v_sub(c, a), mu.v_sub(p, a)
    d1, d2 = mu.v_dot(ab, ap), mu.v_dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a
    bp = mu.v_sub(p, b)
    d3, d4 = mu.v_dot(ab, bp), mu.v_dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        return mu.v_add(a, mu.v_scale(ab, d1 / (d1 - d3)))
    cp = mu.v_sub(p, c)
    d5, d6 = mu.v_dot(ab, cp), mu.v_dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        return mu.v_add(a, mu.v_scale(ac, d2 / (d2 - d6)))
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        return mu.v_add(b, mu.v_scale(mu.v_sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))))
    den = 1.0 / (va + vb + vc)
    v, w = vb * den, vc * den
    return mu.v_add(a, mu.v_add(mu.v_scale(ab, v), mu.v_scale(ac, w)))


def segment_triangle(p, q, a, b, c):
    """Closest points of segment pq and triangle abc: (s, on segment, on triangle)."""
    n = mu.v_cross(mu.v_sub(b, a), mu.v_sub(c, a))
    if mu.v_length(n) > 1.0e-12:
        dp = mu.v_dot(mu.v_sub(p, a), n)
        dq = mu.v_dot(mu.v_sub(q, a), n)
        if dp * dq < 0.0:
            t = dp / (dp - dq)
            x = mu.v_add(p, mu.v_scale(mu.v_sub(q, p), t))
            k = closest_point_triangle(x, a, b, c)
            if mu.v_length(mu.v_sub(x, k)) < 1.0e-9:
                return t, x, k                           # the segment pierces the triangle
    best = None
    for t, pt in ((0.0, p), (1.0, q)):
        k = closest_point_triangle(pt, a, b, c)
        d = mu.v_length(mu.v_sub(pt, k))
        if best is None or d < best[0]:
            best = (d, t, pt, k)
    for u, v in ((a, b), (b, c), (c, a)):
        t, c1, c2 = closest_on_segments(p, q, u, v)
        d = mu.v_length(mu.v_sub(c1, c2))
        if d < best[0]:
            best = (d, t, c1, c2)
    return best[1:]


def is_bone(col):
    return len(col) > 4 and (col[4] is True or col[4] == "tri")


def is_floor(col):
    return len(col) > 4 and col[4] == "floor"


def is_ell(col):
    return len(col) > 4 and col[4] == "ell"


def _inverse3(A):
    """Rows of the inverse of the 3x3 matrix whose COLUMNS are A[0], A[1], A[2]."""
    a, b, c = A
    r0 = mu.v_cross(b, c)
    r1 = mu.v_cross(c, a)
    r2 = mu.v_cross(a, b)
    det = mu.v_dot(a, r0)
    if abs(det) < 1.0e-18:
        raise ValueError("degenerate collider scale")
    inv = 1.0 / det
    return (mu.v_scale(r0, inv), mu.v_scale(r1, inv), mu.v_scale(r2, inv))


def ellipsoid(center, A, local_radius, local_half=0.0):
    """A sphere (``local_half`` 0) / capsule (local Y axis) of the given local
    size under the linear map with columns ``A`` (the world vectors of the
    local X, Y, Z axes, scale included): see the module doc ("ell")."""
    A = tuple(tuple(v) for v in A)
    inv = _inverse3(A)
    scales = [mu.v_length(v) for v in A]
    axis = mu.v_normalize(A[1], (0.0, 1.0, 0.0))
    return (tuple(center), axis, local_half * scales[1], local_radius * max(scales), "ell", A, inv,
            local_radius, local_half)


def as_ell(col):
    """A plain sphere / capsule as the same shape in the "ell" form."""
    if is_ell(col):
        return col
    axis = mu.v_normalize(col[1], (0.0, 1.0, 0.0))
    x = mu.v_cross(axis, (0.0, 0.0, 1.0))
    if mu.v_length(x) < 1.0e-6:
        x = mu.v_cross(axis, (1.0, 0.0, 0.0))
    x = mu.v_normalize(x)
    z = mu.v_cross(x, axis)                      # right handed: X x Y = Z
    return ellipsoid(col[0], (x, axis, z), col[3], col[2])


def ell_local(col, p):
    """``p`` in the collider's unscaled space."""
    d = mu.v_sub(p, col[0])
    inv = col[6]
    return (mu.v_dot(inv[0], d), mu.v_dot(inv[1], d), mu.v_dot(inv[2], d))


def ell_world_point(col, u):
    A = col[5]
    return mu.v_add(col[0], mu.v_add(mu.v_add(mu.v_scale(A[0], u[0]), mu.v_scale(A[1], u[1])), mu.v_scale(A[2], u[2])))


def ell_world_normal(col, n_local):
    """A local normal as a world unit normal (inverse transpose)."""
    inv = col[6]
    n = mu.v_add(mu.v_add(mu.v_scale(inv[0], n_local[0]), mu.v_scale(inv[1], n_local[1])),
                 mu.v_scale(inv[2], n_local[2]))
    return mu.v_normalize(n, (0.0, 1.0, 0.0))


def _closest_ellipse(y, e):
    """Closest point of the ellipsoid (2 or 3 semi-axes ``e``, principal frame)
    to ``y`` (inside or outside): the root t of sum((e_i y_i / (t + e_i^2))^2) = 1,
    x_i = e_i^2 y_i / (t + e_i^2) (Eberly).  Bisection: robust for any shape."""
    n = len(e)
    e2 = [v * v for v in e]
    inside = sum((y[i] / e[i]) ** 2 for i in range(n)) < 1.0

    def F(t):
        return sum((e[i] * y[i] / (t + e2[i])) ** 2 for i in range(n)) - 1.0
    if inside:
        # the root is in (-min e^2, 0]
        k = min(range(n), key=lambda i: e2[i])
        tiny = 1.0e-12 * max(e)
        group = [i for i in range(n) if e2[i] - e2[k] <= 1.0e-12 * e2[k]]     # the shortest axes
        if all(abs(y[i]) < tiny for i in group):
            # in the plane of the shortest axes (e.g. ON the long axis): t = -e_k^2 when the
            # point lies inside the "evolute" there (Eberly's degenerate case), else the root
            x = [0.0] * n
            acc = 0.0
            for i in range(n):
                if i in group:
                    continue
                x[i] = e2[i] * y[i] / (e2[i] - e2[k])
                acc += (x[i] / e[i]) ** 2
            if acc <= 1.0:
                x[k] = e[k] * math.sqrt(1.0 - acc)
                return x
        else:
            k = next(i for i in group if abs(y[i]) >= tiny)
        lo = -e2[k] * (1.0 - 1.0e-15)
        hi = 0.0
    else:
        lo = 0.0
        hi = max(e) * math.sqrt(sum(v * v for v in y)) + 1.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if F(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    t = 0.5 * (lo + hi)
    return [e2[i] * y[i] / (t + e2[i]) for i in range(n)]


def _ell_frame(col):
    """(unit axes, scales) when the collider's axes are perpendicular (a plain
    rotate + scale: no shear), else None."""
    A = col[5]
    s = [mu.v_length(v) for v in A]
    if min(s) < 1.0e-12:
        return None
    u = [mu.v_scale(A[i], 1.0 / s[i]) for i in range(3)]
    if max(abs(mu.v_dot(u[0], u[1])), abs(mu.v_dot(u[0], u[2])), abs(mu.v_dot(u[1], u[2]))) > 1.0e-6:
        return None
    return u, s


def ell_surface(col, p):
    """(signed distance, surface point, outward normal) of ``p`` for an "ell"
    collider: the CLOSEST point of the drawn surface (an ellipsoid, or an
    elliptic cylinder with ellipsoid ends), inside < 0.  (A sheared collider:
    the point on the line from the core through ``p`` in its unscaled space.)"""
    fr = _ell_frame(col)
    if fr is None:
        return _ell_surface_radial(col, p)
    u, s = fr
    lr, lh = col[7], col[8]
    d = mu.v_sub(p, col[0])
    y = [mu.v_dot(d, u[i]) for i in range(3)]
    inside_local = ell_local(col, p)
    ky = max(-lh, min(lh, inside_local[1]))
    rho = math.sqrt(inside_local[0] ** 2 + (inside_local[1] - ky) ** 2 + inside_local[2] ** 2)
    inside = rho < lr
    half_w = lh * s[1]
    cands = []
    if lh > 0.0 and -half_w <= y[1] <= half_w:
        # the body: the ellipse across the axis at this height
        x2 = _closest_ellipse([y[0], y[2]], [lr * s[0], lr * s[2]])
        cands.append(((x2[0], y[1], x2[1]), (x2[0] / (lr * s[0]) ** 2, 0.0, x2[1] / (lr * s[2]) ** 2)))
    for end in ((-1.0, 1.0) if lh > 0.0 else (0.0,)):
        off = end * half_w
        e = [lr * s[0], lr * s[1], lr * s[2]]
        x = _closest_ellipse([y[0], y[1] - off, y[2]], e)
        if lh > 0.0 and (x[1] * end < 0.0):
            continue                                     # on the body side of the end: not the surface
        cands.append(((x[0], x[1] + off, x[2]), (x[0] / e[0] ** 2, x[1] / e[1] ** 2, x[2] / e[2] ** 2)))
    if not cands:
        return _ell_surface_radial(col, p)
    best = None
    for xc, g in cands:
        dist = math.sqrt(sum((y[i] - xc[i]) ** 2 for i in range(3)))
        if best is None or dist < best[0]:
            best = (dist, xc, g)
    dist, xc, g = best
    S = mu.v_add(col[0], mu.v_add(mu.v_add(mu.v_scale(u[0], xc[0]), mu.v_scale(u[1], xc[1])), mu.v_scale(u[2], xc[2])))
    N = mu.v_normalize(mu.v_add(mu.v_add(mu.v_scale(u[0], g[0]), mu.v_scale(u[1], g[1])), mu.v_scale(u[2], g[2])),
                       (0.0, 1.0, 0.0))
    return (-dist if inside else dist), S, N


def _ell_surface_radial(col, p):
    """the point of the drawn surface on the line from the core through ``p``
    (unscaled space) and the normal there; the distance along that normal"""
    u = ell_local(col, p)
    lr, lh = col[7], col[8]
    ky = max(-lh, min(lh, u[1]))
    v = (u[0], u[1] - ky, u[2])
    rho = mu.v_length(v)
    if rho < 1.0e-12:
        v, rho = (1.0, 0.0, 0.0), 1.0
    nl = mu.v_scale(v, 1.0 / rho)
    s = ell_world_point(col, (nl[0] * lr, ky + nl[1] * lr, nl[2] * lr))
    n = ell_world_normal(col, nl)
    return mu.v_dot(mu.v_sub(p, s), n), s, n


def ell_near(col, p, margin):
    """Cheap test: ``p`` may be within ``margin`` of the drawn surface (or inside)."""
    u = ell_local(col, p)
    ky = max(-col[8], min(col[8], u[1]))
    rho = math.sqrt(u[0] ** 2 + (u[1] - ky) ** 2 + u[2] ** 2)
    smin = min(mu.v_length(v) for v in col[5])
    return rho <= col[7] + margin / max(smin, 1.0e-12) + 1.0e-9


def ell_reach_along(col, n):
    """How far the surface is from the core along the world direction ``n``."""
    inv = col[6]
    u = (mu.v_dot(inv[0], n), mu.v_dot(inv[1], n), mu.v_dot(inv[2], n))
    lu = mu.v_length(u)
    return col[7] / lu if lu > 1.0e-12 else col[3]


def floor(center, normal, radius=0.0):
    """Floor collider (infinite plane, ``normal`` = up)."""
    return (center, mu.v_normalize(normal, (0.0, 1.0, 0.0)), 0.0, radius, "floor")


def core_point(col, p):
    """Closest point to ``p`` on the collider's core (center / segment)."""
    center, axis, half = col[0], col[1], col[2]
    if is_floor(col):
        return mu.v_sub(p, mu.v_scale(axis, mu.v_dot(mu.v_sub(p, center), axis)))
    if is_ell(col):
        u = ell_local(col, p)
        return ell_world_point(col, (0.0, max(-col[8], min(col[8], u[1])), 0.0))
    if half <= 0.0:
        return center
    s = mu.v_dot(mu.v_sub(p, center), axis)
    s = max(-half, min(half, s))
    return mu.v_add(center, mu.v_scale(axis, s))


def collider_velocity(now, before, p, h):
    """World velocity of the collider's surface that is at ``p`` now: its
    motion from ``before`` (the same collider one sub step earlier) over the
    time ``h``.  Zero for panels / bones and when ``before`` is unknown."""
    if before is None or h is None or h <= 0.0 or is_bone(now) or is_bone(before):
        return (0.0, 0.0, 0.0)
    if is_floor(now) or is_floor(before):
        if not (is_floor(now) and is_floor(before)):
            return (0.0, 0.0, 0.0)
        n = now[1]
        return mu.v_scale(n, mu.v_dot(mu.v_sub(now[0], before[0]), n) / h)
    if is_ell(now) or is_ell(before):
        # the same point of the collider's own (unscaled) space, then and now
        old = ell_world_point(as_ell(before), ell_local(as_ell(now), p))
        return mu.v_scale(mu.v_sub(p, old), 1.0 / h)
    s = max(-now[2], min(now[2], mu.v_dot(mu.v_sub(p, now[0]), now[1])))
    s_old = max(-before[2], min(before[2], s))
    core = mu.v_add(now[0], mu.v_scale(now[1], s))
    old = mu.v_add(before[0], mu.v_scale(before[1], s_old))
    return mu.v_scale(mu.v_sub(core, old), 1.0 / h)


def _out_along(col, p, d, thickness):
    """(target, normal): the surface point (plus ``thickness``) of a sphere /
    capsule / "ell" collider straight out from its core on the side of the
    world direction ``d``, across the collider's axis (the part of ``d``
    along the axis is left out: ``d`` says which side.  With it, a strand
    settling on the top of a spine capsule, its animation above and behind
    the top, was put straight up and stood on its head).  Along the axis
    only where ``d`` points (almost) along it."""
    e = as_ell(col)
    inv = e[6]
    u = ell_local(e, p)
    lh = e[8]
    ky = max(-lh, min(lh, u[1]))
    dl = [mu.v_dot(inv[0], d), mu.v_dot(inv[1], d), mu.v_dot(inv[2], d)]
    across = math.hypot(dl[0], dl[2])
    if across > SIDE_ACROSS * mu.v_length(dl):
        dl[1] = 0.0
    ln = mu.v_length(dl)
    if ln < 1.0e-9:
        return None
    dl = mu.v_scale(dl, 1.0 / ln)
    s = ell_world_point(e, (dl[0] * e[7], ky + dl[1] * e[7], dl[2] * e[7]))
    n = ell_world_normal(e, dl)
    return mu.v_add(s, mu.v_scale(n, thickness)), n


def _side_exit(col, p, n, pref, thickness):
    """(target, normal) out on the animation's side ``pref`` where the nearest
    way out ``n`` leads to the other side, or None (the nearest way).
    ``pref`` may be ("outside", direction): the animation itself is outside
    the collider at this moment - nothing pulls the strand in, it has swung
    round it - and the side is taken only up to SIDE_ALT_MAX_DEG between the
    two; further round, the strand stays on the side it is on.  (Put on the
    animation's side from anywhere, a strand that had swung round a thigh of
    a user's character and just touched it was sent across the thigh: 84 deg
    in a frame, 34 deg in the result.  Where the animation is inside, e.g. a
    strand starting deep in the waist collider, so excepted it went from
    side to side - up to 120 deg in a frame; turned towards the side by a
    limited angle per sub step instead (sliding round): up to 38 deg.)"""
    gate = False
    if len(pref) == 2 and pref[0] == "outside":
        gate, pref = True, pref[1]
    c = mu.v_dot(n, pref)
    if c >= 0.0 or (gate and c < math.cos(math.radians(SIDE_ALT_MAX_DEG))):
        return None
    return _out_along(col, p, pref, thickness)


def penetration(p, colliders, thickness, prefer=None):
    """Deepest penetration of point ``p``: (target point on the surface,
    outward normal) or None when ``p`` is outside every collider.
    ``prefer``: per collider (same order) None or a world direction: the side
    to come out on.  Where the nearest way out leads to the other side, ``p``
    goes out on that side instead, straight out from the core across the
    collider's axis (_out_along).  (Out only to the border of the sides, a
    strand kept being pulled in there and flailed - 299 turns of over 20 deg
    in a frame in a user's hair; to the mirror image of the nearest way on
    that side: 118 deg in a frame.)"""
    best = None
    for ci, col in enumerate(colliders or ()):
        if len(col) > 5 and col[4] == "tri":
            continue                                   # panels: segments only (bone_contact)
        pref = prefer[ci] if prefer else None
        if is_ell(col):
            if not ell_near(col, p, thickness):
                continue
            dist, s, n = ell_surface(col, p)
            if dist >= thickness:
                continue
            depth = thickness - dist
            target = mu.v_add(s, mu.v_scale(n, thickness))
            if pref is not None:
                alt = _side_exit(col, p, n, pref, thickness)
                if alt is not None:
                    target, n = alt
            if best is None or depth > best[0]:
                best = (depth, target, n)
            continue
        if is_floor(col):
            n = col[1]
            reach = col[3] + thickness
            height = mu.v_dot(mu.v_sub(p, col[0]), n)
            if height < reach:
                depth = reach - height
                if best is None or depth > best[0]:
                    best = (depth, mu.v_add(p, mu.v_scale(n, depth)), n)
            continue
        k = core_point(col, p)
        d = mu.v_sub(p, k)
        dist = mu.v_length(d)
        reach = col[3] + thickness
        if dist >= reach:
            continue
        depth = reach - dist
        if dist > 1.0e-9:
            n = mu.v_scale(d, 1.0 / dist)
        else:
            n = (0.0, 1.0, 0.0)
        target = mu.v_add(k, mu.v_scale(n, reach))
        if pref is not None:
            alt = _side_exit(col, p, n, pref, thickness)
            if alt is not None:
                target, n = alt
        if best is None or depth > best[0]:
            best = (depth, target, n)
    return None if best is None else (best[1], best[2])


def rotation_between(a, b):
    """Shortest rotation (quaternion) taking direction ``a`` to ``b``."""
    a = mu.v_normalize(a)
    b = mu.v_normalize(b)
    c = mu.v_cross(a, b)
    d = max(-1.0, min(1.0, mu.v_dot(a, b)))
    s = mu.v_length(c)
    if s < 1.0e-12:
        if d > 0.0:
            return mu.Q_IDENTITY
        # opposite: any perpendicular axis
        perp = mu.v_cross(a, (1.0, 0.0, 0.0))
        if mu.v_length(perp) < 1.0e-6:
            perp = mu.v_cross(a, (0.0, 1.0, 0.0))
        return mu.q_from_axis_angle(mu.v_normalize(perp), math.pi)
    return mu.q_from_axis_angle(mu.v_scale(c, 1.0 / s), math.atan2(s, d))


def closest_on_segments(p1, q1, p2, q2):
    """Closest points of segments p1-q1 and p2-q2: (s, c1, c2), s in [0, 1]
    along the first segment."""
    d1 = mu.v_sub(q1, p1)
    d2 = mu.v_sub(q2, p2)
    r = mu.v_sub(p1, p2)
    a = mu.v_dot(d1, d1)
    e = mu.v_dot(d2, d2)
    f = mu.v_dot(d2, r)
    if a <= 1.0e-12 and e <= 1.0e-12:
        return 0.0, p1, p2
    if a <= 1.0e-12:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = mu.v_dot(d1, r)
        if e <= 1.0e-12:
            s, t = max(0.0, min(1.0, -c / a)), 0.0
        else:
            b = mu.v_dot(d1, d2)
            den = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / den)) if den > 1.0e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
    return s, mu.v_add(p1, mu.v_scale(d1, s)), mu.v_add(p2, mu.v_scale(d2, t))


def bone_contact(pivot, r, bones, thickness, min_s=0.05, sides=None):
    """Deepest contact of the segment pivot -> pivot + r with bone colliders:
    (offset pivot -> contact point, target point, outward normal) or None.
    Contacts right at the pivot (s < ``min_s``) are ignored: the pivot is
    never pushed (a strand that starts on the other bone stays there)."""
    best = None
    tip = mu.v_add(pivot, r)
    for key, col in bones:
        if col[4] == "tri":
            hit = _panel_contact(pivot, tip, col, thickness, min_s, sides.get(key) if sides else None)
            if hit is not None and (best is None or hit[0] > best[0]):
                best = hit
            continue
        center, axis, half = col[0], col[1], col[2]
        a = mu.v_sub(center, mu.v_scale(axis, half))
        b = mu.v_add(center, mu.v_scale(axis, half))
        s, c, k = closest_on_segments(pivot, tip, a, b)
        if s < min_s:
            continue
        d = mu.v_sub(c, k)
        dist = mu.v_length(d)
        reach = col[3] + thickness
        if dist >= reach:
            continue
        if dist > 1.0e-9:
            n = mu.v_scale(d, 1.0 / dist)
        else:
            n = mu.v_normalize(mu.v_cross(r, axis), (0.0, 0.0, 1.0))
        depth = reach - dist
        if best is None or depth > best[0]:
            best = (depth, mu.v_sub(c, pivot), mu.v_add(k, mu.v_scale(n, reach)), n)
    return None if best is None else best[1:]


def _panel_normal(col):
    a, b, c = col[5]
    nt = mu.v_cross(mu.v_sub(b, a), mu.v_sub(c, a))
    ln = mu.v_length(nt)
    return None if ln < 1.0e-12 else mu.v_scale(nt, 1.0 / ln)


def panel_side(pivot, tip, col):
    """+1 / -1: the side of the panel the segment is on, when it is in
    front of the panel's face and does not pierce it; else None."""
    nt = _panel_normal(col)
    if nt is None:
        return None
    a, b, c = col[5]
    _s, x, k = segment_triangle(pivot, tip, a, b, c)
    d = mu.v_sub(x, k)
    dist = mu.v_length(d)
    if dist < 1.0e-9:
        return None                                     # pierces it
    across = mu.v_dot(d, nt)
    if abs(abs(across) - dist) > 1.0e-6 * max(1.0, dist):
        return None                                     # beside an edge: no clear side
    return 1 if across > 0.0 else -1


def _panel_contact(pivot, tip, col, thickness, min_s, side=None):
    """(depth, offset pivot -> contact, target, normal) for a panel, or None.
    ``side``: the side the segment was on before (+1 / -1 along the panel's
    normal), None = the side of the pivot."""
    a, b, c = col[5]
    s, x, k = segment_triangle(pivot, tip, a, b, c)
    if s < min_s:
        return None
    nt = _panel_normal(col)
    if nt is None:
        return None
    if side is not None:
        if side < 0:
            nt = mu.v_scale(nt, -1.0)                   # the side it came from
    elif mu.v_dot(mu.v_sub(pivot, a), nt) < 0.0:
        nt = mu.v_scale(nt, -1.0)                       # the normal points to the pivot's side
    reach = col[3] + thickness
    d = mu.v_sub(x, k)
    dist = mu.v_length(d)
    across = mu.v_dot(d, nt)
    # through the panel: the closest point is the projection of the contact
    # point inside the triangle, on the other side of the pivot
    through = across < 0.0 and abs(abs(across) - dist) < 1.0e-6 * max(1.0, dist) or dist < 1.0e-9
    if through:
        # bring the TIP (the point furthest through) back to the pivot's side
        across_tip = mu.v_dot(mu.v_sub(tip, a), nt)
        if across_tip < reach:
            return reach - across_tip + reach, mu.v_sub(tip, pivot),                 mu.v_add(tip, mu.v_scale(nt, reach - across_tip)), nt
        n = nt
        depth = reach + dist
    else:
        if dist >= reach:
            return None
        n = mu.v_scale(d, 1.0 / dist)
        depth = reach - dist
    return depth, mu.v_sub(x, pivot), mu.v_add(k, mu.v_scale(n, reach)), n


def push_segment(q, w, pivot, tip_local, length, colliders, thickness, iterations=2, sides=None, prefer=None):
    """Resolve one rotating segment.  Returns (q, w, hit).  ``sides``: dict
    (collider index -> +1 / -1) this segment's remembered side of every panel,
    updated here (pass the same dict every sub step).  ``prefer``: dict
    (collider index -> world direction) the side of a sphere / capsule the
    tip comes out on when it is inside (see penetration)."""
    hit = False
    pref = None
    if prefer:
        # by the index in the list as given
        pref = [prefer.get(k) for k, c in enumerate(colliders or ()) if not (is_floor(c) or is_bone(c))]
    floors = [c for c in colliders or () if is_floor(c)]
    if floors:
        colliders = [c for c in colliders if not is_floor(c)]
        for fl in floors:
            q, w, h = _push_floor(q, w, pivot, tip_local, length, fl, thickness)
            hit = hit or h
    bones = [(k, c) for k, c in enumerate(colliders or ()) if is_bone(c)]
    if bones:
        iterations = max(iterations, 8)        # until out (stops at the first pass without a contact)
        colliders = [c for c in colliders if not is_bone(c)]
    for _ in range(iterations):
        r = mu.v_scale(mu.q_rotate(q, tip_local), length)
        tip = mu.v_add(pivot, r)
        res = penetration(tip, colliders, thickness, pref)
        if res is None:
            if bones:
                contact = bone_contact(pivot, r, bones, thickness, sides=sides)
                if contact is not None:
                    hit = True
                    q, w = _push_contact(q, w, pivot, contact)
                    continue
            break
        hit = True
        target, n = res
        new_dir = mu.v_sub(target, pivot)
        if mu.v_length(new_dir) < 1.0e-9:
            break
        q = mu.q_normalize(mu.q_mul(rotation_between(r, new_dir), q))
        # remove the angular velocity that moves the tip into the collider
        r = mu.v_scale(mu.q_rotate(q, tip_local), length)
        rxn = mu.v_cross(r, n)
        l2 = mu.v_dot(rxn, rxn)
        vn = mu.v_dot(mu.v_cross(w, r), n)
        if vn < 0.0 and l2 > 1.0e-12:
            w = mu.v_add(w, mu.v_scale(rxn, -vn / l2))
    if sides is not None and bones:
        r = mu.v_scale(mu.q_rotate(q, tip_local), length)
        tip = mu.v_add(pivot, r)
        for key, col in bones:
            if col[4] == "tri":
                side = panel_side(pivot, tip, col)
                if side is not None:
                    sides[key] = side
    return q, w, hit


def _push_floor(q, w, pivot, tip_local, length, fl, thickness, margin=1.0e-6):
    """Turn the segment about its pivot so its tip is ``radius + thickness``
    above the floor (the exact angle: aiming at the point above the tip does
    not move a segment that points straight into the floor), and remove the
    angular velocity that moves the tip down."""
    n = fl[1]
    reach = fl[3] + thickness
    r = mu.v_scale(mu.q_rotate(q, tip_local), length)
    h_pivot = mu.v_dot(mu.v_sub(pivot, fl[0]), n)
    if h_pivot + mu.v_dot(r, n) >= reach:
        return q, w, False
    d = mu.v_scale(r, 1.0 / length)
    want = min(1.0, (reach - h_pivot) / length + margin)        # dot(new direction, n)
    cur = mu.v_dot(d, n)
    axis = mu.v_cross(d, n)
    if mu.v_length(axis) < 1.0e-9:
        axis = mu.v_cross(n, (1.0, 0.0, 0.0))
        if mu.v_length(axis) < 1.0e-6:
            axis = mu.v_cross(n, (0.0, 0.0, 1.0))
    axis = mu.v_normalize(axis)
    angle = math.acos(max(-1.0, min(1.0, want))) - math.acos(max(-1.0, min(1.0, cur)))
    # turning about d x n by -angle brings d towards n
    rot = mu.q_from_axis_angle(axis, -angle)
    q = mu.q_normalize(mu.q_mul(rot, q))
    r = mu.q_rotate(rot, r)
    rxn = mu.v_cross(r, n)
    l2 = mu.v_dot(rxn, rxn)
    vn = mu.v_dot(mu.v_cross(w, r), n)
    if vn < 0.0 and l2 > 1.0e-12:
        w = mu.v_add(w, mu.v_scale(rxn, -vn / l2))
    return q, w, True


def _push_contact(q, w, pivot, contact, margin=1.0e-4):
    """Turn the segment about its pivot just enough that the contact point
    moves out by the penetration depth along ``n``, and remove the angular
    velocity that moves it into the bone.

    A turn moves the contact point across the segment only, so the needed
    move is depth / (the part of ``n`` across the segment) - aiming the
    contact point at the target point instead overshoots or falls short when
    the target is nearer to / further from the pivot (it left 18 % of random
    crossings inside, up to 0.97 of the reach)."""
    rc, target, n = contact
    r_len = mu.v_length(rc)
    if r_len < 1.0e-9:
        return q, w
    depth = mu.v_dot(mu.v_sub(target, mu.v_add(pivot, rc)), n)
    if depth <= 0.0:
        return q, w
    u = mu.v_scale(rc, 1.0 / r_len)
    n_perp = mu.v_sub(n, mu.v_scale(u, mu.v_dot(n, u)))
    np_len = mu.v_length(n_perp)
    if np_len < 1.0e-6:
        return q, w                     # pushed straight along the segment: a turn cannot help
    side = mu.v_scale(n_perp, 1.0 / np_len)
    move = (depth + margin) / np_len
    angle = math.atan2(move, r_len)
    rot = mu.q_from_axis_angle(mu.v_cross(u, side), angle)
    q = mu.q_normalize(mu.q_mul(rot, q))
    rc = mu.q_rotate(rot, rc)
    rxn = mu.v_cross(rc, n)
    l2 = mu.v_dot(rxn, rxn)
    vn = mu.v_dot(mu.v_cross(w, rc), n)
    if vn < 0.0 and l2 > 1.0e-12:
        w = mu.v_add(w, mu.v_scale(rxn, -vn / l2))
    return q, w


def push_point(x, v, colliders, thickness):
    """Translation: move a point out and remove its inward velocity."""
    res = penetration(x, colliders, thickness)
    if res is None:
        return x, v, False
    target, n = res
    vn = mu.v_dot(v, n)
    if vn < 0.0:
        v = mu.v_sub(v, mu.v_scale(n, vn))
    return target, v, True
