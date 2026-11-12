# -*- coding: utf-8 -*-
"""「手の回転に合わせて回す」 for a jump rope.

The rope's controllers have no keys of their own (the hands only).  Real
jump ropes are spun by the small circles of the wrists; letting the physics
alone spin the rope up depends on the exact hand motion and on every touch of
the floor / body (hard to control).  Instead the rope's TARGET (what 硬さ
pulls it to) is made to turn with the hands:

* the hands' middle point circles round a centre that moves with the
  character (the average position over a 1 s window);  its angle around the
  line between the hands is the turning angle of the rope;
* the target rope is an arc in the plane through both hands and that
  direction, with the rope's length (the further the hands, the flatter);
* where the hands do not circle (before the jump, after it) the arc turns
  back to hanging straight down (the ordinary target of a rope without keys
  is the straight line between the hands: 硬さ would lift the rope there).

The physics adds the lag, the whip and the collisions.

Pure Python, no Maya dependency.
"""
from __future__ import annotations

import math

from ..utils import math_utils as mu

WINDOW_SECONDS = 0.5       # half width of the averaging window for the circle centre
MIN_RADIUS = 0.01          # circles smaller than this x rope length: not turning
FULL_RADIUS = 0.03         # ... from this x rope length: fully turning


def _arc_points(D, L, fractions):
    """Points (x along the hands, y out of the line) of an arc of length L
    between (0, 0) and (D, 0) at the given length fractions."""
    if L <= D * 1.0005:
        return [(D * u, 0.0) for u in fractions]
    # circular arc with chord D and length L: L / D = theta / sin(theta)
    lo, hi = 1.0e-6, math.pi - 1.0e-6
    ratio = L / D
    if ratio >= hi / math.sin(hi):
        theta = hi
    else:
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if mid / math.sin(mid) < ratio:
                lo = mid
            else:
                hi = mid
        theta = 0.5 * (lo + hi)
    R = L / (2.0 * theta)               # radius, theta = half the opening angle
    cy = -R * math.cos(theta)           # centre below the chord (arc bulges to +y)
    out = []
    for u in fractions:
        a = -theta + 2.0 * theta * u    # from the first hand to the second
        out.append((D * 0.5 + R * math.sin(a), cy + R * math.cos(a)))
    return out


def _smoothstep(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def spin_targets(grip_a, grip_b, rest, fps, up=(0.0, 1.0, 0.0)):
    """Targets for the middle points ([point][frame]) and info.

    ``grip_a`` / ``grip_b``: per frame positions of the hands.  ``rest``:
    segment lengths along the rope (grip -> grip)."""
    n_frames = len(grip_a)
    L = sum(rest)
    along = [0.0]
    for x in rest:
        along.append(along[-1] + x)
    fractions = [along[i] / L for i in range(1, len(rest))]
    mids = [mu.v_lerp(a, b, 0.5) for a, b in zip(grip_a, grip_b)]
    half = max(1, int(round(WINDOW_SECONDS * fps)))
    out = [[None] * n_frames for _ in fractions]
    weights, angles = [], []
    prev_dir = None
    for f in range(n_frames):
        lo, hi = max(0, f - half), min(n_frames - 1, f + half)
        centre = mu.v_scale(tuple(sum(mids[k][c] for k in range(lo, hi + 1)) for c in range(3)), 1.0 / (hi - lo + 1))
        axis = mu.v_sub(grip_b[f], grip_a[f])
        D = mu.v_length(axis)
        axis_n = mu.v_scale(axis, 1.0 / D) if D > 1.0e-9 else (1.0, 0.0, 0.0)
        off = mu.v_sub(mids[f], centre)
        off = mu.v_sub(off, mu.v_scale(axis_n, mu.v_dot(off, axis_n)))
        r = mu.v_length(off)
        w = _smoothstep((r / L - MIN_RADIUS) / (FULL_RADIUS - MIN_RADIUS))
        # straight down, across the hands' line
        down = mu.v_scale(up, -1.0)
        down = mu.v_normalize(mu.v_sub(down, mu.v_scale(axis_n, mu.v_dot(down, axis_n))), prev_dir or (0.0, -1.0, 0.0))
        d_hand = mu.v_scale(off, 1.0 / r) if r > 1.0e-9 else down
        # the turning angle around the hands' line, from straight down
        ang = math.atan2(mu.v_dot(mu.v_cross(down, d_hand), axis_n), mu.v_dot(down, d_hand))
        weights.append(w)
        angles.append(ang)
        # where the hands circle less, the arc turns back towards hanging down
        d = mu.q_rotate(mu.q_from_axis_angle(axis_n, w * ang), down)
        prev_dir = d
        arc = _arc_points(D, L, fractions)
        for k, (x, y) in enumerate(arc):
            out[k][f] = mu.v_add(grip_a[f], mu.v_add(mu.v_scale(axis_n, x), mu.v_scale(d, y)))
    turns = 0.0
    for a0, a1, w in zip(angles, angles[1:], weights[1:]):
        if w <= 0.0:
            continue
        da = a1 - a0
        while da > math.pi:
            da -= 2.0 * math.pi
        while da < -math.pi:
            da += 2.0 * math.pi
        turns += da / (2.0 * math.pi)
    info = {"spin_frames": sum(1 for w in weights if w >= 1.0), "spin_turns": abs(turns)}
    return out, info
