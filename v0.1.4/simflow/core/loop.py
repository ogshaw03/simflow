# -*- coding: utf-8 -*-
"""ループ: the range is one cycle of a looping animation (pure Python).

The calculation runs through earlier cycles of the range first, in place of
the pre-roll (the pre-roll input is filled from the range itself), so it
reaches the range already swinging the way the loop swings, and the
result's last frame leads into its first.

The cycle: when every controller (and collider) is at the same pose on the
last frame as on the first, the last frame IS the first frame of the next
cycle (a walk keyed 1-25 with 25 = 1): the cycle is the range without its
last frame.  Otherwise the cycle is the whole range (1-24, 25 = 1 again).
"""
from __future__ import annotations

import math

LOOP_CYCLES = 3              # cycles run before the range ...
LOOP_MIN_SECONDS = 10.0      # ... and at least this long: a simulation with 減衰 30 took 8-10 s to repeat
                             # itself (seam 0.16 deg after 8 s, 0.01 after 10); オーバーラップ 2 cycles
LOOP_MAX_FRAMES = 3000       # never more (a long range)
LOOP_POST = 2                # frames of the next cycle calculated after the range
POSE_TOL = 1.0e-3           # matrix entries / cm: the same pose


def _flat(x, out):
    if isinstance(x, (int, float)) and not isinstance(x, bool):
        out.append(float(x))
    elif isinstance(x, (list, tuple)):
        for y in x:
            _flat(y, out)
    elif hasattr(x, "world"):
        _flat(x.world, out)
    return out


def same_pose(a, b, tol=POSE_TOL):
    fa, fb = _flat(a, []), _flat(b, [])
    if len(fa) != len(fb):
        return False
    return all(abs(x - y) <= tol * max(1.0, abs(x), abs(y)) for x, y in zip(fa, fb))


def loop_preroll(frames, ctl_rows, col_rows=(), fps=24.0):
    """(pre-roll frames, the range's frame index of each, info).

    ``ctl_rows`` / ``col_rows``: per controller / collider, its samples over
    ``frames`` (anything with numbers, or samples with ``world``)."""
    n = len(frames)
    closed = n > 2 and all(same_pose(rows[0], rows[-1]) for rows in list(ctl_rows) + list(col_rows) if rows)
    period = n - 1 if closed else n
    step = (frames[-1] - frames[0]) / float(n - 1) if n > 1 else 1.0
    cycles = max(LOOP_CYCLES, int(math.ceil(LOOP_MIN_SECONDS * fps / max(1.0, period * step))))
    cycles = max(1, min(cycles, LOOP_MAX_FRAMES // max(1, period)))
    total = cycles * period
    pre_frames = [frames[0] - k * step for k in range(total, 0, -1)]
    pre_idx = [(-k) % period for k in range(total, 0, -1)]
    # and the next cycle's first frames after the range (not baked): the last
    # frame is then calculated as in any other cycle (the interpolation looks
    # one frame ahead; at the end of the range it had nothing to look at and
    # the tail of a chain came out 0.07 deg off at the seam)
    post_frames = [frames[-1] + k * step for k in range(1, LOOP_POST + 1)]
    post_idx = [(n - 1 + k) % period for k in range(1, LOOP_POST + 1)]
    return pre_frames, pre_idx, {"closed": closed, "period": period, "cycles": cycles,
                                 "post_frames": post_frames, "post_idx": post_idx}


def seam(first, last):
    """How far the result's last frame is from leading into its first: the
    largest difference of the matrices' numbers (0 = seamless)."""
    fa, fb = _flat(first, []), _flat(last, [])
    return max([abs(x - y) for x, y in zip(fa, fb)] or [0.0])
