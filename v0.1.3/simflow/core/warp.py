# -*- coding: utf-8 -*-
"""Warp absorption (camera cheats) - pure Python, no Maya import.

Animators often move a character a lot in ONE frame to fit the camera
("1F warp"): on screen it barely moves, but a world-space simulation sees a
huge acceleration and the swinging parts fly around.

Idea: find the frames whose motion jumps out of the motion around them, and
remove that jump from the input before solving::

    world = C(t) * sim          C: rigid transform, identity before the first warp,
                                changes only on warp frames (accumulated jumps)

The solver runs on ``C(t)^-1 * input``, in which the warp does not exist, and
the result is moved back by ``C(t)``.  So the swinging parts warp together
with the character and keep swinging as they were (no reset).

* Detection works on the root controller of the unit (chain root / the single
  controller): pivot position and world orientation.  A block of 1..N frames
  is a warp when its motion differs from the motion right BEFORE and right
  AFTER it by more than the threshold, while the motion before and after are
  similar (a sudden start - e.g. a dash - changes the speed for good and is
  NOT a warp; a steady run has no jump at all).
* Only the rotation about the gravity axis ("yaw") is absorbed, so gravity
  stays the same in the simulation space.  Translation is absorbed fully.
* Optional camera check: when the jump is clearly visible on screen (more
  than ``warp_screen`` % of the frame width) it is a real fast action, not a
  cheat, and is NOT absorbed.
* Frames can be added / ignored by hand.
"""
from __future__ import annotations

import math
import re

from ..solvers import collision as colmod
from ..solvers.base_solver import SolverInput, SolverResult
from ..utils import math_utils as mu

IDENTITY = (mu.Q_IDENTITY, (0.0, 0.0, 0.0))


def parse_frames(text):
    """"37, 52 60-62" -> [37, 52, 60, 61, 62].  Raises ValueError."""
    out = []
    for tok in re.split(r"[,\s、，]+", (text or "").strip()):
        if not tok:
            continue
        m = re.match(r"^(-?\d+)-(-?\d+)$", tok)
        if m:
            a, b = int(m.group(1)), int(m.group(2))
            if b < a:
                a, b = b, a
            if b - a > 10000:
                raise ValueError(tok)
            out.extend(range(a, b + 1))
        else:
            out.append(int(float(tok)))
    return sorted(set(out))


def up_axis(settings):
    return mu.v_normalize(mu.v_scale(tuple(settings.gravity_direction), -1.0), (0.0, 1.0, 0.0))


# ---------------------------------------------------------------------------
# camera projection
# ---------------------------------------------------------------------------
def project(cam, p):
    """Screen position of world point ``p`` in fractions of the frame width
    (0 = centre).  ``cam`` = (inverse world matrix, tan(horizontal fov / 2)).
    None when the point is behind the camera."""
    inv, tan_h = cam
    pc = mu.m_transform_point(inv, p)
    depth = -pc[2]
    if depth < 1.0e-6 or tan_h <= 0.0:
        return None
    k = 1.0 / (depth * 2.0 * tan_h)
    return (pc[0] * k, pc[1] * k)


# ---------------------------------------------------------------------------
# detection
# ---------------------------------------------------------------------------
def _vel(pos):
    return [None] + [mu.v_sub(pos[i], pos[i - 1]) for i in range(1, len(pos))]


def _ang(rot):
    out = [None]
    prev = rot[0]
    for i in range(1, len(rot)):
        q = mu.q_align(rot[i], prev)
        out.append(mu.q_to_rotvec(mu.q_mul(q, mu.q_conj(prev))))
        prev = q
    return out


def _excess(vel, i, w):
    """(excess of the block i..i+w-1, |motion before - motion after|) or None."""
    if i - 1 < 1 or i + w >= len(vel):
        return None
    vin, vout = vel[i - 1], vel[i + w]
    avg = mu.v_scale(mu.v_add(vin, vout), 0.5)
    total = (0.0, 0.0, 0.0)
    for k in range(i, i + w):
        total = mu.v_add(total, vel[k])
    return mu.v_sub(total, mu.v_scale(avg, float(w))), mu.v_length(mu.v_sub(vin, vout)), avg


def _is_jump(vel, i, w, threshold):
    r = _excess(vel, i, w)
    if r is None:
        return False, 0.0
    ex, change, avg = r
    size = mu.v_length(ex)
    if size < threshold:
        return False, size
    # before / after similar (otherwise it is a change of speed, e.g. a dash start)
    if change > 0.5 * size:
        return False, size
    # the block must start and end with the jump (not a normal frame next to it)
    per = 0.5 * size / w
    if mu.v_length(mu.v_sub(vel[i], vel[i - 1])) < per or mu.v_length(mu.v_sub(vel[i + w - 1], vel[i + w])) < per:
        return False, size
    return True, size


def _screen_jump(screen, i, w):
    """Largest on-screen excess (fraction of the frame width) of the block,
    over the check points.  None when it cannot be judged."""
    best = None
    npts = len(screen[0]) if screen and screen[0] else 0
    for k in range(npts):
        pts = [row[k] for row in screen]
        if any(pts[j] is None for j in range(max(0, i - 2), min(len(pts), i + w + 1))):
            continue
        vel = [None] + [(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1], 0.0) if pts[j] is not None
                        and pts[j - 1] is not None else (0.0, 0.0, 0.0) for j in range(1, len(pts))]
        r = _excess(vel, i, w)
        if r is None:
            continue
        size = mu.v_length(r[0])
        best = size if best is None else max(best, size)
    return best


class WarpPlan(object):
    """Result of :func:`plan`: per frame compensation and the found warps."""

    def __init__(self, comp, warps):
        self.comp = comp            # per frame (quat, translation): world = C * sim
        self.warps = warps          # list of dict (frame, frames, distance, angle, screen, status)

    @property
    def active(self):
        return any(w["status"] in ("absorbed", "manual") for w in self.warps)

    def absorbed_frames(self):
        return [w["frame"] for w in self.warps if w["status"] in ("absorbed", "manual")]


def plan(frames, pos, rot, settings, screen=None):
    """Find warps of the root motion ``pos`` / ``rot`` (world, per frame) and
    build the compensation.

    ``screen``: per frame list of check-point screen positions (see
    :func:`project`), or None (no camera check).
    ``settings``: SimSettings (warp_* fields, gravity_direction)."""
    n = len(frames)
    vel, ang = _vel(pos), _ang(rot)
    dist_thr = float(settings.warp_distance)
    ang_thr = math.radians(float(settings.warp_angle))
    max_w = max(1, min(3, int(settings.warp_frames)))
    add = set(parse_frames(settings.warp_add))
    ignore = set(parse_frames(settings.warp_ignore))
    index = {int(round(f)): i for i, f in enumerate(frames)}

    warps = []
    blocks = []                 # (start index, length)
    i = 2
    while i < n - 1:
        found = None
        for w in range(1, max_w + 1):
            jt, dt = _is_jump(vel, i, w, dist_thr)
            jr, dr = _is_jump(ang, i, w, ang_thr)
            if jt or jr:
                found = (w, dt, dr)
                break
        if found is None:
            i += 1
            continue
        w, dt, dr = found
        fr = [int(round(frames[k])) for k in range(i, i + w)]
        info = {"frame": fr[0], "frames": fr, "distance": dt, "angle": math.degrees(dr), "screen": None,
                "status": "absorbed"}
        if ignore.intersection(fr):
            info["status"] = "ignored"
        elif add.intersection(fr):
            info["status"] = "manual"
        elif screen is not None:
            s = _screen_jump(screen, i, w)
            info["screen"] = None if s is None else s * 100.0
            if s is not None and s * 100.0 > float(settings.warp_screen):
                info["status"] = "visible"
        warps.append(info)
        if info["status"] in ("absorbed", "manual"):
            blocks.append((i, w))
        i += w
    # frames added by hand that were not found: a 1 frame warp there
    taken = set(f for wp in warps for f in wp["frames"])
    for f in sorted(add - taken - ignore):
        k = index.get(f)
        if k is None or k < 1:
            continue
        r = _excess(vel, k, 1)
        dt = mu.v_length(r[0]) if r else 0.0
        warps.append({"frame": f, "frames": [f], "distance": dt, "angle": 0.0, "screen": None, "status": "manual"})
        blocks.append((k, 1))
    warps.sort(key=lambda wp: wp["frame"])
    return WarpPlan(_compensation(n, pos, vel, ang, blocks, settings), warps)


def _neighbours(vel, i, w):
    """Average motion of the frames before / after a block (with fallbacks
    at the ends of the range, for frames added by hand)."""
    cand = []
    if i - 1 >= 1:
        cand.append(vel[i - 1])
    if i + w < len(vel):
        cand.append(vel[i + w])
    if not cand:
        return (0.0, 0.0, 0.0)
    return mu.v_scale(cand[0] if len(cand) == 1 else mu.v_add(cand[0], cand[1]), 1.0 / len(cand))


def _compensation(n, pos, vel, ang, blocks, settings):
    s = max(0.0, min(100.0, float(settings.warp_strength))) / 100.0
    up = up_axis(settings)
    step = {}
    for i, w in blocks:
        vavg, ravg = _neighbours(vel, i, w), _neighbours(ang, i, w)
        for k in range(i, i + w):
            step[k] = (mu.v_scale(mu.v_sub(vel[k], vavg), s), mu.v_dot(mu.v_sub(ang[k], ravg), up) * s)
    comp = []
    cq, ct = IDENTITY
    for k in range(n):
        if k in step:
            et, yaw = step[k]
            rx = mu.q_from_axis_angle(up, yaw)
            a = mu.v_sub(pos[k], et)          # where the root would be without the jump
            b = pos[k]
            # X(x) = Rx (x - a) + b ; C <- X o C
            cq = mu.q_normalize(mu.q_mul(rx, cq))
            ct = mu.v_add(mu.q_rotate(rx, mu.v_sub(ct, a)), b)
        comp.append((cq, ct))
    return comp


# ---------------------------------------------------------------------------
# applying
# ---------------------------------------------------------------------------
def to_sim_point(c, p):
    cq, ct = c
    return mu.q_rotate(mu.q_conj(cq), mu.v_sub(p, ct))


def to_sim_rot(c, q):
    return mu.q_mul(mu.q_conj(c[0]), q)


def _ell_to_sim(c, col):
    """An "ell" (non-uniformly scaled) collider moved into the simulation space."""
    A = tuple(mu.q_rotate(mu.q_conj(c[0]), v) for v in col[5])
    return colmod.ellipsoid(to_sim_point(c, col[0]), A, col[7], col[8])


def to_world_point(c, p):
    cq, ct = c
    return mu.v_add(mu.q_rotate(cq, p), ct)


def to_world_rot(c, q):
    return mu.q_mul(c[0], q)


def sim_input(inp, comp):
    """SolverInput moved into the simulation space."""
    def pts(seq):
        return [to_sim_point(c, p) for c, p in zip(comp, seq)]

    def rots(seq):
        return [to_sim_rot(c, q) for c, q in zip(comp, seq)]
    cols = None
    if inp.colliders:
        cols = []
        for c, row in zip(comp, inp.colliders):
            cols.append([_ell_to_sim(c, col) if colmod.is_ell(col) else
                         (to_sim_point(c, col[0]), mu.q_rotate(mu.q_conj(c[0]), col[1]), col[2], col[3]) +
                         ((col[4], tuple(to_sim_point(c, p) for p in col[5])) if len(col) > 5 and col[4] == "tri"
                          else tuple(col[4:]))
                         for col in row])
    scene_root = (pts(inp.scene_root[0]), rots(inp.scene_root[1])) if inp.scene_root else None
    return SolverInput(fps=inp.fps, frames=list(inp.frames), target_pos=pts(inp.target_pos),
                       target_rot=rots(inp.target_rot), orig_pos=pts(inp.orig_pos), orig_rot=rots(inp.orig_rot),
                       tip_local=inp.tip_local, name=inp.name, colliders=cols, scene_root=scene_root,
                       hangs=inp.hangs)


def world_result(raw, comp):
    """Solver result moved back to world space."""
    return SolverResult(frames=list(raw.frames),
                        pos=[to_world_point(c, p) for c, p in zip(comp, raw.pos)],
                        rot=[to_world_rot(c, q) for c, q in zip(comp, raw.rot)],
                        info=dict(raw.info))


def describe(warps):
    """Short Japanese summary for the UI / log."""
    parts = []
    absorbed = [w for w in warps if w["status"] in ("absorbed", "manual")]
    visible = [w for w in warps if w["status"] == "visible"]
    ignored = [w for w in warps if w["status"] == "ignored"]

    def fr(ws):
        return "、".join(("%d" % w["frame"]) if len(w["frames"]) == 1 else "%d〜%d" % (w["frames"][0], w["frames"][-1])
                        for w in ws)
    if absorbed:
        parts.append("吸収: %s" % fr(absorbed))
    if visible:
        parts.append("画面で見えている動きなので吸収しない: %s" % fr(visible))
    if ignored:
        parts.append("除外: %s" % fr(ignored))
    return " / ".join(parts) if parts else "ワープは見つかりませんでした"
