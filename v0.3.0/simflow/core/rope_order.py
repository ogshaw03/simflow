# -*- coding: utf-8 -*-
"""Order of a rope's controllers ("縄・ひも"): grip, controllers along the
rope, grip - found from the rig, not from the selection order (a box /
outliner selection gives any order, and a rope joined in the wrong order
flies apart: the user's "順番もなく散っていく").

1. Hierarchy: the rope controllers hang under the two grips (half of the
   rope under each).  Grips = the two registered controllers that are
   parents of the others; each half ordered by the distance from its grip.
2. Motion: the rope controllers follow the grips in the rig (a rope rig
   whose controllers ride on groups that follow a blend of the two hands -
   the user's rig: the controller next to a grip moves exactly like that
   grip, the middle one like a mix of both).  Every controller's motion over
   the range is fitted as ``w * grip A + (1 - w) * grip B``: the order along
   the rope is the order of ``w`` (ties: distance from the nearer grip).
   The grips are the first and last of the list.
3. Otherwise the list order is kept.

Ordering by position alone (shortest path) is NOT used: a U-shaped rope
with the hands close together is shorter zig-zagging between its two arms.
"""
from __future__ import annotations

from ..utils import math_utils as mu

MOTION_MIN_SPREAD = 0.3     # the weights of the rope controllers must spread at least this much
MOTION_MIN_GRIP_MOVE = 5.0  # cm: the grips must move apart / around each other this much


def _positions(nodes, frames):
    """World pivots of ``nodes`` at ``frames`` ([node][frame]), SIMFLOW layers muted."""
    import maya.cmds as cmds
    from . import scene_manager
    out = []
    with scene_manager.tool_layers_muted():
        for n in nodes:
            rp = cmds.getAttr(n + ".rotatePivot")[0]
            out.append([mu.m_transform_point(cmds.getAttr(n + ".worldMatrix[0]", time=f), rp) for f in frames])
    return out


def grip_weights(paths, a, b):
    """Least squares ``w`` per point: path - B = w (A - B) + const.  None when
    the grips do not move relative to each other (no information)."""
    n = len(paths[a])
    d = [mu.v_sub(x, y) for x, y in zip(paths[a], paths[b])]
    dm = [sum(v[i] for v in d) / n for i in range(3)]
    den = sum(mu.v_dot(mu.v_sub(v, dm), mu.v_sub(v, dm)) for v in d)
    spread = max(mu.v_length(mu.v_sub(v, dm)) for v in d)
    if spread < MOTION_MIN_GRIP_MOVE or den <= 1.0e-9:
        return None
    out = {}
    for i, path in enumerate(paths):
        if i in (a, b):
            continue
        r = [mu.v_sub(x, y) for x, y in zip(path, paths[b])]
        rm = [sum(v[k] for v in r) / n for k in range(3)]
        num = sum(mu.v_dot(mu.v_sub(x, rm), mu.v_sub(y, dm)) for x, y in zip(r, d))
        out[i] = num / den
    return out


def order_points(pts, grips=None, halves=None, paths=None):
    """(order, how).

    ``pts``: start frame positions.  ``grips``: the two grip indices,
    ``halves``: {grip index: [indices of the controllers under it]} (rule 1),
    ``paths``: positions over the range per point (rule 2)."""
    n = len(pts)

    def dist(i, g):
        return mu.v_length(mu.v_sub(pts[i], pts[g]))
    if grips and len(grips) == 2 and halves is not None:
        a, b = grips
        under = set(halves.get(a, [])) | set(halves.get(b, []))
        if under == set(range(n)) - {a, b} and halves.get(a) and halves.get(b):
            half_a = sorted(halves[a], key=lambda i: (dist(i, a), i))
            half_b = sorted(halves[b], key=lambda i: (-dist(i, b), i))
            return [a] + half_a + half_b + [b], "持ち手の子から"
    if paths is not None and n >= 3:
        a, b = (grips if grips and len(grips) == 2 else (0, n - 1))
        w = grip_weights(paths, a, b)
        if w and max(w.values()) - min(w.values()) >= MOTION_MIN_SPREAD:
            mids = sorted(w, key=lambda i: (-round(w[i], 2), dist(i, a) if w[i] >= 0.5 else -dist(i, b), i))
            return [a] + mids + [b], "持ち手と一緒の動き方から"
    return list(range(n)), "リストの順"


def rope_order(nodes, frames):
    """(order of ``nodes`` indices, how it was found).  The first and the
    last node are the grips unless the hierarchy says otherwise."""
    frames = list(frames)
    if len(frames) > 240:                       # enough to see how they move
        step = len(frames) / 240.0
        frames = [frames[int(k * step)] for k in range(240)] + [frames[-1]]
    paths = _positions(nodes, frames)
    pts = [p[0] for p in paths]
    parents = [i for i, a in enumerate(nodes) if any(b.startswith(a + "|") for b in nodes if b != a)]
    if len(parents) == 2:
        a, b = parents
        halves = {a: [], b: []}
        for i, node in enumerate(nodes):
            if i in parents:
                continue
            for g in parents:
                if node.startswith(nodes[g] + "|"):
                    halves[g].append(i)
                    break
        order, how = order_points(pts, [a, b], halves)
        if how != "リストの順":
            return order, how
    return order_points(pts, None, None, paths)
