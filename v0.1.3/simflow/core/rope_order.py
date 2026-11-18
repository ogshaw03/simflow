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

These rules only give CANDIDATES (with the list order, its reverse and the
nearest-neighbour orders): :func:`choose_order` takes the one whose rope is
the shortest averaged over the range, and keeps the list order unless
another is clearly shorter.  (Rule 2 alone once scrambled a rig whose rope
controllers have keys of their own: the fit mixes the keys in.  The user
put the list right by hand and every calculation put it back.)

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


KEEP_LIST = 1.005          # the list order is kept unless another order is this much shorter
SCORE_FRAMES = 60


def mean_length(paths, order, frames=None):
    """Rope length along ``order``, averaged over the frames.  The real order
    is short at every frame (the rope keeps its length); a wrong one jumps
    across the rope, and a zig-zag that is short while the hands are close
    gets long when they move apart."""
    n_f = len(paths[0])
    frames = frames if frames is not None else range(n_f)
    tot = 0.0
    for f in frames:
        for i, j in zip(order, order[1:]):
            tot += mu.v_length(mu.v_sub(paths[i][f], paths[j][f]))
    return tot / max(1, len(frames))


def _nearest(paths, a, b, f):
    left = [i for i in range(len(paths)) if i not in (a, b)]
    out, cur = [a], a
    while left:
        nxt = min(left, key=lambda i: (mu.v_length(mu.v_sub(paths[i][f], paths[cur][f])), i))
        out.append(nxt)
        left.remove(nxt)
        cur = nxt
    return out + [b]


def _two_opt(paths, order, frames):
    """Reverse pieces between the grips while the rope gets shorter."""
    best = list(order)
    score = mean_length(paths, best, frames)
    improved = True
    rounds = 0
    while improved and rounds < 50:
        improved = False
        rounds += 1
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                s = mean_length(paths, cand, frames)
                if s < score - 1.0e-9:
                    best, score, improved = cand, s, True
    return best


def choose_order(paths, grips, candidates, list_order):
    """The order whose rope is the shortest over the range (see
    :func:`mean_length`), from ``candidates`` ((order, how) with the grips at
    the ends) improved by reversing pieces.  ``list_order`` (the user's list)
    is kept when no order is clearly shorter: an order the user put right by
    hand is never changed back."""
    n_f = len(paths[0])
    frames = list(range(n_f)) if n_f <= SCORE_FRAMES else \
        sorted(set(int(k * (n_f - 1) / float(SCORE_FRAMES - 1)) for k in range(SCORE_FRAMES)))
    a, b = grips
    cands = list(candidates)
    for f in (0, n_f // 2, n_f - 1):
        cands.append((_nearest(paths, a, b, f), "近い順"))
    scored = sorted(((mean_length(paths, o, frames), k, o, how) for k, (o, how) in enumerate(cands)))
    _, _, best, how = scored[0]
    refined = _two_opt(paths, best, frames)
    if refined != best:
        best, how = refined, "縄が最も短くなる順"
    keep = list_order is not None and list_order[0] in grips and list_order[-1] in grips
    if keep and mean_length(paths, list_order, frames) <= KEEP_LIST * mean_length(paths, best, frames):
        return list(list_order), "リストの順"
    return best, how


def rope_order(nodes, frames):
    """(order of ``nodes`` indices, how it was found).  The first and the
    last node are the grips unless the hierarchy says otherwise.  The rig
    (hierarchy, motion) gives candidates; the one whose rope stays the
    shortest over the range wins, and the list order is kept unless another
    is clearly shorter (see :func:`choose_order`)."""
    frames = list(frames)
    if len(frames) > 240:                       # enough to see how they move
        step = len(frames) / 240.0
        frames = [frames[int(k * step)] for k in range(240)] + [frames[-1]]
    paths = _positions(nodes, frames)
    pts = [p[0] for p in paths]
    n = len(nodes)
    cands = []
    grips = (0, n - 1)
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
            grips = (a, b)
            cands.append((order, how))
    order, how = order_points(pts, list(grips), None, paths)
    if how != "リストの順":
        cands.append((order, how))
    a, b = grips
    mids = [i for i in range(n) if i not in grips]
    list_order = [a] + mids + [b]
    cands.append((list_order, "リストの順"))
    cands.append(([a] + mids[::-1] + [b], "リストの逆順"))
    return choose_order(paths, grips, cands, list_order if grips == (0, n - 1) else None)
