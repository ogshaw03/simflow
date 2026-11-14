# -*- coding: utf-8 -*-
"""How far the bones of a swing group went into the bones / cloth panels of
the groups that push it out ("押し出す揺れもの"), measured on the FINAL result
(after Strength, per frame).  Pure Python.

When this finds nothing but the meshes still go through each other, the
bones are fine and the meshes are thicker than the 太さ (or the cloth goes
beyond the outer chains / below the last joint, where there is no panel).
"""
from __future__ import annotations

from ..solvers import collision as col
from ..utils import math_utils as mu

MIN_S = 0.05            # same as collision.bone_contact: the pivot is never pushed


def segment(result, k):
    """(pivot, tip) of a SimResult at frame index k."""
    fr = result.frame_results[k]
    d = mu.q_rotate(fr.quat, mu.v_normalize(result.tip_local, (1.0, 0.0, 0.0)))
    return fr.pivot, mu.v_add(fr.pivot, mu.v_scale(d, result.pendulum_length))


def depth_against(pivot, tip, c, thickness):
    """(depth, through) of segment pivot-tip in one bone / panel collider;
    depth <= 0 = outside."""
    reach = c[3] + thickness
    if len(c) > 5 and c[4] == "tri":
        a, b, cc = c[5]
        s, x, k = col.segment_triangle(pivot, tip, a, b, cc)
        if s < MIN_S:
            return 0.0, False
        dist = mu.v_length(mu.v_sub(x, k))
        if dist < 1.0e-9:
            # pierces the panel: how far the tip is on the other side + the reach
            n = mu.v_cross(mu.v_sub(b, a), mu.v_sub(cc, a))
            ln = mu.v_length(n)
            if ln < 1.0e-12:
                return 0.0, False
            n = mu.v_scale(n, 1.0 / ln)
            side = 1.0 if mu.v_dot(mu.v_sub(pivot, a), n) >= 0.0 else -1.0
            across = abs(min(0.0, side * mu.v_dot(mu.v_sub(tip, a), n)))
            return reach + across, True
        return reach - dist, False
    center, axis, half = c[0], c[1], c[2]
    a = mu.v_sub(center, mu.v_scale(axis, half))
    b = mu.v_add(center, mu.v_scale(axis, half))
    s, p1, p2 = col.closest_on_segments(pivot, tip, a, b)
    if s < MIN_S:
        return 0.0, False
    return reach - mu.v_length(mu.v_sub(p1, p2)), False


def pivot_inside(pivot, c, thickness):
    """The segment's pivot (never pushed) is itself within the reach."""
    reach = c[3] + thickness
    if len(c) > 5 and c[4] == "tri":
        k = col.closest_point_triangle(pivot, *c[5])
    else:
        k = col.core_point(c, pivot)
    return mu.v_length(mu.v_sub(pivot, k)) < reach


def check(entries, cols_per_frame, thickness, tolerance=None):
    """``entries``: [(label, SimResult)] of the pushed group, ``cols_per_frame``:
    per frame the bone / panel colliders it was pushed by.  Returns a dict:
    hits (list of (frame, label, depth, through, kind, pivot_inside)), worst,
    frames, root_frames (frames where every hit has its pivot inside)."""
    hits = []
    for label, r in entries:
        if r.pendulum_length <= 1.0e-6:
            continue
        for k, fr in enumerate(r.frame_results):
            if k >= len(cols_per_frame):
                break
            pivot, tip = segment(r, k)
            best = None
            for c in cols_per_frame[k]:
                if not col.is_bone(c):
                    continue
                reach = c[3] + thickness
                tol = tolerance if tolerance is not None else max(0.01, 0.03 * reach)
                d, through = depth_against(pivot, tip, c, thickness)
                if d > tol and (best is None or d > best[0]):
                    best = (d, through, "面" if len(c) > 5 and c[4] == "tri" else "骨",
                            pivot_inside(pivot, c, thickness))
            if best is not None:
                hits.append((fr.frame, label, best[0], best[1], best[2], best[3]))
    frames = sorted(set(h[0] for h in hits))
    worst = max(hits, key=lambda h: h[2]) if hits else None
    root_frames = sorted(f for f in frames if all(h[5] for h in hits if h[0] == f))
    return {"hits": hits, "frames": frames, "worst": worst, "root_frames": root_frames}


def ranges(frames):
    """[1, 2, 3, 7] -> "1-3、7"."""
    out = []
    start = prev = None
    for f in frames:
        if start is None:
            start = prev = f
        elif f - prev <= 1.0 + 1.0e-6:
            prev = f
        else:
            out.append((start, prev))
            start = prev = f
    if start is not None:
        out.append((start, prev))
    return "、".join("%g" % a if a == b else "%g〜%g" % (a, b) for a, b in out)


def causes(report, strength=100.0, rotate=(True, True, True)):
    """Likely causes of the hits, most likely first (Japanese sentences)."""
    if not report["hits"]:
        return []
    out = []
    root = report.get("root_frames") or []
    if root:
        out.append("骨の根元（押せない所）が当たりの中にあります：フレーム %s。根元を押す側の当たりから離すか、"
                   "押す側か、この揺れものの「太さ」を小さくしてください。" % ranges(root))
    if strength < 100.0 - 1.0e-6:
        out.append("適用率が %g です。計算では当たりの外に出していても、元のアニメーションと混ぜた分だけ戻って入ります"
                   "（適用率 100 で計算し直すと確かめられます）。" % strength)
    if not all(rotate):
        out.append("回転の軸の一部（%s）がオフです。その軸の回転は元のアニメーションのままなので、押し出した分が戻ります。"
                   % "・".join(a for a, on in zip("XYZ", rotate) if not on))
    rest = [f for f in report["frames"] if f not in set(root)]
    settings_hint = strength < 100.0 - 1.0e-6 or not all(rotate)
    if rest and not settings_hint:
        out.append("SIMFLOW が押し出しきれていません：フレーム %s（骨と面に挟まれた、など）。"
                   "このフレームと揺れもの名を開発者に伝えてください。" % ranges(rest))
    return out


def describe(report, unit=lambda cm: "%.2f cm" % cm, strength=100.0, rotate=(True, True, True)):
    if not report["hits"]:
        return "食い込みなし（骨は当たりの外にあります）"
    worst = report["worst"]
    f, label, depth, through, kind = worst[:5]
    frames = report["frames"]
    text = "食い込み：フレーム %s（%d フレーム）" % (ranges(frames), len(frames))
    text += "／最大 %s（フレーム %g、%s、%s%s）" % (unit(depth), f, label, kind, "を貫通" if through else "")
    reasons = causes(report, strength, rotate)
    if len(reasons) == 1:
        text += "\n原因の見当：" + reasons[0]
    elif reasons:
        text += "\n原因の見当：\n・" + "\n・".join(reasons)
    return text
