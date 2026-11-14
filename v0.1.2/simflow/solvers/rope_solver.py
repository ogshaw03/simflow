# -*- coding: utf-8 -*-
"""Rope solver ("縄・ひも（両端を持つ）"), e.g. a jump rope.

Pure Python, deterministic, no Maya dependency.

A rope rig (spline IK) is driven by a few controllers along the rope, side by
side (not parented), and its two ends (the grips) are constrained to the
hands.  The middle controllers are simulated as point masses joined by
distance constraints to their neighbours; the two ends are kinematic (they
follow the animated grips) and are not changed.

Model (position based, XPBD, per sub step ``h``)::

    v += h (g + w^2 (x_t - x) - c (v - v_t) - drag v)      goal spring = 硬さ
    p  = x + h v                                          (+ Follow carry)
    repeat: distance constraints |p_a - p_b| = L_ab (compliance = 伸び),
            push out of colliders / the floor
    v  = (p - x) / h,  x = p

``L_ab`` is the distance in the animated pose at the first frame.  When the
hands are further apart than the rope is long, every ``L_ab`` is lengthened
by the same factor (the rope stretches evenly, like the spline IK rig;
constraints that cannot all be met would stretch unevenly).  伸び 0
is (almost) inelastic, higher values stretch more under its own weight /
swinging.

Collision: when the rig's curve is known (``inputs[0].rope_curve``, a
rope_curve.RopeCurve: the curve that draws the rope, found and measured in
Maya), points along that CURVE are pushed out of the colliders / the floor,
moving the controllers that move them.  The controllers need not lie on the
curve and the curve does not go through them, so colliding the controller
points let the curve cut into the body between them and on bends.  Without
a curve the controller points themselves collide.  When a point touches a
collider its velocity into it is removed and the sliding velocity is damped
(friction), so the rope does not bounce off the floor.  縄どうしの当たり uses the
lines between the controllers either way.

Strength blends the position between the animation and the simulation.
"""
from __future__ import annotations

import math

from ..utils import math_utils as mu
from . import collision as col
from .base_solver import BaseSolver, Progress, SolverError, SolverResult
from .spring_solver import MIN_FREQUENCY_HZ, PhysicalParams
from ..core import curve as curve_mod
from . import rope_spin

STRETCH_HZ_100 = 3.9        # 伸び 100: the stretch mode of one segment (a hanging rope ~ +40 %)
FRICTION = 20.0             # 1/s: sliding on a collider / the floor slows down
MAX_SUBSTEPS = 2000         # per frame
MAX_MOVE = 0.25             # a point may move this fraction of its shortest segment per sub step
SELF_GAP = 0.3             # 縄どうしの当たり: kept distance between rope parts x the shortest segment
BEND_HZ = 20.0              # bend resistance (None = off): see _Stepper.step
BEND_FREE_DEG = 60.0        # every point may always bend this much (a rope with no keys, keyed straight, must still bend)


COS_FREE = math.cos(math.radians(BEND_FREE_DEG))


def stretch_compliance(stretch, mass):
    """伸び (0-100) -> XPBD compliance (1 / stiffness) of one segment.  0 =
    inelastic.  The sag stretch grows with the square of the value (a hanging
    rope: 25 ~ +3 %, 50 ~ +11 %, 100 ~ +40 %)."""
    if stretch <= 0.0:
        return 0.0
    f = STRETCH_HZ_100 * 100.0 / min(stretch, 100.0)
    return 1.0 / (mass * (2.0 * math.pi * f) ** 2)


class RopeSolver(BaseSolver):
    """``assign(inputs)``: SolverInputs along the rope, end -> end; the first
    and the last are the grips (kinematic, ``orig_pos`` only).  ``configure``
    takes one SimSettings.  ``get_result`` returns a SolverResult per MIDDLE
    controller (pos = simulated pivot, rot = the original orientation)."""

    name = "rope"

    def __init__(self):
        BaseSolver.__init__(self)
        self._inputs = None

    def assign(self, inputs):
        inputs = list(inputs)
        if len(inputs) < 3:
            raise ValueError("a rope needs two grips and at least one controller between them")
        n = len(inputs[0].frames)
        for inp in inputs:
            inp.validate()
            if len(inp.frames) != n:
                raise ValueError("rope inputs must share the same frames")
        self._inputs = inputs
        self._input = inputs[1]
        self._result = None

    def cleanup(self):
        BaseSolver.cleanup(self)
        self._inputs = None

    # ------------------------------------------------------------------
    def calculate(self, progress=None):
        if self._inputs is None or self._settings is None:
            raise RuntimeError("Solver needs assign() and configure() before calculate()")
        progress = progress or Progress()
        s = self._settings
        P = PhysicalParams(s)
        ins = self._inputs
        frames = ins[0].frames
        fps = ins[0].fps
        n_frames = len(frames)
        N = len(ins)                       # points incl. the two grips
        mids = range(1, N - 1)
        tgt = [list(inp.target_pos) for inp in ins]
        orig = [list(inp.orig_pos) for inp in ins]
        # the grips are where the hands put them (the animation), never the Rest Pose
        tgt[0], tgt[-1] = orig[0], orig[-1]

        rest = [mu.v_length(mu.v_sub(orig[i + 1][0], orig[i][0])) for i in range(N - 1)]
        if min(rest) < 1.0e-6:
            k = rest.index(min(rest))
            raise SolverError("縄の %s と %s が同じ位置にあるため、縄の長さを決められません。"
                              % (ins[k].name, ins[k + 1].name))
        start_length = sum(rest)
        if getattr(s, "rope_length", 0.0) > 1.0e-6:
            # the rope's real length (e.g. the spline IK), spread like the start shape
            k = s.rope_length / start_length
            rest = [x * k for x in rest]
        seg_min = min(rest)
        spin_info = {}
        if getattr(s, "rope_spin", False) and N > 2:
            # 手の回転に合わせて回す: the target rope turns with the hands
            spun, spin_info = rope_spin.spin_targets(orig[0], orig[-1], rest, fps, P.up)
            for k, i in enumerate(mids):
                tgt[i] = spun[k]
        # 硬さのカーブ: 0 = at a grip, 1 = the middle of the rope (along the
        # start shape), the same on both halves
        along = [0.0]
        for x_ in rest:
            along.append(along[-1] + x_)
        half = 0.5 * along[-1]
        point_P = [P]
        for i in range(1, N - 1):
            u = min(along[i], along[-1] - along[i]) / half
            si = s.copy()
            si.stiffness = s.stiffness * curve_mod.evaluate(s.stiffness_curve, u)
            point_P.append(PhysicalParams(si))
        point_P.append(P)
        omega_max = max(p_.omega for p_ in point_P[1:-1]) if N > 2 else P.omega
        c_max = max(p_.c_t for p_ in point_P[1:-1]) if N > 2 else P.c_t
        alpha = stretch_compliance(getattr(s, "rope_stretch", 0.0), P.mass)
        cols_frames = ins[0].colliders if P.collision else None
        thick = P.thickness
        rc = getattr(ins[0], "rope_curve", None) if cols_frames else None
        curve = rc.prepare(orig) if rc is not None and rc.n_cv > rc.degree else None

        def pos_at(arr, f, t):
            return mu.catmull_rom(arr[max(f - 1, 0)], arr[f], arr[f + 1], arr[min(f + 2, n_frames - 1)], t)

        def frame_inputs(f, t):
            if f >= n_frames - 1:
                f, t = n_frames - 2, 1.0
            pts = [pos_at(tgt[i], f, t) for i in range(N)]
            cols = col.lerp_colliders(cols_frames[f], cols_frames[f + 1], t, cols_frames[max(f - 1, 0)],
                                      cols_frames[min(f + 2, n_frames - 1)]) if cols_frames else None
            return pts, cols, (curve.base_at(f, t, n_frames) if curve is not None else None)

        # ---- state ---------------------------------------------------------
        x = [orig[i][0] for i in range(N)]
        dt0 = (frames[1] - frames[0]) / fps
        if s.initial_velocity == "animation":
            # the motion the rope follows (the target: without the drag of a
            # rotating grip, see SimulationManager._rope_targets); the Follow
            # part of the motion is carried, not stored as velocity
            v = [mu.v_scale(mu.v_sub(tgt[i][1], tgt[i][0]), (1.0 - P.follow) / dt0) for i in range(N)]
        else:
            v = [(0.0, 0.0, 0.0)] * N
        st = _State(x, v)
        stepper = _Stepper(N, rest, P, alpha, thick, point_P, curve)
        info = {"settled": None, "substeps": [], "segments": N - 2, "rest_length": sum(rest),
                "start_length": start_length,
                "stretch": [], "floor_hits": 0, "collision_hits": 0,
                "curve": (rc.name, curve.n) if curve is not None else None}
        info.update(spin_info)

        def n_sub(dt, f):
            n = max(1, int(math.ceil(dt * P.substep_rate - 1.0e-9)))
            w = max(omega_max, 2.0 * math.pi * MIN_FREQUENCY_HZ)
            n = max(n, int(math.ceil(dt * w / 0.35)), int(math.ceil(dt * (c_max + P.air_drag) / 0.9)))
            # the grips / the rope must not jump more than a fraction of a segment per sub step
            fastest = max(mu.v_length(x_) for x_ in st.v) * dt
            for i in (0, N - 1):
                fastest = max(fastest, mu.v_length(mu.v_sub(tgt[i][min(f + 1, n_frames - 1)], tgt[i][f])))
            n = max(n, int(math.ceil(fastest / (MAX_MOVE * seg_min))))
            return min(n, MAX_SUBSTEPS)

        if s.start_state == "settled":
            info["settled"] = stepper.settle(st, frame_inputs(0, 0.0), n_sub(1.0 / fps, 0) * fps)
        else:
            stepper.push_out(st, frame_inputs(0, 0.0), force=abs(sum(rest) - start_length) > 1.0e-6)

        out = [[st.x[i]] for i in range(N)]
        info["stretch"].append(_stretch(st.x, rest))
        prev = frame_inputs(0, 0.0)
        n_int = max(1, n_frames - 1)
        for f in range(n_frames - 1):
            progress.check(float(f) / n_int, "フレーム %g（%d / %d）" % (frames[f + 1], f + 1, n_int))
            dt = (frames[f + 1] - frames[f]) / fps
            n = n_sub(dt, f)
            h = dt / n
            for k in range(1, n + 1):
                if k % 400 == 0:
                    progress.check((f + float(k) / n) / n_int, "フレーム %g（%d / %d）激しい動きのため細かく計算中（%d / %d）"
                                   % (frames[f + 1], f + 1, n_int, k, n))
                cur = frame_inputs(f, float(k) / n)
                stepper.step(st, prev, cur, h)
                prev = cur
            for i in range(N):
                for c in st.x[i] + st.v[i]:
                    if not math.isfinite(c):
                        raise SolverError("フレーム %s で縄のシミュレーションが発散しました（%s）。"
                                          "伸びを下げるか、減衰または内部ステップレートを上げてください。"
                                          % (frames[f + 1], ins[i].name))
                out[i].append(st.x[i])
            info["substeps"].append(n)
            info["stretch"].append(_stretch(st.x, rest))
        progress.check(1.0, "計算が終わりました")
        info["floor_hits"] = stepper.floor_hits
        info["collision_hits"] = stepper.hits
        info["self_hits"] = stepper.self_hits

        results = []
        for i in mids:
            pos = [mu.v_lerp(orig[i][f], out[i][f], P.strength) for f in range(n_frames)]
            seg = dict(info)
            seg["index"] = i - 1
            results.append(SolverResult(frames=list(frames), pos=pos, rot=list(ins[i].orig_rot), info=seg))
        self._result = results
        return results


def _stretch(x, rest):
    """Rope length now / rest length."""
    return sum(mu.v_length(mu.v_sub(x[i + 1], x[i])) for i in range(len(rest))) / sum(rest)


class _State(object):
    def __init__(self, x, v):
        self.x = list(x)
        self.v = list(v)


class _Stepper(object):

    def __init__(self, N, rest, P, alpha, thickness, point_P=None, curve=None):
        self.N = N
        self.curve = curve                                # rope_curve.PreparedCurve or None
        point_P = point_P or [P] * N
        self.w2 = [p_.omega2 for p_ in point_P]          # spring per point (硬さのカーブ)
        self.omega = [p_.omega for p_ in point_P]
        self.rest = rest
        self.total = sum(rest)
        self.P = P
        self.alpha = alpha
        self.thick = thickness
        self.inv_m = 1.0 / P.mass
        self.iterations = max(8, 2 * N)
        self.hits = 0
        self.floor_hits = 0
        self.self_hits = 0
        self.c = [p_.c_t for p_ in point_P]               # damping per point
        # 縄どうしの当たり: the rope's own parts keep this distance (the rope's
        # thickness on both sides, at least SELF_GAP of a segment)
        self.self_on = bool(getattr(P, "rope_self", True))
        self.self_gap = max(2.0 * thickness, SELF_GAP * min(rest)) if rest else 0.0
        self.follow = P.follow

    def step(self, st, prev, cur, h):
        N = self.N
        P = self.P
        pts, cols, cbase = cur
        pts_prev = prev[0]
        x, v = st.x, st.v
        inv_h = 1.0 / h
        # grips: exactly where the hands are
        x[0], x[-1] = pts_prev[0], pts_prev[-1]
        p = list(x)
        p[0], p[-1] = pts[0], pts[-1]
        carry = [(0.0, 0.0, 0.0)] * N
        for i in range(1, N - 1):
            vt = mu.v_scale(mu.v_sub(pts[i], pts_prev[i]), inv_h)
            a = P.gravity
            if self.w2[i] > 0.0:
                a = mu.v_add(a, mu.v_scale(mu.v_sub(pts[i], x[i]), self.w2[i]))
            a = mu.v_sub(a, mu.v_scale(mu.v_sub(v[i], vt), self.c[i]))
            if P.air_drag > 0.0:
                a = mu.v_sub(a, mu.v_scale(v[i], P.air_drag))
            v[i] = mu.v_add(v[i], mu.v_scale(a, h))
            p[i] = mu.v_add(x[i], mu.v_scale(v[i], h))
            if self.follow > 0.0:
                # Follow: carried along with the animation (not part of the stored velocity)
                carry[i] = mu.v_scale(mu.v_sub(pts[i], pts_prev[i]), self.follow)
                p[i] = mu.v_add(p[i], carry[i])
        # distance constraints (XPBD), alternating direction, and collisions
        lam = [0.0] * (N - 1)
        span = mu.v_length(mu.v_sub(p[-1], p[0]))
        scale = max(1.0, span / self.total) if self.total > 0.0 else 1.0
        rest = self.rest if scale == 1.0 else [x * scale for x in self.rest]
        a_t = self.alpha * inv_h * inv_h
        lam_b = [0.0] * max(0, N - 2)
        bend_rest = []
        if BEND_HZ:
            for j in range(N - 2):
                u = mu.v_sub(pts[j + 1], pts[j])
                w = mu.v_sub(pts[j + 2], pts[j + 1])
                lu, lw = mu.v_length(u), mu.v_length(w)
                cos_bend = mu.v_dot(u, w) / (lu * lw) if lu > 1.0e-9 and lw > 1.0e-9 else 1.0
                cos_bend = min(cos_bend, COS_FREE)
                la, lb = rest[j], rest[j + 1]
                bend_rest.append(math.sqrt(max(0.0, la * la + lb * lb + 2.0 * la * lb * cos_bend)))
        ab_t = (1.0 / (self.P.mass * (2.0 * math.pi * BEND_HZ) ** 2)) * inv_h * inv_h if BEND_HZ else 0.0
        touched = [None] * N
        pairs = self._self_pairs(x, p) if self.self_on and N > 4 else []
        for it in range(self.iterations):
            order = range(N - 1) if it % 2 == 0 else range(N - 2, -1, -1)
            for j in order:
                wa = 0.0 if j == 0 else self.inv_m
                wb = 0.0 if j + 1 == N - 1 else self.inv_m
                if wa + wb <= 0.0:
                    continue
                d = mu.v_sub(p[j + 1], p[j])
                ln = mu.v_length(d)
                if ln < 1.0e-12:
                    continue
                C = ln - rest[j]
                dl = (-C - a_t * lam[j]) / (wa + wb + a_t)
                lam[j] += dl
                n = mu.v_scale(d, 1.0 / ln)
                if wa > 0.0:
                    p[j] = mu.v_sub(p[j], mu.v_scale(n, wa * dl))
                if wb > 0.0:
                    p[j + 1] = mu.v_add(p[j + 1], mu.v_scale(n, wb * dl))
            if BEND_HZ:
                # bending: a point and its next-but-one neighbour may not come
                # closer than the keyed BEND ANGLE allows with this rope's own
                # segment lengths (no sharper bend than the animation; without it
                # a rope stopped by the floor folds into a V).  The angle, not the
                # keyed positions: the keyed rope need not follow the hands.
                for j in range(N - 2):
                    a, b = j, j + 2
                    wa = 0.0 if a == 0 else self.inv_m
                    wb = 0.0 if b == N - 1 else self.inv_m
                    d = mu.v_sub(p[b], p[a])
                    ln = mu.v_length(d)
                    d0 = bend_rest[j]
                    if ln >= d0 or ln < 1.0e-12:
                        lam_b[j] = 0.0 if ln >= d0 else lam_b[j]
                        continue
                    C = ln - d0
                    dl = (-C - ab_t * lam_b[j]) / (wa + wb + ab_t)
                    lam_b[j] += dl
                    n = mu.v_scale(d, 1.0 / ln)
                    if wa > 0.0:
                        p[a] = mu.v_sub(p[a], mu.v_scale(n, wa * dl))
                    if wb > 0.0:
                        p[b] = mu.v_add(p[b], mu.v_scale(n, wb * dl))
            for pair in pairs:
                self._push_apart(p, pair)
            if cols and self.curve is not None:
                self._collide_curve(p, cols, cbase, touched)
            elif cols:
                for i in range(1, N - 1):
                    res = col.penetration(p[i], cols, self.thick)
                    if res is not None:
                        p[i] = res[0]
                        touched[i] = res[1]
        floor_now = False
        if cols and self.curve is not None and any(col.is_floor(c) for c in cols):
            floor_now = any(_on_floor(q, cols, self.thick) for q in self.curve.points(p, cbase))
        for i in range(1, N - 1):
            nv = mu.v_scale(mu.v_sub(mu.v_sub(p[i], x[i]), carry[i]), inv_h)
            n = touched[i]
            if n is not None:
                vn = mu.v_dot(nv, n)
                if vn < 0.0:
                    nv = mu.v_sub(nv, mu.v_scale(n, vn))
                # sliding on the surface slows down (relative to the target motion)
                vt = mu.v_scale(mu.v_sub(pts[i], pts_prev[i]), inv_h)
                rel = mu.v_sub(nv, vt)
                rel_t = mu.v_sub(rel, mu.v_scale(n, mu.v_dot(rel, n)))
                nv = mu.v_sub(nv, mu.v_scale(rel_t, 1.0 - math.exp(-FRICTION * h)))
                self.hits += 1
                if self.curve is not None:
                    if floor_now:
                        self.floor_hits += 1
                elif any(col.is_floor(c) for c in cols) and _on_floor(p[i], cols, self.thick):
                    self.floor_hits += 1
            v[i] = nv
            x[i] = p[i]
        x[0], x[-1] = pts[0], pts[-1]
        v[0] = mu.v_scale(mu.v_sub(pts[0], pts_prev[0]), inv_h)
        v[-1] = mu.v_scale(mu.v_sub(pts[-1], pts_prev[-1]), inv_h)

    def _collide_curve(self, p, cols, cbase, touched):
        """Push the points along the rig's curve out of the colliders, moving
        the controllers that move them (see rope_curve.PreparedCurve.push).
        A controller that moved most of a pushed sample is marked touched
        (its velocity into the collider is removed, friction)."""
        c = self.curve
        N = self.N
        inv_m = [0.0] + [self.inv_m] * (N - 2) + [0.0]
        for s in range(c.n):
            if not c.movable[s]:
                continue
            q = c.point(s, p, cbase)
            res = col.penetration(q, cols, self.thick)
            if res is None:
                continue
            target, n = res
            depth = mu.v_dot(mu.v_sub(target, q), n)
            if depth <= 0.0:
                continue
            moved = c.push(s, p, n, depth, inv_m)
            if not moved:
                continue
            top = max(share for _, share in moved)
            for i, share in moved:
                if share >= 0.5 * top:
                    touched[i] = n

    def _self_pairs(self, x, p):
        """Pairs of non-neighbouring segments that may touch in this step,
        with the direction that separated them at its start (x).  Pushing
        along that direction catches a pass-through inside one step too."""
        N = self.N
        reach = self.self_gap + max(mu.v_length(mu.v_sub(p[i], x[i])) for i in range(N)) * 2.0
        out = []
        for i in range(N - 1):
            for j in range(i + 2, N - 1):
                s_, t_, ci, cj = _closest(x[i], x[i + 1], x[j], x[j + 1])
                d = mu.v_sub(ci, cj)
                ln = mu.v_length(d)
                if ln > reach:
                    continue
                if ln < 1.0e-9:
                    d = mu.v_cross(mu.v_sub(x[i + 1], x[i]), mu.v_sub(x[j + 1], x[j]))
                    ln = mu.v_length(d)
                    if ln < 1.0e-12:
                        continue
                out.append((i, j, mu.v_scale(d, 1.0 / ln)))
        return out

    def _push_apart(self, p, pair):
        """Keep segment i at least self_gap from segment j along n."""
        i, j, n = pair
        N = self.N
        s_, t_, ci, cj = _closest(p[i], p[i + 1], p[j], p[j + 1])
        C = mu.v_dot(mu.v_sub(ci, cj), n) - self.self_gap
        if C >= 0.0:
            return
        w = [0.0 if k in (0, N - 1) else self.inv_m for k in (i, i + 1, j, j + 1)]
        b = (1.0 - s_, s_, 1.0 - t_, t_)
        W = sum(bb * bb * ww for bb, ww in zip(b, w))
        if W <= 0.0:
            return
        lam = -C / W
        for k, bb, ww, sign in zip((i, i + 1, j, j + 1), b, w, (1.0, 1.0, -1.0, -1.0)):
            if ww > 0.0:
                p[k] = mu.v_add(p[k], mu.v_scale(n, sign * bb * ww * lam))
        self.self_hits += 1

    def push_out(self, st, frozen, force=False):
        """Start outside the colliders / above the floor.  The points are moved
        together with the length and bend constraints (pushing only the points
        under the floor would leave a fold in the rope).  ``force``: also when
        nothing is inside (the rope's length differs from the start shape)."""
        pts, cols, cbase = frozen
        if self.curve is not None:
            inside = bool(cols) and any(col.penetration(q, cols, self.thick)
                                        for q in self.curve.points(st.x, cbase))
        else:
            inside = bool(cols) and any(col.penetration(st.x[i], cols, self.thick) for i in range(1, self.N - 1))
        if not inside and not force:
            return
        saved = (self.alpha, self.c, self.follow, self.P.gravity, self.w2)
        self.alpha, self.c, self.follow = 0.0, [0.0] * self.N, 0.0
        self.P.gravity, self.w2 = (0.0, 0.0, 0.0), [0.0] * self.N
        st.x[0], st.x[-1] = pts[0], pts[-1]
        v = list(st.v)
        st.v = [(0.0, 0.0, 0.0)] * self.N
        try:
            for _ in range(4):
                self.step(st, frozen, frozen, 1.0e-3)
                st.v = [(0.0, 0.0, 0.0)] * self.N
        finally:
            self.alpha, self.c, self.follow, self.P.gravity, self.w2 = saved
        # keep the animated start velocity, without the part into the colliders
        normals = [None] * self.N
        if self.curve is not None and cols:
            c = self.curve
            for s, q in enumerate(c.points(st.x, cbase)):
                res = col.penetration(q, cols, self.thick + 1.0e-3)
                if res is None:
                    continue
                best = max(c.W[s], key=lambda im: sum(abs(x) for x in im[1]), default=None)
                if best is not None:
                    normals[best[0]] = (None, res[1])
        for i in range(1, self.N - 1):
            if self.curve is not None:
                res = normals[i]
            else:
                res = col.penetration(st.x[i], cols, self.thick + 1.0e-3) if cols else None
            vi = v[i]
            if res is not None and mu.v_dot(vi, res[1]) < 0.0:
                vi = mu.v_sub(vi, mu.v_scale(res[1], mu.v_dot(vi, res[1])))
            st.v[i] = vi
        self.hits = self.floor_hits = self.self_hits = 0

    def settle(self, st, frozen, n_per_second, max_seconds=30.0):
        """Hang still from the grips (first frame), critically damped."""
        saved = (self.c, self.follow)
        P = self.P
        self.c = [2.0 * max(1.0, P.zeta) * max(w, 2.0 * math.pi * 1.0) for w in self.omega]
        self.follow = 0.0
        st.v = [(0.0, 0.0, 0.0)] * self.N
        n = max(60, int(n_per_second))
        h = 1.0 / n
        converged = False
        for k in range(int(max_seconds * n)):
            self.step(st, frozen, frozen, h)
            if k > n // 4 and max(mu.v_length(x_) for x_ in st.v) < 1.0e-3:
                converged = True
                break
        self.c, self.follow = saved
        st.v = [(0.0, 0.0, 0.0)] * self.N
        self.hits = self.floor_hits = self.self_hits = 0
        return converged


def _closest(p1, p2, q1, q2):
    """(s, t, point on p1-p2, point on q1-q2) closest between two segments."""
    d1, d2, r = mu.v_sub(p2, p1), mu.v_sub(q2, q1), mu.v_sub(p1, q1)
    a, e, f = mu.v_dot(d1, d1), mu.v_dot(d2, d2), mu.v_dot(d2, r)
    c, b = mu.v_dot(d1, r), mu.v_dot(d1, d2)
    den = a * e - b * b
    s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > 1.0e-12 else 0.0
    t = (b * s + f) / e if e > 1.0e-12 else 0.0
    if t < 0.0:
        t, s = 0.0, (min(1.0, max(0.0, -c / a)) if a > 1.0e-12 else 0.0)
    elif t > 1.0:
        t, s = 1.0, (min(1.0, max(0.0, (b - c) / a)) if a > 1.0e-12 else 0.0)
    return s, t, mu.v_add(p1, mu.v_scale(d1, s)), mu.v_add(q1, mu.v_scale(d2, t))


def _on_floor(p, cols, thick):
    for c in cols:
        if col.is_floor(c) and mu.v_dot(mu.v_sub(p, c[0]), c[1]) <= c[3] + thick + 1.0e-6:
            return True
    return False
