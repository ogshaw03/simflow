# -*- coding: utf-8 -*-
"""Coupled chain solver (Chain Mode).

Pure Python, deterministic, no Maya dependency.

The chain is an articulated system of rigid segments solved *together* every
sub step, so a child pulls back on its parent (the old one-way chain let every
segment be driven by the previous one at the same natural frequency, which
amplified the motion towards the tip without bound).

Model
-----
Segment ``i`` (one controller) has a world orientation ``q_i`` and angular
velocity ``w_i``.  Its pivot is fixed in the parent segment::

    P_0 = animated pivot of the first controller (kinematic)
    P_i = P_{i-1} + q_{i-1} * o_i            o_i = animated offset in the parent frame

A point mass ``m_i`` sits at ``T_i = P_i + q_i * tip_i * L_i`` (``Pendulum
Length`` along the aim axis, or up to the next controller with auto length).
Mass velocities depend on all angular velocities up the chain::

    v_j = v_0 + sum_{i<j} w_i x (P_{i+1} - P_i) + w_j x (T_j - P_j)

which gives the mass matrix ``M(q)`` (3N x 3N).  Generalised forces:

* gravity and the inertial pseudo force of the accelerating root pivot on
  every mass, plus the centripetal terms;
* a joint spring ``tau = k_i * log(q_i^target * q_i^-1)`` and a joint damper
  ``-c_i (w_i - w_i^target)``, applied to segment ``i`` and with the opposite
  sign to its parent (action / reaction).  ``q_i^target = q_{i-1} * L_i``
  (animated local rotation on the *simulated* parent).

``k_i = I * omega_i^2`` where ``I`` is the inertia of the whole chain about
the root pivot (first frame).  Damping is a ratio per joint:
``c_i = 2 zeta_i w_i I_i`` with ``I_i`` the inertia of everything below joint
``i`` and ``w_i`` that joint's own frequency (spring + gravity of the part
below), so light tip joints are not over-damped.  With equal
settings every joint is equally stiff, like a uniform rod: the chain bends
most at the root and the tip follows smoothly, and Stiffness keeps its
meaning (natural frequency of the whole chain swinging rigidly about the
root).  A single segment chain is the single controller Spring model.  A
twist inertia ``m L^2`` about each segment's own axis keeps ``M`` regular.

Joint springs / dampers are integrated implicitly (linearised in the joint
velocity), so the stiff internal modes of short segments cannot blow up;
gravity, the pivot pseudo force and centripetal terms are explicit
(``M + h D^T C' D``) a = f is solved every sub step.  Twist about a segment
axis has no gravity / inertia torque, so it follows the animation through
the joint spring.

Strength blends each segment's *local* rotation (relative to its parent)
between the animation and the simulation, so 0 % on every segment returns the
original animation exactly.
"""
from __future__ import annotations

import math

from ..utils import math_utils as mu
from . import collision as col
from . import sideways
from .base_solver import BaseSolver, Progress, SolverError, SolverResult
from .spring_solver import MIN_FREQUENCY_HZ, PhysicalParams, _align_sequence

CHAIN_SAFETY = 2.0      # coupled modes can be faster than a single segment
MAX_TURN_PER_STEP = 0.2  # radians a segment may turn in one sub-step
SIDE_VELOCITY = 0.05     # 横揺れ抑え: part of each segment's own horizontal turning speed removed (x strength)
MAX_SUBSTEPS = 4000      # per frame (keeps a pathological input from hanging Maya)


def _pivot_acceleration(pos, f, dt):
    """Upper estimate of the root pivot acceleration between frame index f and
    f + 1 (Catmull-Rom interpolation of the sampled positions)."""
    n = len(pos)
    best = 0.0
    for k in (f, f + 1):
        if 1 <= k < n - 1:
            d2 = mu.v_add(mu.v_sub(pos[k + 1], mu.v_scale(pos[k], 2.0)), pos[k - 1])
            best = max(best, mu.v_length(d2))
    # Catmull-Rom can overshoot the finite difference (up to ~3x on a step)
    return 3.0 * best / (dt * dt)


LINK_RATE = 20.0         # "横のつながり" s (below 100): fraction s of the over-stretch is removed per 1/20 s
LINK_NEAR = 1.5          # chains whose roots are within 1.5 x the nearest root distance are neighbours


def find_links(roots):
    """Neighbouring chains from their root pivots: every chain is linked to
    its nearest chain and to the nearest one on the OTHER side, when that is
    about as near (a row of chains -> A-B, B-C; a skirt ring -> a closed
    ring; never across a chain in between).  Returns sorted (a, b) pairs."""
    pairs = set()
    n = len(roots)
    for i in range(n):
        d = sorted((mu.v_length(mu.v_sub(roots[i], roots[j])), j) for j in range(n) if j != i)
        if not d or d[0][0] < 1.0e-6:
            continue
        near, first = d[0]
        pairs.add((min(i, first), max(i, first)))
        u = mu.v_scale(mu.v_sub(roots[first], roots[i]), 1.0 / near)
        for dist, j in d[1:]:
            if dist > near * LINK_NEAR + 1.0e-9:
                break
            if mu.v_dot(u, mu.v_sub(roots[j], roots[i])) < 0.0:
                pairs.add((min(i, j), max(i, j)))
                break
    return sorted(pairs)


class ChainSolver(BaseSolver):
    """``assign`` takes a list of SolverInput (root first), ``configure`` a
    list of SimSettings (one per segment) and ``lengths`` (pendulum length
    per segment).  ``get_result`` returns a list of SolverResult.

    Several chains of one swing group (a sleeve / skirt) can be solved in the
    same sub steps with :meth:`assign_chains` / :meth:`configure_chains`;
    ``link`` (0-100, "横のつながり") then keeps the tips of the same joint of
    neighbouring chains from moving further apart than in the animated pose
    (cloth does not stretch, but it may bunch up).  ``get_result`` is then a
    list (per chain) of lists of SolverResult."""

    name = "chain"

    def __init__(self):
        BaseSolver.__init__(self)
        self._chains = None
        self._settings_chains = None
        self._link = 0.0
        self._multi = False

    def assign(self, inputs):
        self.assign_chains([inputs])
        self._multi = False

    def assign_chains(self, chains):
        if not chains:
            raise ValueError("ChainSolver needs at least one chain")
        n = len(chains[0][0].frames) if chains[0] else 0
        for inputs in chains:
            if len(inputs) < 1:
                raise ValueError("ChainSolver needs at least one segment")
            for inp in inputs:
                inp.validate()
                if len(inp.frames) != n:
                    raise ValueError("chain inputs must share the same frames")
        self._chains = [list(c) for c in chains]
        self._input = self._chains[0][0]
        self._result = None
        self._multi = True

    def configure(self, settings_list):
        self.configure_chains([settings_list])

    def configure_chains(self, settings_lists, link=0.0):
        if self._chains is not None:
            if len(settings_lists) != len(self._chains) or any(
                    len(s) != len(c) for s, c in zip(settings_lists, self._chains)):
                raise ValueError("one settings object per segment is required")
        self._settings_chains = [list(s) for s in settings_lists]
        self._settings = self._settings_chains[0][0]
        self._link = min(100.0, max(0.0, float(link))) / 100.0
        self._result = None

    def cleanup(self):
        BaseSolver.cleanup(self)
        self._chains = None
        self._settings_chains = None

    # ------------------------------------------------------------------
    def _prepare(self, ins, settings_list):
        """Per chain constants, interpolation and the initial state."""
        r = _Run()
        r.ins = ins
        r.settings_list = settings_list
        N = r.N = len(ins)
        n_frames = r.n_frames = len(ins[0].frames)
        P = r.P = [PhysicalParams(s) for s in settings_list]
        for i, p in enumerate(P):
            if p.length <= 1.0e-6:
                raise SolverError("チェーンでは振り子の長さを 0 より大きくしてください（%s）。" % ins[i].name)
        fps = ins[0].fps

        # ---- per frame input -----------------------------------------
        orig_rot = r.orig_rot = [_align_sequence(inp.orig_rot) for inp in ins]
        tgt_rot = r.tgt_rot = [_align_sequence(inp.target_rot) for inp in ins]
        orig_pos = r.orig_pos = [list(inp.orig_pos) for inp in ins]
        root_pos = r.root_pos = orig_pos[0]
        root_tgt = tgt_rot[0]
        # local target / original rotation and pivot offset of segment i in
        # the (animated) frame of segment i-1
        loc_tgt = [None] * N
        loc_orig = r.loc_orig = [None] * N
        offs = r.offs = [None] * N
        for i in range(1, N):
            lt, lo, of = [], [], []
            for f in range(n_frames):
                inv_parent = mu.q_conj(orig_rot[i - 1][f])
                lt.append(mu.q_mul(inv_parent, tgt_rot[i][f]))
                lo.append(mu.q_mul(inv_parent, orig_rot[i][f]))
                of.append(mu.q_rotate(inv_parent, mu.v_sub(orig_pos[i][f], orig_pos[i - 1][f])))
            loc_tgt[i] = _align_sequence(lt)
            loc_orig[i] = _align_sequence(lo)
            offs[i] = of
        tips = r.tips = [mu.v_normalize(inp.tip_local, (1.0, 0.0, 0.0)) for inp in ins]
        masses = [p.mass for p in P]
        lengths = r.lengths = [p.length for p in P]
        grav = [p.gravity for p in P]
        follow = [p.follow for p in P]
        inertia = [1.0 - x for x in follow]

        def pos_at(arr, f, t):
            return mu.catmull_rom(arr[max(f - 1, 0)], arr[f], arr[f + 1], arr[min(f + 2, n_frames - 1)], t)

        col_frames = ins[0].colliders if any(p.collision for p in P) else None

        def frame_inputs(f, t):
            """Interpolated kinematic input at frame index f + t."""
            if f >= n_frames - 1:
                f, t = n_frames - 2, 1.0
            p0 = pos_at(root_pos, f, t)
            qt0 = mu.q_slerp(root_tgt[f], root_tgt[f + 1], t)
            lt = [None] + [mu.q_slerp(loc_tgt[i][f], loc_tgt[i][f + 1], t) for i in range(1, N)]
            of = [None] + [mu.v_lerp(offs[i][f], offs[i][f + 1], t) for i in range(1, N)]
            cols = col.lerp_colliders(col_frames[f], col_frames[f + 1], t, col_frames[max(f - 1, 0)],
                                      col_frames[min(f + 2, n_frames - 1)]) if col_frames else None
            return p0, qt0, lt, of, cols
        r.frame_inputs = frame_inputs

        # ---- initial state ---------------------------------------------
        q = [None] * N
        p0, qt0, lt0, of0, cols0 = r.frozen = frame_inputs(0, 0.0)
        q[0] = qt0
        for i in range(1, N):
            q[i] = mu.q_mul(q[i - 1], lt0[i])
        # joint inertia about each pivot (first frame) -> spring / damper constants
        piv, tipp = _kinematics(q, p0, of0, tips, lengths)
        # every joint uses the inertia of the whole chain about the root, so
        # equal settings give a uniformly stiff rod (bends most at the root)
        chain_inertia = sum(masses[j] * _len2(mu.v_sub(tipp[j], piv[0])) for j in range(N))
        k_j = [chain_inertia * p.omega2 for p in P]
        # Damping ratio per joint: relative to that joint swinging everything
        # below it (its own spring + the gravity pendulum of the part below).
        # Using the whole chain's inertia here would over-damp the light tip
        # joints by orders of magnitude and lock the tip straight (a chain
        # could never become as soft as hair, even with Stiffness 0).
        sub_inertia = [sum(masses[j] * _len2(mu.v_sub(tipp[j], piv[i])) for j in range(i, N)) for i in range(N)]
        sub_lever = [sum(masses[j] * math.sqrt(_len2(mu.v_sub(tipp[j], piv[i]))) for j in range(i, N))
                     for i in range(N)]
        floor = 2.0 * math.pi * MIN_FREQUENCY_HZ
        c_j, damping_omega, settle_c = [], [], []
        for i, p in enumerate(P):
            inertia_i = sub_inertia[i]
            w_r = math.sqrt(k_j[i] / inertia_i + p.gravity_mag * sub_lever[i] / inertia_i)
            damping_omega.append(w_r)
            c_j.append(inertia_i * 2.0 * p.zeta * max(w_r, floor))
            # settling always uses (at least) critical damping, also with Damping 0
            settle_c.append(inertia_i * 2.0 * max(1.0, p.zeta) * max(w_r, floor))
        r.settle_c = settle_c

        dt0 = (ins[0].frames[1] - ins[0].frames[0]) / fps
        w = [(0.0, 0.0, 0.0)] * N
        p1, qt1, lt1, _of1, _cols1 = frame_inputs(1, 0.0)
        if settings_list[0].initial_velocity == "animation":
            vp = mu.v_scale(mu.v_sub(p1, p0), 1.0 / dt0)
        else:
            vp = (0.0, 0.0, 0.0)
        tgt_rate0 = [mu.v_scale(mu.q_to_rotvec(mu.q_align(mu.q_mul(qt1, mu.q_conj(qt0)), mu.Q_IDENTITY)), 1.0 / dt0)]
        for i in range(1, N):
            tgt_rate0.append(mu.v_scale(mu.q_to_rotvec(mu.q_align(mu.q_mul(lt1[i], mu.q_conj(lt0[i])), mu.Q_IDENTITY)), 1.0 / dt0))
        w_par = (0.0, 0.0, 0.0)
        for i in range(N):
            # world angular velocity of the animated chain, inertial part only
            if i == 0:
                total = tgt_rate0[0]
            else:
                total = mu.v_add(w_par, mu.q_rotate(q[i - 1], tgt_rate0[i]))
            w_par = total
            if settings_list[i].initial_velocity == "animation":
                w[i] = mu.v_scale(total, inertia[i])

        r.info = {"settled": None, "substeps": [], "segments": N, "damping_omega": damping_omega,
                  "joint_inertia": [k / P[i].omega2 if P[i].omega2 > 0 else None for i, k in enumerate(k_j)]}

        # Joint springs / dampers are implicit (unconditionally stable).  The
        # explicit gravity / pseudo force terms set the step: a light segment
        # carrying the rest of the chain swings at up to
        # sqrt((m_i + m_below) / m_i * g / L_i).
        w_grav = 0.0
        for i in range(N):
            m_tot = sum(masses[i:])
            g_i = max(P[j].gravity_mag for j in range(i, N))
            w_grav = max(w_grav, math.sqrt(m_tot / masses[i] * g_i / lengths[i]))

        # bones / panels of other swing groups: no point of them (and no tip
        # of this chain) may move more than half the reach in one sub step
        bone_disp, bone_reach = col.bone_motion(col_frames, min([p.thickness for p in P if p.collision] or [0.0]))
        r.bone_reach = bone_reach
        r.max_len = max(lengths)

        def n_sub(dt, zeta=None, f=None, w_now=None):
            n = max(1, int(math.ceil(dt * P[0].substep_rate - 1.0e-9)))
            if bone_disp and f is not None and f < len(bone_disp):
                n = max(n, int(math.ceil(bone_disp[f] / (0.5 * bone_reach))))
            n = max(n, int(math.ceil(dt * CHAIN_SAFETY * w_grav / 0.35)))
            if f is not None:
                # a violent pivot motion (e.g. moved a lot in ONE frame) swings the
                # chain like a stronger gravity: sqrt(a / L), plus keep the turn per
                # step small (explicit centripetal terms)
                a = _pivot_acceleration(root_pos, f, dt)
                n = max(n, int(math.ceil(dt * CHAIN_SAFETY * math.sqrt(a / min(lengths)) / 0.35)))
                if w_now:
                    n = max(n, int(math.ceil(dt * max(mu.v_length(x) for x in w_now) / MAX_TURN_PER_STEP)))
            return min(n, MAX_SUBSTEPS)
        r.n_sub = n_sub

        r.state = _State(q, w, vp, p0)
        r.sim = _Stepper(N, masses, lengths, tips, grav, inertia, follow, k_j, c_j, [p.air_drag for p in P],
                         [p.collision for p in P], [p.thickness for p in P], [p.shape_frac for p in P],
                         ins[0].hangs)
        return r

    def calculate(self, progress=None):
        if self._chains is None or self._settings_chains is None:
            raise RuntimeError("Solver needs assign() and configure() before calculate()")
        progress = progress or Progress()
        runs = [self._prepare(ins, sl) for ins, sl in zip(self._chains, self._settings_chains)]
        frames = runs[0].ins[0].frames
        fps = runs[0].ins[0].fps
        n_frames = len(frames)
        linker = None
        if self._link > 0.0 and len(runs) > 1:
            pairs = find_links([r.root_pos[0] for r in runs])
            if pairs:
                linker = _Linker(runs, pairs, self._link)

        # めり込んで始まる: start outside the colliders, keeping the shape
        # (pushing one segment alone bends the chain sharply and the joint
        # springs snap it back: the chain went wild on the first frames)
        inside = [r for r in runs if r.frozen[4] and
                  r.sim.start_outside(r.state, r.frozen[0], r.frozen[3], r.frozen[4])]
        for r in runs:
            r.info["start_inside"] = r in inside
        lean = [r for r in inside if all(s.start_state != "settled" for s in r.settings_list)]
        if lean:
            # then lean on the colliders: relax without gravity (the joint
            # springs rest against the colliders), keep the animated velocity
            w0 = [list(r.state.w) for r in lean]
            vp0 = [r.state.vp for r in lean]
            self._settle(lean, None, (), max_seconds=5.0, gravity=False)
            for r, w_, vp in zip(lean, w0, vp0):
                r.state.w, r.state.vp = w_, vp
        sides = [_Sideways(r) for r in runs]
        sides = [(k, sd) for k, sd in enumerate(sides) if sd.active]
        if any(s.start_state == "settled" for r in runs for s in r.settings_list):
            settled = self._settle(runs, linker, sides)
            for r in runs:
                r.info["settled"] = settled
        for r in runs:
            r.out_q = [[r.state.q[i]] for i in range(r.N)]

        # ---- time stepping ---------------------------------------------
        prev = [r.frozen for r in runs]
        n_int = max(1, n_frames - 1)
        for f in range(n_frames - 1):
            progress.check(float(f) / n_int, "フレーム %g（%d / %d）" % (frames[f + 1], f + 1, n_int))
            dt = (frames[f + 1] - frames[f]) / fps
            n = max(r.n_sub(dt, f=f, w_now=r.state.w) for r in runs)
            # The chain can speed up a lot INSIDE a frame (pivot moved a lot in
            # one frame): when a segment turns too far in one sub-step, redo the
            # frame with twice as many sub-steps.
            saved = [(list(r.state.q), list(r.state.w), r.state.vp, r.state.piv_prev, r.sim.hits,
                      [dict(d) for d in r.sim.sides]) for r in runs]
            saved_prev = list(prev)
            while True:
                h = dt / n
                diverged = too_fast = False
                for s in range(1, n + 1):
                    t = float(s) / n
                    if s % 200 == 0:
                        # a violent frame takes many sub steps: keep the dialog alive and honest
                        progress.check((f + t) / n_int, "フレーム %g（%d / %d）激しい動きのため細かく計算中（%d / %d）"
                                       % (frames[f + 1], f + 1, n_int, s, n))
                    cur = [r.frame_inputs(f, t) for r in runs]
                    try:
                        for r, pv, cu in zip(runs, prev, cur):
                            r.sim.step(r.state, pv, cu, h)
                        if linker is not None:
                            linker.apply(cur, f, t, h)
                        for k, sd in sides:
                            sd.apply(cur[k], f, t, h, dt)
                    except (ValueError, OverflowError, ZeroDivisionError):
                        diverged = True        # non-finite values reached math functions
                        break
                    prev = cur
                    turn = max(mu.v_length(x) for r in runs for x in r.state.w) * h
                    if not math.isfinite(turn):
                        diverged = True
                        break
                    if turn > 2.0 * MAX_TURN_PER_STEP:
                        too_fast = True
                        break
                    if any(r.bone_reach and max(mu.v_length(x) for x in r.state.w) * r.max_len * h > 0.5 * r.bone_reach
                           for r in runs):
                        too_fast = True        # a tip would jump over a thin bone / panel
                        break
                if not (diverged or too_fast) or n >= MAX_SUBSTEPS:
                    break
                for r, sv in zip(runs, saved):
                    r.state.q, r.state.w, r.state.vp, r.state.piv_prev, r.sim.hits = (
                        list(sv[0]), list(sv[1]), sv[2], sv[3], sv[4])
                    r.sim.sides = [dict(d) for d in sv[5]]
                prev = list(saved_prev)
                n = min(MAX_SUBSTEPS, n * 2)
            for r in runs:
                r.info["substeps"].append(n)
                for i in range(r.N):
                    for v in r.state.q[i] + r.state.w[i]:
                        if diverged or not math.isfinite(v):
                            raise SolverError(
                                "フレーム %s でチェーンのシミュレーションが発散しました（%s）。"
                                "硬さを下げるか、減衰または内部ステップレートを上げてください。\n"
                                "キャラを 1 フレームで大きく動かしている（カメラ合わせのワープ）場合は、"
                                "「カメラ合わせ（ワープ吸収）」をオンにしてください。"
                                % (frames[f + 1], r.ins[i].name))
                    r.out_q[i].append(r.state.q[i])
        progress.check(1.0, "計算が終わりました")
        for r in runs:
            r.info["collision_hits"] = r.sim.hits
            r.info["links"] = linker.links_of(runs.index(r)) if linker is not None else 0
            r.info["link_hits"] = linker.hits if linker is not None else 0
        for k, sd in sides:
            runs[k].info["side_worst"] = sd.worst

        results = [self._blend(r) for r in runs]
        self._result = results if self._multi else results[0]
        return self._result

    def _blend(self, r):
        """Strength blend (local, root first) -> SolverResults of one chain."""
        N, n_frames, P = r.N, r.n_frames, r.P
        out_q = r.out_q
        results = []
        out_rot = [[None] * n_frames for _ in range(N)]
        out_pos = [[None] * n_frames for _ in range(N)]
        for f in range(n_frames):
            for i in range(N):
                p = P[i]
                s_i = p.strength if p.rotate_on else 0.0
                if i == 0:
                    qo = mu.q_slerp(r.orig_rot[0][f], out_q[0][f], s_i)
                    po = r.root_pos[f]
                else:
                    sim_local = mu.q_mul(mu.q_conj(out_q[i - 1][f]), out_q[i][f])
                    sim_local = mu.q_align(sim_local, r.loc_orig[i][f])
                    qo = mu.q_mul(out_rot[i - 1][f], mu.q_slerp(r.loc_orig[i][f], sim_local, s_i))
                    po = mu.v_add(out_pos[i - 1][f], mu.q_rotate(out_rot[i - 1][f], r.offs[i][f]))
                if f > 0:
                    qo = mu.q_align(qo, out_rot[i][f - 1])
                out_rot[i][f] = mu.q_normalize(qo)
                out_pos[i][f] = po
        for i in range(N):
            seg_info = dict(r.info)
            seg_info["index"] = i
            results.append(SolverResult(frames=list(r.ins[i].frames), pos=out_pos[i], rot=out_rot[i], info=seg_info))
        return results

    # ------------------------------------------------------------------
    def _settle(self, runs, linker, sides=(), max_seconds=30.0, gravity=True):
        """Relax under gravity with a static target, (at least) critically
        damped per joint (all chains together when they are linked)."""
        zeta = max(1.0, max(p.zeta for r in runs for p in r.P))
        saved = []
        for r in runs:
            sim = r.sim
            saved.append((list(sim.c), list(sim.inertia), list(sim.follow), list(sim.g)))
            if not gravity:
                sim.g = [(0.0, 0.0, 0.0)] * sim.N
            sim.c = list(r.settle_c)
            sim.inertia = [1.0] * sim.N
            sim.follow = [0.0] * sim.N
            r.state.w = [(0.0, 0.0, 0.0)] * sim.N
            r.state.vp = (0.0, 0.0, 0.0)
            r.state.piv_prev = r.frozen[0]
        n = max(r.n_sub(1.0, zeta) for r in runs)
        h = 1.0 / n
        converged = False
        min_steps = int(0.25 * n)
        frozen = [r.frozen for r in runs]
        for step in range(int(max_seconds * n)):
            for r, fz in zip(runs, frozen):
                r.sim.step(r.state, fz, fz, h)
            if linker is not None:
                linker.apply(frozen, 0, 0.0, h)
            for k, sd in sides:
                # 横揺れ抑え also while settling (the first frame is the settled pose)
                sd.apply(frozen[k], 0, 0.0, h, None)
            if step > min_steps and max(mu.v_length(x) for r in runs for x in r.state.w) < 1.0e-6:
                converged = True
                break
        for r, sv in zip(runs, saved):
            r.sim.c, r.sim.inertia, r.sim.follow, r.sim.g = sv
            r.state.w = [(0.0, 0.0, 0.0)] * r.sim.N
            r.state.vp = (0.0, 0.0, 0.0)
            r.state.piv_prev = r.frozen[0]
        if linker is not None:
            linker.hits = 0
        return converged


class _Run(object):
    """One chain of a (possibly multi chain) solve."""


class _Sideways(object):
    """「横揺れ抑え」 of one chain (see simflow.solvers.sideways): the END of
    the chain may not move horizontally (left / right, front / back) away
    from the animated end of the same frame.  Judged at the end only: the
    segments in between may bend; the correction is shared by all segments
    (by how far each can move the end along the axis).  Done along two
    horizontal axes, one after the other."""

    def __init__(self, r):
        self.r = r
        self.s = r.P[0].side
        self.active = False
        if self.s <= 0.0 or not r.P[0].rotate_on:
            return
        N = r.N
        # the animated pivots below each segment and the animated end
        self.anim = [[r.orig_pos[j + 1][f] for j in range(N - 1)] + [_Linker._pose_tip(r, N - 1, f)]
                     for f in range(r.n_frames)]
        self.piv = [list(r.orig_pos[j]) for j in range(N)]
        scene = r.ins[0].scene_root
        if r.P[0].side_down:
            # 基準「真下」: the chain hanging straight down from its root pivot
            down = mu.v_scale(r.P[0].up, -1.0)
            cum = [0.0]
            for L in r.lengths:
                cum.append(cum[-1] + L)
            for f in range(r.n_frames):
                root = r.orig_pos[0][f]
                self.anim[f] = [mu.v_add(root, mu.v_scale(down, cum[j + 1])) for j in range(N)]
                for j in range(N):
                    self.piv[j][f] = mu.v_add(root, mu.v_scale(down, cum[j]))
            scene = None
        if scene:
            # carried by a simulated parent: back to the scene animation (the
            # whole chain is carried rigidly with its root)
            for f in range(r.n_frames):
                qd = mu.q_mul(scene[1][f], mu.q_conj(r.ins[0].orig_rot[f]))
                p0, s0 = r.ins[0].orig_pos[f], scene[0][f]

                def back(x):
                    return mu.v_add(s0, mu.q_rotate(qd, mu.v_sub(x, p0)))
                self.anim[f] = [back(x) for x in self.anim[f]]
                for j in range(N):
                    self.piv[j][f] = back(self.piv[j][f])
        self.axes = sideways.horizontal_axes(r.P[0].up)
        self.active = True
        self.worst = 0.0

    def apply(self, cur, f, t, h, dt):
        k = sideways.alpha(self.s, h)
        worst = 0.0
        for n in self.axes:
            worst += self._along(n, k, cur, f, t, dt) ** 2
        self.worst = max(self.worst, math.sqrt(worst))
        if cur[4]:
            self.r.sim.collide(self.r.state, cur[0], cur[3], cur[4])

    def _along(self, n, k, cur, f, t, dt):
        """Remove part of the end's deviation along the horizontal axis ``n``.
        Returns that deviation."""
        r = self.r
        f1 = min(f + 1, r.n_frames - 1)
        N = r.N
        piv, tipp = _kinematics(r.state.q, cur[0], cur[3], r.tips, r.lengths)
        pts = list(piv) + [tipp[N - 1]]
        segs = [mu.v_sub(pts[j + 1], pts[j]) for j in range(N)]
        # sideways deviation and relative velocity of the end
        a0, a1 = self.anim[f][N - 1], self.anim[f1][N - 1]
        dev = mu.v_dot(mu.v_sub(pts[N], mu.v_lerp(a0, a1, t)), n)
        # how far each segment can move the end sideways: its length across n
        lever = [math.sqrt(max(0.0, _len2(d) - mu.v_dot(d, n) ** 2)) for d in segs]
        left = sum(lever)
        want = k * dev             # sideways move of the end still to do
        for j in range(N):
            if lever[j] <= 1.0e-9 or left <= 1.0e-9:
                continue
            share = lever[j] / left
            left -= lever[j]
            p = pts[j + 1]
            # position only: the velocities are left alone.  Slowing every
            # segment's own horizontal turning also stopped the bending that
            # lets the end move up / down on its own (4 segments at 100: the
            # up / down movement went from 23 to 0.1 cm); removing the end's
            # velocity shared out diverged.  The position correction alone
            # takes the energy of the horizontal swing out every step.
            s0, s1 = self.piv[j][f], self.piv[j][f1]
            e0, e1 = self.anim[f][j], self.anim[f1][j]
            anim_rate = mu.v_dot(mu.v_sub(mu.v_sub(e1, s1), mu.v_sub(e0, s0)), n) / dt if dt else 0.0
            rate = mu.v_dot(mu.v_cross(r.state.w[j], segs[j]), n) - anim_rate
            rot = _turn_to(r, j, pts[j], p, mu.v_sub(p, mu.v_scale(n, want * share)), n,
                           -k * SIDE_VELOCITY * rate)
            if rot is not None:
                # the segments below keep their world orientation: they move
                # with the end of segment j
                moved = mu.v_sub(mu.v_add(pts[j], mu.q_rotate(rot, mu.v_sub(p, pts[j]))), p)
                want += mu.v_dot(moved, n)
                for m in range(j + 1, N + 1):
                    pts[m] = mu.v_add(pts[m], moved)
        return dev


class _Linker(object):
    """"横のつながり": the tip of joint j of a chain must not move further
    from the tip of joint j of a neighbouring chain than in the target
    (animated / rest) pose.  Position based: after every sub step the
    over-stretch is removed by turning the two segments about their pivots
    (root -> tip), and the speed at which the two tips move apart is removed
    from the angular velocities (inelastic: no speed towards each other is
    added, so a partly linked sleeve does not bounce inwards)."""

    def __init__(self, runs, pairs, strength):
        self.runs = runs
        self.pairs = pairs
        self.strength = strength
        self.hits = 0
        n_frames = runs[0].n_frames
        self.rest = []
        for a, b in pairs:
            ra, rb = runs[a], runs[b]
            levels = min(ra.N, rb.N)
            per = []
            for f in range(n_frames):
                per.append([mu.v_length(mu.v_sub(self._pose_tip(ra, j, f), self._pose_tip(rb, j, f)))
                            for j in range(levels)])
            self.rest.append(per)

    @staticmethod
    def _pose_tip(r, j, f):
        return mu.v_add(r.orig_pos[j][f], mu.v_scale(mu.q_rotate(r.tgt_rot[j][f], r.tips[j]), r.lengths[j]))

    def links_of(self, idx):
        return sum(1 for a, b in self.pairs if idx in (a, b))

    def apply(self, cur, f, t, h):
        if self.strength >= 1.0:
            alpha = 1.0
        else:
            alpha = 1.0 - (1.0 - self.strength) ** (h * LINK_RATE)
        n_frames = self.runs[0].n_frames
        f1 = min(f + 1, n_frames - 1)
        touched = set()
        for (a, b), rest in zip(self.pairs, self.rest):
            ra, rb = self.runs[a], self.runs[b]
            ca, cb = cur[a], cur[b]
            for j in range(len(rest[f])):
                d0 = rest[f][j] + (rest[f1][j] - rest[f][j]) * t
                piv_a, tip_a = _kinematics(ra.state.q, ca[0], ca[3], ra.tips, ra.lengths)
                piv_b, tip_b = _kinematics(rb.state.q, cb[0], cb[3], rb.tips, rb.lengths)
                d_vec = mu.v_sub(tip_b[j], tip_a[j])
                d = mu.v_length(d_vec)
                if d <= d0 + 1.0e-9:
                    continue
                self.hits += 1
                move = alpha * (d - d0) * 0.5
                dirv = mu.v_scale(d_vec, 1.0 / d)
                sep = mu.v_dot(mu.v_sub(_tip_velocity(rb, piv_b, tip_b, j), _tip_velocity(ra, piv_a, tip_a, j)), dirv)
                dv = alpha * sep * 0.5 if sep > 0.0 else 0.0
                _turn_to(ra, j, piv_a[j], tip_a[j], mu.v_add(tip_a[j], mu.v_scale(dirv, move)), dirv, dv)
                _turn_to(rb, j, piv_b[j], tip_b[j], mu.v_sub(tip_b[j], mu.v_scale(dirv, move)), dirv, -dv)
                touched.add(a)
                touched.add(b)
        # the correction must not push a segment into a collider
        for k in touched:
            c = cur[k]
            if c[4]:
                self.runs[k].sim.collide(self.runs[k].state, c[0], c[3], c[4])


def _tip_velocity(r, piv, tipp, j):
    """World velocity of the tip of segment j (inertial angular velocities)."""
    w = r.state.w
    v = r.state.vp
    for i in range(j):
        v = mu.v_add(v, mu.v_cross(w[i], mu.v_sub(piv[i + 1], piv[i])))
    return mu.v_add(v, mu.v_cross(w[j], mu.v_sub(tipp[j], piv[j])))


def _turn_to(r, j, pivot, tip, target, dirv, dv):
    """Turn segment j of run r about its pivot so its tip moves towards
    ``target``, and slow its tip by ``dv`` along ``dirv``."""
    st = r.state
    st.q = list(st.q)
    st.w = list(st.w)
    v0 = mu.v_sub(tip, pivot)
    v1 = mu.v_sub(target, pivot)
    axis = mu.v_cross(v0, v1)
    s = mu.v_length(axis)
    rot = None
    if s > 1.0e-12:
        angle = math.atan2(s, mu.v_dot(v0, v1))
        rot = mu.q_from_rotvec(mu.v_scale(axis, angle / s))
        st.q[j] = mu.q_normalize(mu.q_mul(rot, st.q[j]))
    l2 = _len2(v0)
    if dv != 0.0 and l2 > 1.0e-12:
        # tip velocity change w x r = dv * dirv (its part across the segment)
        st.w[j] = mu.v_add(st.w[j], mu.v_scale(mu.v_cross(v0, mu.v_scale(dirv, dv)), 1.0 / l2))
    return rot


# ---------------------------------------------------------------------------
class _State(object):
    def __init__(self, q, w, vp, p0):
        self.q = list(q)
        self.w = list(w)
        self.vp = vp
        self.piv_prev = p0


def _len2(v):
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]


def _kinematics(q, p0, offs, tips, lengths):
    piv = [p0]
    for i in range(1, len(q)):
        piv.append(mu.v_add(piv[i - 1], mu.q_rotate(q[i - 1], offs[i])))
    tipp = [mu.v_add(piv[i], mu.v_scale(mu.q_rotate(q[i], tips[i]), lengths[i])) for i in range(len(q))]
    return piv, tipp


def _block(a, b):
    """3x3 block ``(a.b) I - b a^T`` (row-major list of 9)."""
    d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    return [d - b[0] * a[0], -b[0] * a[1], -b[0] * a[2],
            -b[1] * a[0], d - b[1] * a[1], -b[1] * a[2],
            -b[2] * a[0], -b[2] * a[1], d - b[2] * a[2]]


def _solve_spd(A, b):
    """Cholesky solve of a symmetric positive definite system (lists)."""
    n = len(b)
    L = [[0.0] * n for _ in range(n)]
    for i in range(n):
        Li = L[i]
        Ai = A[i]
        for j in range(i + 1):
            Lj = L[j]
            s = Ai[j]
            for k in range(j):
                s -= Li[k] * Lj[k]
            if i == j:
                if s <= 0.0:
                    raise SolverError("チェーンの質量行列が正定値ではありません（長さか質量が 0 ですか？）。")
                Li[i] = math.sqrt(s)
            else:
                Li[j] = s / Lj[j]
    y = [0.0] * n
    for i in range(n):
        s = b[i]
        Li = L[i]
        for k in range(i):
            s -= Li[k] * y[k]
        y[i] = s / Li[i]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = y[i]
        for k in range(i + 1, n):
            s -= L[k][i] * x[k]
        x[i] = s / L[i][i]
    return x


class _Stepper(object):
    """One semi-implicit Euler sub step of the coupled chain."""

    def __init__(self, N, masses, lengths, tips, grav, inertia, follow, k, c, drag=None, collide=None,
                 thickness=None, shape=None, hangs=False):
        self.N = N
        self.hangs = hangs
        self.shape = list(shape) if shape else [0.0] * N     # 形を保つ (0..1) per joint
        self.collide_on = list(collide) if collide else [False] * N
        self.thickness = list(thickness) if thickness else [0.0] * N
        self.hits = 0
        self.sides = [dict() for _ in range(N)]     # remembered side of every panel (collision.push_segment)
        self.m = masses
        self.drag = list(drag) if drag else [0.0] * N     # air drag per mass (1/s)
        self.L = lengths
        self.tips = tips
        self.g = grav
        self.inertia = inertia
        self.follow = follow
        self.k = k
        self.c = c
        # mass of everything strictly below segment i
        self.m_below = [sum(masses[i + 1:]) for i in range(N)]

    def _weighted_matrix(self, weights, r, ol, twist):
        """sum_j w_j J_j^T J_j (3N x 3N) for point masses with weights
        ``weights`` (the mass matrix for weights = masses)."""
        N = self.N
        below = [sum(weights[i + 1:]) for i in range(N)]
        n3 = 3 * N
        A = [[0.0] * n3 for _ in range(n3)]
        for i in range(N):
            blk = [x * weights[i] for x in _block(r[i], r[i])]
            if i < N - 1 and below[i] > 0.0:
                b2 = _block(ol[i], ol[i])
                blk = [x + below[i] * y for x, y in zip(blk, b2)]
            if twist:
                d = mu.v_scale(r[i], 1.0 / self.L[i])
                tw = weights[i] * self.L[i] * self.L[i]
                for a_ in range(3):
                    for b_ in range(3):
                        blk[3 * a_ + b_] += tw * d[a_] * d[b_]
            _put(A, i, i, blk)
            for k in range(i + 1, N):
                blk = [x * weights[k] for x in _block(ol[i], r[k])]
                if k < N - 1 and below[k] > 0.0:
                    b2 = _block(ol[i], ol[k])
                    blk = [x + below[k] * y for x, y in zip(blk, b2)]
                _put(A, i, k, blk)
                _put(A, k, i, [blk[0], blk[3], blk[6], blk[1], blk[4], blk[7], blk[2], blk[5], blk[8]])
        return A

    def step(self, st, prev, cur, h):
        N = self.N
        inv_h = 1.0 / h
        p0, qt0, lt, of, cols = cur
        _pp0, qt0_prev, lt_prev, _of_prev, _cols_prev = prev
        q, w = st.q, st.w

        # root pivot acceleration (pseudo force)
        vp_new = mu.v_scale(mu.v_sub(p0, st.piv_prev), inv_h)
        ap = mu.v_scale(mu.v_sub(vp_new, st.vp), inv_h)
        st.vp = vp_new
        st.piv_prev = p0

        piv, tipp = _kinematics(q, p0, of, self.tips, self.L)
        r = [mu.v_sub(tipp[i], piv[i]) for i in range(N)]               # pivot -> own mass
        ol = [mu.v_sub(piv[i + 1], piv[i]) for i in range(N - 1)] + [None]  # pivot -> next pivot

        # target rotations / rates (child targets ride on the simulated parent)
        tgt_rate_local = [mu.v_scale(mu.q_to_rotvec(mu.q_align(mu.q_mul(qt0, mu.q_conj(qt0_prev)), mu.Q_IDENTITY)), inv_h)]
        for i in range(1, N):
            tgt_rate_local.append(mu.v_scale(mu.q_to_rotvec(mu.q_align(mu.q_mul(lt[i], mu.q_conj(lt_prev[i])), mu.Q_IDENTITY)), inv_h))
        qt = [qt0] + [mu.q_mul(q[i - 1], lt[i]) for i in range(1, N)]
        w_tot = []
        w_tgt = []
        for i in range(N):
            if i == 0:
                wt = tgt_rate_local[0]
            else:
                wt = mu.v_add(w_tot[i - 1], mu.q_rotate(q[i - 1], tgt_rate_local[i]))
            w_tgt.append(wt)
            w_tot.append(mu.v_add(w[i], mu.v_scale(wt, self.follow[i])))

        # joint torques, linearised implicit Euler in the joint velocity:
        #   tau' = k e - (c + h k) (D w' + phi)      D w = w_i - w_{i-1}
        # (phi = the rest of the relative velocity: target rate, Follow)
        f = [(0.0, 0.0, 0.0)] * N
        cimp = [self.c[i] + h * self.k[i] for i in range(N)]
        for i in range(N):
            e = mu.q_to_rotvec(mu.q_align(mu.q_mul(qt[i], mu.q_conj(q[i])), mu.Q_IDENTITY))
            v_rel = mu.v_sub(w_tot[i], w_tgt[i])
            # explicit part evaluated with the current velocity; the implicit
            # correction -h C' D a is moved to the left hand side below
            tau = mu.v_sub(mu.v_scale(e, self.k[i]), mu.v_scale(v_rel, cimp[i]))
            f[i] = mu.v_add(f[i], tau)
            if i > 0:
                f[i - 1] = mu.v_sub(f[i - 1], tau)
        if any(x > 0.0 for x in self.shape):
            self._hold_shape(f, r, ol)
        # point mass forces (gravity, pivot pseudo force, centripetal, air drag)
        cent_acc = (0.0, 0.0, 0.0)
        v_through = vp_new                        # world velocity of the current pivot
        has_drag = any(x > 0.0 for x in self.drag)
        F = []
        for j in range(N):
            cj = mu.v_add(cent_acc, mu.v_cross(w[j], mu.v_cross(w[j], r[j])))
            a = mu.v_sub(mu.v_sub(self.g[j], mu.v_scale(ap, self.inertia[0])), cj)
            if has_drag and self.drag[j] > 0.0:
                # air drag opposes the WORLD velocity of the mass (explicit
                # part; the implicit part is added to the matrix below)
                v_mass = mu.v_add(v_through, mu.v_cross(w_tot[j], r[j]))
                a = mu.v_sub(a, mu.v_scale(v_mass, self.drag[j]))
            F.append(mu.v_scale(a, self.m[j]))
            if j < N - 1:
                cent_acc = mu.v_add(cent_acc, mu.v_cross(w[j], mu.v_cross(w[j], ol[j])))
                v_through = mu.v_add(v_through, mu.v_cross(w_tot[j], ol[j]))
        F_below = (0.0, 0.0, 0.0)
        for i in range(N - 1, -1, -1):
            gi = mu.v_cross(r[i], F[i])
            if i < N - 1:
                gi = mu.v_add(gi, mu.v_cross(ol[i], F_below))
            f[i] = mu.v_add(f[i], gi)
            F_below = mu.v_add(F_below, F[i])

        # mass matrix (+ h * drag matrix: implicit air drag)
        A = self._weighted_matrix(self.m, r, ol, twist=True)
        if has_drag:
            Ad = self._weighted_matrix([m_ * c_ * h for m_, c_ in zip(self.m, self.drag)], r, ol, twist=False)
            for row, drow in zip(A, Ad):
                for jj in range(len(row)):
                    row[jj] += drow[jj]
        # implicit joint stiffness / damping: + h D^T C' D
        for i in range(N):
            ci = h * cimp[i]
            for a_ in range(3):
                A[3 * i + a_][3 * i + a_] += ci
                if i > 0:
                    A[3 * (i - 1) + a_][3 * (i - 1) + a_] += ci
                    A[3 * i + a_][3 * (i - 1) + a_] -= ci
                    A[3 * (i - 1) + a_][3 * i + a_] -= ci
        rhs = []
        for v in f:
            rhs.extend(v)
        acc = _solve_spd(A, rhs)

        # integrate (inertial part), then Follow carry root -> tip
        new_q = []
        for i in range(N):
            wi = (w[i][0] + acc[3 * i] * h, w[i][1] + acc[3 * i + 1] * h, w[i][2] + acc[3 * i + 2] * h)
            w[i] = wi
            new_q.append(mu.q_normalize(mu.q_mul(mu.q_from_rotvec(mu.v_scale(wi, h)), q[i])))
        if any(x > 0.0 for x in self.follow):
            for i in range(N):
                if self.follow[i] <= 0.0:
                    continue
                if i == 0:
                    old_t, new_t = qt0_prev, qt0
                else:
                    old_t = mu.q_mul(q[i - 1], lt_prev[i])
                    new_t = mu.q_mul(new_q[i - 1], lt[i])
                dqt = mu.q_align(mu.q_mul(new_t, mu.q_conj(old_t)), mu.Q_IDENTITY)
                new_q[i] = mu.q_normalize(mu.q_mul(mu.q_slerp(mu.Q_IDENTITY, dqt, self.follow[i]), new_q[i]))
        st.q = new_q
        if cols:
            self.collide(st, p0, of, cols)

    def _hold_shape(self, f, r, ol):
        """形を保つ: gravity does not bend the joints between controllers.
        Joint j (j >= 1, and the root joint of a chain that hangs from a
        simulated controller) holds up s of the gravity load of everything
        below it at the CURRENT pose, so the springs keep the animated shape
        of this moment (a tilted neck is part of it): a curl does not sag.
        The chain root still carries the whole weight (it hangs / swings and
        falls with the head as before); the inertia of the head's motion still
        bends the chain.
        (Holding the load of the ANIMATED pose made a curl held up level an
        upside-down pendulum: it flipped over (180 deg) when swung up.)"""
        N = self.N
        F_below = (0.0, 0.0, 0.0)
        load = (0.0, 0.0, 0.0)
        for i in range(N - 1, -1, -1):
            Fi = mu.v_scale(self.g[i], self.m[i])
            g_i = mu.v_cross(r[i], Fi)
            if i < N - 1:
                g_i = mu.v_add(g_i, mu.v_cross(ol[i], F_below))
            F_below = mu.v_add(F_below, Fi)
            load = mu.v_add(load, g_i)            # gravity torque of segments i .. N-1
            if i == 0 and not self.hangs:
                break
            s = self.shape[i]
            if s <= 0.0:
                continue
            tau = mu.v_scale(load, -s)
            f[i] = mu.v_add(f[i], tau)
            if i > 0:
                f[i - 1] = mu.v_sub(f[i - 1], tau)

    def start_outside(self, st, p0, of, cols, passes=4):
        """Push the start pose out of the colliders, root -> tip.  A pushed
        segment turns everything below it by the same rotation (the shape is
        kept), so no joint is bent by the push."""
        q, w = st.q, st.w
        pushed = False
        for _ in range(passes):
            moved = False
            pivot = p0
            for i in range(self.N):
                if i > 0:
                    pivot = mu.v_add(pivot, mu.q_rotate(q[i - 1], of[i]))
                if not self.collide_on[i]:
                    continue
                q_new, w[i], hit = col.push_segment(q[i], w[i], pivot, self.tips[i], self.L[i],
                                                    cols, self.thickness[i], iterations=4)
                if not hit:
                    continue
                d = mu.q_mul(q_new, mu.q_conj(q[i]))
                q[i] = q_new
                for j in range(i + 1, self.N):
                    q[j] = mu.q_normalize(mu.q_mul(d, q[j]))
                moved = True
            if not moved:
                break
            pushed = True
        self.sides = [dict() for _ in range(self.N)]
        return pushed

    def collide(self, st, p0, of, cols):
        """Push every segment tip out of the colliders, root -> tip (a pushed
        segment moves the pivots below it)."""
        q, w = st.q, st.w
        pivot = p0
        for i in range(self.N):
            if i > 0:
                pivot = mu.v_add(pivot, mu.q_rotate(q[i - 1], of[i]))
            if self.collide_on[i]:
                q[i], w[i], hit = col.push_segment(q[i], w[i], pivot, self.tips[i], self.L[i],
                                                   cols, self.thickness[i], sides=self.sides[i])
                self.hits += int(hit)


def _put(A, i, k, blk):
    r0, c0 = 3 * i, 3 * k
    for a in range(3):
        row = A[r0 + a]
        for b in range(3):
            row[c0 + b] = blk[3 * a + b]
