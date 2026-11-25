# -*- coding: utf-8 -*-
"""Offline Spring solver (spec 2.1 / 2.2 / 5.1).

Pure Python, deterministic, no Maya dependency.

Model
-----
Translation (world rotate-pivot position ``x``)::

    a = w^2 (x_t - x) - c (v - v_t) + g

Rotation (world orientation quaternion ``q``, angular velocity ``w``)::

    e     = log(q_t * q^-1)                  shortest-arc rotation vector
    alpha = w^2 e - c (w - w_t) + (r x (g - a_pivot)) / L^2

* ``w^2 = k / m``: Stiffness sets the natural frequency for mass 1, Mass
  scales inertia (heavier = slower, sags more under gravity).
* Damping is a damping *ratio* applied to the velocity *relative to the
  target* (a damper attached to the moving goal), so a constant-velocity
  motion produces no steady-state lag; only accelerations create overlap.
* Gravity (translation) is a world acceleration.  Gravity (rotation) is the
  torque of gravity on a virtual tip mass at ``Pendulum Length`` along the
  aim axis (see :mod:`pendulum_solver`).  The same term also applies the
  inertial pseudo-force of an accelerating pivot so a moving controller
  swings even when only rotation is simulated.
* Follow carries the state along with ``Follow %`` of the target motion each
  step (100 % = no inertia from animation, gravity still acts).  The stored
  velocities exclude the carried part, so damping acts against
  ``v - (1 - follow) * v_target``.
* Strength blends the final result from the original animation (0 %) to the
  full simulation (100 %).

Integration uses semi-implicit Euler with a fixed maximum internal step
(``substep_rate`` Hz, default 240 Hz) so results are nearly independent of
the scene frame rate.  The sub-step count is increased automatically when
stiffness / damping would make the step unstable.
"""
from __future__ import annotations

import math

from ..utils import math_utils as mu
from . import collision as col
from . import pendulum_solver as pend
from . import sideways
from .base_solver import BaseSolver, Progress, SolverError, SolverResult

GRAVITY_INTERNAL = 980.665          # 1 G in Maya internal units (cm / s^2)
MIN_FREQUENCY_HZ = 0.1
MAX_FREQUENCY_HZ = 10.0
AIR_DRAG_MAX = 80.0                 # 1/s at Air Drag 100 (quadratic: 50 -> 20 /s)


def air_drag_value(air_drag):
    """0..100 -> drag coefficient (1/s) against the world velocity."""
    return AIR_DRAG_MAX * (max(0.0, air_drag) / 100.0) ** 2


# ---------------------------------------------------------------------------
# UI value -> physical value mapping (spec 4.1: never use % directly)
# ---------------------------------------------------------------------------

def stiffness_to_frequency(stiffness):
    """0 -> 0 Hz (no spring), 0..100 -> 0.1 .. 10 Hz logarithmically
    (50 -> 1 Hz) for mass 1."""
    if stiffness <= 0.0:
        return 0.0
    return MIN_FREQUENCY_HZ * (MAX_FREQUENCY_HZ / MIN_FREQUENCY_HZ) ** (stiffness / 100.0)


def mass_value(mass):
    """0..100 -> 0.1 .. 10 logarithmically (50 -> 1.0)."""
    return 10.0 ** ((mass - 50.0) / 50.0)


class PhysicalParams(object):
    """Physical quantities derived from :class:`SimSettings`."""

    def __init__(self, settings):
        from ..core.settings import AIM_AXES
        self.overlap = False                     # set by overlap.apply
        self.mass = mass_value(settings.mass)
        freq = stiffness_to_frequency(settings.stiffness)
        k = (2.0 * math.pi * freq) ** 2          # spring constant for mass 1
        self.omega2 = k / self.mass
        self.omega = math.sqrt(self.omega2)
        self.zeta = settings.damping / 100.0
        self.follow = settings.follow / 100.0
        self.side = min(100.0, max(0.0, float(getattr(settings, "side_damp", 0.0)))) / 100.0
        # 形を保つ (chain): part of the gravity sag at the animated pose that
        # the joints hold up (the animated pose is the rest shape)
        self.rope_self = bool(getattr(settings, "rope_self_collision", True))
        self.side_down = bool(getattr(settings, "side_down", False))
        self.shape_frac = min(100.0, max(0.0, float(getattr(settings, "shape_keep", 0.0)))) / 100.0
        self.up = sideways.up_vector(settings.gravity_direction)
        # air drag (1/s) against the world velocity (floaty, lingers at the top)
        self.air_drag = air_drag_value(getattr(settings, "air_drag", 0.0))
        self.collision = bool(getattr(settings, "collision", True))
        self.thickness = max(0.0, float(getattr(settings, "collision_thickness", 0.5)))
        self.strength = settings.strength / 100.0
        if settings.gravity_enabled and settings.gravity > 0.0:
            gdir = mu.v_normalize(tuple(settings.gravity_direction), (0.0, -1.0, 0.0))
            self.gravity = mu.v_scale(gdir, GRAVITY_INTERNAL * settings.gravity / 100.0)
        else:
            self.gravity = (0.0, 0.0, 0.0)
        self.gravity_mag = mu.v_length(self.gravity)
        self.length = max(0.0, float(settings.pendulum_length))
        self.translate_on = any(settings.translate)
        self.rotate_on = any(settings.rotate)
        self.substep_rate = float(settings.substep_rate)
        self.start_state = settings.start_state
        self.initial_velocity = settings.initial_velocity
        self.aim = AIM_AXES[settings.aim_axis]

        # Effective natural frequencies, used for damping and stability.
        # Translation: spring only.  Rotation: spring + small-angle pendulum.
        self.omega_t = self.omega
        pend_w2 = self.gravity_mag / self.length if self.length > 1.0e-9 else 0.0
        self.omega_r = math.sqrt(self.omega2 + pend_w2)
        # Damping coefficients (relative velocity).  A floor frequency keeps
        # damping meaningful when stiffness is 0.
        floor = 2.0 * math.pi * MIN_FREQUENCY_HZ
        self.c_t = 2.0 * self.zeta * max(self.omega_t, floor)
        self.c_r = 2.0 * self.zeta * max(self.omega_r, floor)

    def substeps(self, dt, zeta_override=None):
        """Number of sub steps for a frame interval ``dt`` (seconds)."""
        c_max = max(self.c_t, self.c_r)
        if zeta_override is not None:
            floor = 2.0 * math.pi * MIN_FREQUENCY_HZ
            c_max = 2.0 * zeta_override * max(self.omega_t, self.omega_r, floor)
        c_max += self.air_drag
        w_max = max(self.omega_t, self.omega_r)
        n = max(1,
                int(math.ceil(dt * self.substep_rate - 1.0e-9)),
                int(math.ceil(dt * w_max / 0.35)),
                int(math.ceil(dt * c_max / 0.9)))
        return n


# ---------------------------------------------------------------------------
# Solver
# ---------------------------------------------------------------------------

class SpringSolver(BaseSolver):
    name = "spring"

    def calculate(self, progress=None):
        if self._input is None or self._settings is None:
            raise RuntimeError("Solver needs assign() and configure() before calculate()")
        progress = progress or Progress()
        inp = self._input
        p = PhysicalParams(self._settings)
        if getattr(self._settings, "overlap", False) and p.length > 1.0e-6:
            # オーバーラップ: the frame step of the chain solver, as a chain of one
            from .chain_solver import ChainSolver
            cs = ChainSolver()
            cs.initialize()
            cs.assign([inp])
            cs.configure([self._settings])
            self._result = cs.calculate(progress)[0]
            return self._result
        if getattr(self._settings, "overlap", False):
            # オーバーラップ: the spring and the drag from 遅れ / 戻りの強さ
            from . import overlap
            overlap.apply([p], [self._settings], inp.fps)

        n_frames = len(inp.frames)
        tgt_pos = list(inp.target_pos)
        orig_pos = list(inp.orig_pos)
        tgt_rot = _align_sequence(inp.target_rot)
        orig_rot = _align_sequence(inp.orig_rot)
        tip = mu.v_normalize(inp.tip_local, (1.0, 0.0, 0.0))
        follow = p.follow
        inertia = 1.0 - follow

        def pos_at(arr, i, t):
            return mu.catmull_rom(arr[max(i - 1, 0)], arr[i], arr[i + 1],
                                  arr[min(i + 2, n_frames - 1)], t)

        def rot_at(arr, i, t):
            return mu.q_slerp(arr[i], arr[i + 1], t)

        pivot_src = tgt_pos if p.translate_on else orig_pos
        col_frames = inp.colliders if p.collision else None
        bone_disp, bone_reach = col.bone_motion(col_frames, p.thickness)
        hits = 0
        side_on = p.side > 0.0 and p.rotate_on
        r_len = p.length if p.length > 1.0e-9 else 1.0

        # ---- initial state -------------------------------------------
        x = tgt_pos[0]
        q = tgt_rot[0]
        dt0 = (inp.frames[1] - inp.frames[0]) / inp.fps
        if p.initial_velocity == "animation":
            # velocities are stored *excluding* the part carried by Follow
            v = mu.v_scale(mu.v_sub(tgt_pos[1], tgt_pos[0]), inertia / dt0)
            w = mu.v_scale(mu.q_to_rotvec(mu.q_mul(tgt_rot[1], mu.q_conj(tgt_rot[0]))), inertia / dt0)
            vp = mu.v_scale(mu.v_sub(pivot_src[1], pivot_src[0]), 1.0 / dt0)
        else:
            v = (0.0, 0.0, 0.0)
            w = (0.0, 0.0, 0.0)
            vp = (0.0, 0.0, 0.0)

        info = {"settled": None, "substeps": [], "params": {
            "omega": p.omega, "mass": p.mass, "zeta": p.zeta,
            "gravity": p.gravity, "length": p.length}}

        if p.start_state == "settled":
            x, q, converged = self._settle(p, x, q, tgt_pos[0], tgt_rot[0], tip)
            info["settled"] = converged

        if col_frames and col_frames[0]:
            # めり込んで始まる: start outside the colliders (no jump on frame 2)
            c0 = col_frames[0]
            if p.translate_on:
                x, v, hit = col.push_point(x, v, c0, p.thickness)
                hits += int(hit)
            if p.rotate_on and p.length > 1.0e-9:
                piv0 = x if p.translate_on else orig_pos[0]
                q, w, hit = col.push_segment(q, w, piv0, tip, p.length, c0, p.thickness, iterations=4)
                hits += int(hit)
                info["start_inside"] = bool(hit)

        out_pos = [x]
        out_rot = [q]
        piv_prev = x if p.translate_on else orig_pos[0]

        # ---- time stepping -------------------------------------------
        for i in range(n_frames - 1):
            if i % 2 == 0:
                progress.check(float(i) / max(1, n_frames - 1), "フレーム %g（%d / %d）" % (
                    inp.frames[i + 1], i + 1, n_frames - 1))
            dt = (inp.frames[i + 1] - inp.frames[i]) / inp.fps
            n = p.substeps(dt)
            if bone_disp and i < len(bone_disp):
                # bones / panels of other swing groups must not jump over the segment
                n = min(4000, max(n, int(math.ceil(bone_disp[i] / (0.5 * bone_reach))),
                                  int(math.ceil(mu.v_length(w) * p.length * dt / (0.5 * bone_reach)))))
            info["substeps"].append(n)
            h = dt / n
            inv_h = 1.0 / h
            xt_prev = tgt_pos[i]
            qt_prev = tgt_rot[i]
            for s in range(1, n + 1):
                t = float(s) / n
                xt = pos_at(tgt_pos, i, t)
                qt = rot_at(tgt_rot, i, t)
                cols = col.lerp_colliders(col_frames[i], col_frames[i + 1], t, col_frames[max(i - 1, 0)],
                                          col_frames[min(i + 2, n_frames - 1)]) if col_frames else None

                # -- translation spring --
                if p.translate_on:
                    vt = mu.v_scale(mu.v_sub(xt, xt_prev), inv_h)
                    if follow > 0.0:
                        x = mu.v_add(x, mu.v_scale(mu.v_sub(xt, xt_prev), follow))
                    a = mu.v_add(
                        mu.v_sub(mu.v_scale(mu.v_sub(xt, x), p.omega2),
                                 mu.v_scale(mu.v_sub(v, mu.v_scale(vt, inertia)), p.c_t)),
                        p.gravity)
                    if p.air_drag > 0.0:
                        # world velocity = inertial part + part carried by Follow
                        a = mu.v_sub(a, mu.v_scale(mu.v_add(v, mu.v_scale(vt, follow)), p.air_drag))
                    v = mu.v_add(v, mu.v_scale(a, h))
                    x = mu.v_add(x, mu.v_scale(v, h))
                    if cols:
                        x, v, hit = col.push_point(x, v, cols, p.thickness)
                        hits += int(hit)
                    piv = x
                else:
                    piv = pos_at(orig_pos, i, t)

                # pivot acceleration (for pendulum inertia)
                vp_new = mu.v_scale(mu.v_sub(piv, piv_prev), inv_h)
                ap = mu.v_scale(mu.v_sub(vp_new, vp), inv_h)
                vp = vp_new
                piv_prev = piv

                # -- rotation spring + pendulum --
                if p.rotate_on:
                    dqt = mu.q_align(mu.q_mul(qt, mu.q_conj(qt_prev)), mu.Q_IDENTITY)
                    wt = mu.v_scale(mu.q_to_rotvec(dqt), inv_h)
                    if follow > 0.0:
                        q = mu.q_normalize(mu.q_mul(mu.q_slerp(mu.Q_IDENTITY, dqt, follow), q))
                    e = mu.q_to_rotvec(mu.q_mul(qt, mu.q_conj(q)))
                    alpha = mu.v_sub(mu.v_scale(e, p.omega2), mu.v_scale(mu.v_sub(w, mu.v_scale(wt, inertia)), p.c_r))
                    if p.length > 1.0e-9:
                        acc = pend.effective_tip_acceleration(p.gravity, ap, inertia)
                        if p.air_drag > 0.0:
                            # drag on the virtual tip mass against its WORLD velocity
                            r_tip = pend.tip_vector(q, tip, p.length)
                            w_world = mu.v_add(w, mu.v_scale(wt, follow))
                            v_tip = mu.v_add(vp, mu.v_cross(w_world, r_tip))
                            acc = mu.v_sub(acc, mu.v_scale(v_tip, p.air_drag))
                        alpha = mu.v_add(alpha, pend.angular_acceleration(q, tip, p.length, acc))
                    if p.overlap:
                        # オーバーラップ: the twist about the bone's axis (not braked by the drag)
                        ax = mu.q_rotate(q, tip)
                        tw = mu.v_dot(mu.v_sub(w, wt), ax)
                        alpha = mu.v_sub(alpha, mu.v_scale(ax, tw * p.twist_c))
                    w = mu.v_add(w, mu.v_scale(alpha, h))
                    q = mu.q_normalize(mu.q_mul(mu.q_from_rotvec(mu.v_scale(w, h)), q))
                    if side_on:
                        # 横揺れ抑え: remove part of the horizontal distance / speed
                        # from the animated pose of this moment
                        k = sideways.alpha(p.side, h)
                        r_sim = mu.v_scale(mu.q_rotate(q, tip), r_len)
                        if p.side_down:
                            r_anim = mu.v_scale(p.up, -r_len)       # straight down
                            wt_side = (0.0, 0.0, 0.0)
                        else:
                            r_anim = mu.v_scale(mu.q_rotate(qt, tip), r_len)
                            wt_side = wt
                        d = mu.v_sub(r_sim, r_anim)
                        d = mu.v_sub(d, mu.v_scale(p.up, mu.v_dot(d, p.up)))
                        v = mu.v_sub(mu.v_cross(mu.v_add(w, mu.v_scale(wt, follow)), r_sim), mu.v_cross(wt_side, r_anim))
                        v = mu.v_sub(v, mu.v_scale(p.up, mu.v_dot(v, p.up)))
                        r_new = mu.v_sub(r_sim, mu.v_scale(d, k))
                        ax = mu.v_cross(r_sim, r_new)
                        sn = mu.v_length(ax)
                        if sn > 1.0e-12:
                            ang = math.atan2(sn, mu.v_dot(r_sim, r_new))
                            q = mu.q_normalize(mu.q_mul(mu.q_from_rotvec(mu.v_scale(ax, ang / sn)), q))
                        w = mu.v_add(w, mu.v_scale(mu.v_cross(r_sim, mu.v_scale(v, -k)), 1.0 / (r_len * r_len)))
                    if cols and p.length > 1.0e-9:
                        q, w, hit = col.push_segment(q, w, piv, tip, p.length, cols, p.thickness)
                        hits += int(hit)

                xt_prev = xt
                qt_prev = qt

            _check_finite(x, q, inp.frames[i + 1], inp.name)
            out_pos.append(x)
            out_rot.append(q)

        progress.check(1.0, inp.name)
        info["collision_hits"] = hits

        # ---- strength blend & channel group selection -----------------
        res_pos, res_rot = [], []
        prev_q = None
        for i in range(n_frames):
            if p.translate_on:
                res_pos.append(mu.v_lerp(orig_pos[i], out_pos[i], p.strength))
            else:
                res_pos.append(orig_pos[i])
            if p.rotate_on:
                qo = mu.q_slerp(orig_rot[i], out_rot[i], p.strength)
            else:
                qo = orig_rot[i]
            if prev_q is not None:
                qo = mu.q_align(qo, prev_q)
            prev_q = qo
            res_rot.append(qo)

        self._result = SolverResult(frames=list(inp.frames), pos=res_pos, rot=res_rot, info=info)
        return self._result

    # ------------------------------------------------------------------
    def _settle(self, p, x, q, xt, qt, tip, max_seconds=30.0):
        """Relax the system under gravity with a static target (critically
        damped) so the simulation starts in equilibrium (sagged) instead of
        dropping during the first frames."""
        zeta = max(1.0, p.zeta)
        floor = 2.0 * math.pi * MIN_FREQUENCY_HZ
        c_t = 2.0 * zeta * max(p.omega_t, floor)
        c_r = 2.0 * zeta * max(p.omega_r, floor)
        n = p.substeps(1.0, zeta_override=zeta)
        h = 1.0 / n
        v = (0.0, 0.0, 0.0)
        w = (0.0, 0.0, 0.0)
        vp = (0.0, 0.0, 0.0)
        piv_prev = x
        steps = int(max_seconds * n)
        min_steps = int(0.25 * n)
        for step in range(steps):
            if p.translate_on:
                a = mu.v_add(mu.v_sub(mu.v_scale(mu.v_sub(xt, x), p.omega2),
                                      mu.v_scale(v, c_t)), p.gravity)
                v = mu.v_add(v, mu.v_scale(a, h))
                x = mu.v_add(x, mu.v_scale(v, h))
            vp_new = mu.v_scale(mu.v_sub(x, piv_prev), 1.0 / h) if p.translate_on else (0.0, 0.0, 0.0)
            ap = mu.v_scale(mu.v_sub(vp_new, vp), 1.0 / h)
            vp = vp_new
            piv_prev = x
            if p.rotate_on:
                e = mu.q_to_rotvec(mu.q_mul(qt, mu.q_conj(q)))
                alpha = mu.v_sub(mu.v_scale(e, p.omega2), mu.v_scale(w, c_r))
                if p.length > 1.0e-9:
                    acc = pend.effective_tip_acceleration(p.gravity, ap, 1.0)
                    alpha = mu.v_add(alpha, pend.angular_acceleration(q, tip, p.length, acc))
                w = mu.v_add(w, mu.v_scale(alpha, h))
                q = mu.q_normalize(mu.q_mul(mu.q_from_rotvec(mu.v_scale(w, h)), q))
            if step > min_steps and mu.v_length(v) < 1.0e-4 and mu.v_length(w) < 1.0e-6:
                return x, q, True
        return x, q, False


def _align_sequence(quats):
    out = []
    prev = None
    for q in quats:
        q = mu.q_normalize(tuple(q))
        if prev is not None:
            q = mu.q_align(q, prev)
        out.append(q)
        prev = q
    return out


def _check_finite(x, q, frame, name):
    for value in tuple(x) + tuple(q):
        if not math.isfinite(value):
            raise SolverError(
                "フレーム %s でシミュレーションが発散しました（%s）。"
                "硬さを下げるか、減衰または内部ステップレートを上げてください。" % (frame, name))
