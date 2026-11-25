# -*- coding: utf-8 -*-
"""作り方「オーバーラップ」: overlap made from three animator values instead
of physical ones (pure Python, no Maya).

The controllers are solved by the same solvers as the simulation (chains:
chain_solver, single controllers: spring_solver), so collision, 横のつながり,
適用率 and the bake are the same.  Only the physical constants are made here:

* no gravity, every controller as heavy (mass 1);
* each joint is a spring towards its animated pose ``w^2`` and its masses are
  braked against their WORLD velocity ``D`` (an air drag; the simulation's
  damping works against the velocity relative to the animation and leaves
  no lag behind a steady motion).  A joint turning steadily then lags
  ``tau = D / w^2`` seconds behind, and settles with the damping ratio
  ``zeta = D / (2 w)``:  w = 2 zeta / tau,  D = 4 zeta^2 / tau;
* 遅れ (frames) is how late the TIP follows: the joints' lags add up along
  the chain, so it is shared between the joints - evenly (先ほど大きく 0) or
  more on the tip joints (100: the tip joint TIP_RATIO x the root joint);
* 戻りの強さ 0..100 is zeta ZETA_CALM .. ZETA_BOUNCY (0: it settles without
  overshooting; 100: it swings back and forth a few times).

A chain's joints carry the part below them, which moves faster the further
out it is: chain_constants() gives the chain one drag and sets each joint's
spring from the chain's rest pose so that every joint lags its share
(measured: the tip of chains of 2-8 lags 3.85-3.87 frames for 遅れ 4, of a
chain of 5 3.05 / 6.05 for 3 / 6; a single controller 0.9 / 2.9 / 5.9 for
1 / 3 / 6).
"""
from __future__ import annotations

import math

ZETA_CALM = 1.0          # 戻りの強さ 0: critically damped
ZETA_BOUNCY = 0.2        # 戻りの強さ 100 (0.15: past the pose by 1.8 x the lag after a stop)
# a chain: the joints damp each other - zeta x 1 / (1 + CHAIN_ZETA x
# min(n - 1, CHAIN_ZETA_N)), so a chain overshoots too (without it, chains
# of 5 / 8 did not overshoot at 戻りの強さ 30).  Capped: lowered with the
# whole length of a user's 10-joint hair strands, the joints rang wherever
# the collider pushed them (戻りの強さ 25: 140 turns of over 20 deg in a frame;
# capped at 2: none, at 4: none / 28 with 遅れ 8 戻りの強さ 60)
# 0.25 before JOINT_ZETA; that damps a chain's bounce too (chains of 2-8 at
# 戻りの強さ 30-60 hardly overshot), 1.0 gives it back (overshoot after a stop,
# 戻りの強さ 30 / 60: chain of 2 0.04 / 0.34, of 3 0.26 / 0.70, of 5 0.13 / 0.48,
# of 8 0.03 / 0.25 x the lag; the same 戻りの強さ now bounces a long chain
# as 4 x it did - the long hair preset: 戻りの強さ 10)
CHAIN_ZETA = 1.0
CHAIN_ZETA_N = 2
TIP_RATIO = 4.0         # 先ほど大きく 100: the tip joint's share of the lag / the root joint's
REACH_MAX = 8.0         # a chain joint far out from the root: its drag counted at most x this
MIN_LAG = 1.0e-3        # seconds (a joint with no lag share at all is still a stiff spring)
# a chain: each joint's swing against its parent is damped with this ratio
# (on top of the world drag).  The drag alone left the light tip joints of a
# soft chain free to whip about their parents: the user's long hair with 遅れ
# 48 / 戻りの強さ 30 turned its tips by over 20 deg in a frame 371 times (the
# simulation of the same hair: 7); with 1 (not the root joint, whose bounce
# is the chain's): 11 at 遅れ 60 and the same bounce, and the hair moves as far
# as the simulation (the tips 46 deg off the animation on average, the
# simulation 44; 0.3 / 0.5: 160 / 79 turns).  It does not slow a steady turn
# (the joints do not turn against each other then), only a joint swinging
# against its parent.
JOINT_ZETA = 1.0
MIN_FRAME_LAG = 1.0e-3   # frames: a joint lagging less follows the animation exactly
# the frame step: the joints below a chain's root keep at most 1 - this of
# their motion of the last frame.  Without it the joints of a user's 10-joint
# hair strands swung each other on (遅れ 20 / 戻りの強さ 30: turns of over 20
# deg in a frame 375 times, up to 46 deg, where the animation turns at most 4)
# 戻りの強さ 0 .. 100: FRAME_SWING_DAMP .. FRAME_SWING_DAMP_BOUNCY.  With the
# joints swinging about their pivots (INERTIA) one value did not do: 0.9 took
# the bounce out of chains (戻りの強さ 30: none), 0.5 gave it back but made the
# long hair at 戻りの強さ 0 flail (39 turns of over 20 deg in a frame, 0.9: 1).
FRAME_SWING_DAMP = 0.9
FRAME_SWING_DAMP_BOUNCY = 0.3
# the frame step: the joints below a chain's root swing about their own
# pivots (carried along by their parents) and stay behind by this part of
# their own pivot's move in the frame (their inertia).  Behind the chain
# root's move instead (all joints at once), the lag did not travel down the
# chain: the user's long hair bent as one C (「根元から末端にかけて遅れていって
# ほしい」「途中で S 字に」): the tip segments' turns followed the root
# segment's 7 frames late (the simulation 23; 遅れ 40: 6), bent both ways at
# once by over 10 deg in 10 % of the frames (the simulation 20 %).  Behind
# their own pivots: 24 frames, 51 %; tips 33 deg off the animation (0.6: 34 /
# 8 turns of over 20 deg in a frame, 0.8: 38 / 35).  All in the world, as
# スプリングマジック (1.0): the move of the pivots made each joint's lag, and
# the joints' own steps had to be shortened by their reach to keep the tip's
# lag - the user's long hair took its lagging shape far too quickly
# (「遅れたシルエットになるまでが速い」「クイックイ」: 遅れ 25 at 63 % after 8
# frames; its tips' turn speed changing twice the simulation's).  About the
# pivot only (0): the lag builds up in 17 frames but the hair barely moved as
# the body moved (tips 15 deg off the animation on average, the simulation
# 44).  0.5: 45 deg, 1 turn of over 20 deg in a frame (the simulation 7), the
# turn speed changing a little less than the simulation's (0.3: 34 deg, 0.6:
# 49 deg / 12 turns).
INERTIA = 0.5


def zeta(bounce):
    """戻りの強さ (0..100) -> damping ratio (square root: the overshoot shows
    from the low values on - 30 overshoots about 15 %, 50 about 25 %)."""
    b = min(100.0, max(0.0, float(bounce))) / 100.0
    return ZETA_CALM + (ZETA_BOUNCY - ZETA_CALM) * math.sqrt(b)


def shares(n, tip):
    """Share of the tip's lag of each of ``n`` joints (root first, sum 1).
    先ほど大きく ``tip`` 0..100: the weights grow linearly from 1 at the root
    to 1 + (TIP_RATIO - 1) * tip / 100 at the tip."""
    if n <= 1:
        return [1.0]
    top = 1.0 + (TIP_RATIO - 1.0) * min(100.0, max(0.0, float(tip))) / 100.0
    w = [1.0 + (top - 1.0) * i / (n - 1.0) for i in range(n)]
    s = sum(w)
    return [x / s for x in w]


def _discrete_zeta(s, d):
    """damping ratio of the frame step  e' = (1 - s) (e + (1 - d) (e - e_prev))
    (its poles as a continuous system's: -ln|z| / |ln z|)."""
    a = (1.0 - s) * (2.0 - d)
    b = (1.0 - s) * (1.0 - d)
    disc = a * a - 4.0 * b
    if disc >= 0.0:
        return 1.0 + math.sqrt(disc)        # real poles: critically / over-damped (grows with it)
    mod = math.sqrt(b)
    if mod <= 0.0:
        return 1.0
    ang = math.atan2(math.sqrt(-disc), a)
    lr = -math.log(mod)
    return lr / math.hypot(lr, ang)


def frame_constants(lag, zeta_):
    """(s, d) of one joint for the frame step (スプリングマジック型): every
    frame its tip keeps (1 - d) of its last frame's motion and is pulled the
    part s of the way to where the animation puts it.  Turning steadily it
    then lags  d (1 - s) / s  frames; s is searched so that the step settles
    with the damping ratio ``zeta_`` (as near as d <= 1 allows)."""
    lag = float(lag)
    if lag <= MIN_FRAME_LAG:
        return 1.0, 1.0
    best = None
    for k in range(600):
        s = 10.0 ** (-6.0 + 6.0 * k / 600.0)          # 1e-6 .. 1 (log steps: long lags need tiny s)
        d = lag * s / (1.0 - s)
        if d > 1.0:
            break
        err = abs(_discrete_zeta(s, d) - zeta_)
        if best is None or err < best[0]:
            best = (err, s, d)
    if best is None:                       # lag too short for any s: follow almost fully
        return 1.0 / (1.0 + lag), 1.0
    return best[1], best[2]


def chain_frame_constants(settings_list, reach=None):
    """(s, d) per joint for a chain (or one controller): 遅れ shared out by
    先ほど大きく, every joint settling with the 戻りの強さ's damping ratio.

    ``reach`` per joint (>= 1): how far its tip is from the chain's root, in
    its own lengths.  The chain turning about its root, the joint's tip is
    carried by its pivot too, and the step leaves it behind by its lag times
    that whole way: the joint's own lag is its share / reach (without it the
    tip of a chain of 5 lagged 2.9 x 遅れ)."""
    n = len(settings_list)
    s0 = settings_list[0]
    total = max(float(s0.overlap_delay), 0.0)
    z = zeta(s0.overlap_bounce)
    reach = list(reach) if reach else [1.0] * n
    b = min(100.0, max(0.0, float(s0.overlap_bounce))) / 100.0
    floor = FRAME_SWING_DAMP + (FRAME_SWING_DAMP_BOUNCY - FRAME_SWING_DAMP) * b
    out = []
    for i, (sh, rc) in enumerate(zip(shares(n, s0.overlap_tip), reach)):
        lag = total * sh / rc
        s, d = frame_constants(lag, z)
        if i > 0 and d < floor and lag > MIN_FRAME_LAG:
            # the joints below the root keep little of their motion (the root's
            # bounce is the chain's): the pull for the same lag
            d = floor
            s = d / (lag + d)
        out.append((s, d))
    return out


def bend_limits(settings_list):
    """曲がりの上限 per joint (radians, root first) or None (no limit): the
    chain's ``overlap_bend`` shared evenly by its joints.  (Tried: more of it
    on the tip joints, as 先ほど大きく - the tips of a user's long hair then
    whipped more: 122 turns of over 20 deg in a frame for 47.)"""
    n = len(settings_list)
    total = max(float(getattr(settings_list[0], "overlap_bend", 0.0) or 0.0), 0.0)
    if total <= 0.0 or n == 0:
        return None
    return [math.radians(total) / n] * n


BEND_KNEE = 0.5          # a joint bends freely up to this part of its limit, then ever more stiffly


def soft_limit(angle, limit):
    """``angle`` (>= 0) held under ``limit`` softly: unchanged up to
    BEND_KNEE x limit, then bending ever harder towards the limit (a tanh;
    smooth, no hard stop that would bounce the joint back)."""
    knee = BEND_KNEE * limit
    if angle <= knee:
        return angle
    room = limit - knee
    return knee + room * math.tanh((angle - knee) / room)


def joint_constants(settings_list, fps, reach=None):
    """(omega, drag) per joint (rad/s, 1/s) for a chain (or one controller)
    whose joints have the SimSettings ``settings_list``.  The chain's lag is
    its root joint's 遅れ / 先ほど大きく (a chain has one set of values).

    ``reach`` per joint (>= 1, default 1): when the whole chain turns about
    its root, the masses below joint i move faster than they would turning
    about joint i - by sum m (r_i . r_0) / sum m |r_i|^2 (r_i from joint i,
    r_0 from the root) - and the drag on them turns joint i that much
    further.  With it, the joint's equation is
        I th'' + D I th' + k th = D (reach I) W
    for the chain turning at W: lag tau = D reach I / k and damping ratio
    zeta = D I / (2 sqrt(k I))  ->  w = 2 zeta reach / tau,
    D = 4 zeta^2 reach / tau.  (Without it the tip of a chain of 5 lagged
    2.3 x 遅れ.)"""
    n = len(settings_list)
    s0 = settings_list[0]
    total = max(float(s0.overlap_delay), 0.0) / float(fps)
    reach = list(reach) if reach else [1.0] * n
    out = []
    for sh, s, rc in zip(shares(n, s0.overlap_tip), settings_list, reach):
        tau = max(total * sh, MIN_LAG)
        z = zeta(s.overlap_bounce)
        w = 2.0 * z * rc / tau
        out.append((w, 4.0 * z * z * rc / tau))
    return out


def chain_constants(settings_list, fps, masses, lever, lever0):
    """(drag per mass, joint spring k, joint inertia I) for a chain.

    ``lever[i][j]``: vector from joint i's pivot to mass j (j >= i);
    ``lever0[j]``: from the root's pivot to mass j (the rest pose).
    One drag D for the whole chain (it moves through one medium).  Turning
    the whole chain about its root at W, the masses below joint i move at
    |r_0j| W, so the drag turns the joint by about  D reach_i I_i W  (reach_i
    = sum m |r_ij| |r_0j| / sum m |r_ij|^2, kept to 1 .. REACH_MAX); the
    spring  k_i = D reach_i I_i / tau_i  makes it lag its share tau_i of 遅れ.
    Its own swing is braked by D I_i: damping ratio zeta_i = sqrt(D tau_i /
    reach_i) / 2.  D is set so that the median joint has the chain's zeta.
    (Tried: a drag per mass solved joint by joint for the exact lag and
    zeta - on a user's curved hair the drags of most joints came out 0 and
    the short, bent-back tip joints of a strand had almost no spring: the
    tip flailed, up to 179 deg in a frame.)"""
    n = len(settings_list)
    s0 = settings_list[0]
    total = max(float(s0.overlap_delay), 0.0) / float(fps)
    sh = shares(n, s0.overlap_tip)
    z = zeta(s0.overlap_bounce) / (1.0 + CHAIN_ZETA * min(n - 1, CHAIN_ZETA_N))
    inertia, reach, tau = [], [], []
    for i in range(n):
        I = sum(masses[j] * _dot(lever[i][j], lever[i][j]) for j in range(i, n))
        far = sum(masses[j] * math.sqrt(_dot(lever[i][j], lever[i][j]) * _dot(lever0[j], lever0[j]))
                  for j in range(i, n))
        inertia.append(max(I, 1.0e-12))
        reach.append(min(REACH_MAX, max(1.0, far / max(I, 1.0e-12))))
        tau.append(max(total * sh[i], MIN_LAG))
    ratio = sorted(rc / t for rc, t in zip(reach, tau))[n // 2]
    D = 4.0 * z * z * ratio
    k = [D * rc * I / t for rc, I, t in zip(reach, inertia, tau)]
    return [D] * n, k, inertia


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def apply(params, settings_list, fps, reach=None):
    """Put the overlap constants into the PhysicalParams ``params`` (one per
    joint; in place).  Returns the list of (omega, drag)."""
    consts = joint_constants(settings_list, fps, reach)
    for p, (w, d) in zip(params, consts):
        p.overlap = True
        p.mass = 1.0
        p.omega = w
        p.omega2 = w * w
        p.zeta = 0.0                     # no damping relative to the animation: the drag does it all
        p.follow = 0.0
        p.air_drag = d
        p.gravity = (0.0, 0.0, 0.0)
        p.gravity_mag = 0.0
        p.side = 0.0
        p.shape_frac = 0.0
        p.start_state = "animated"
        p.omega_t = p.omega_r = w
        p.c_t = p.c_r = 0.0
        # the drag brakes the swing only (the tip's velocity): the twist about
        # the bone's own axis is damped against the animation instead (one
        # controller; a chain has its own twist rate) - undamped, the twist of
        # an ear kept swinging and the loop never repeated itself (1.8 deg)
        p.twist_c = d                    # 2 zeta w: the same damping ratio as the swing
    return consts
