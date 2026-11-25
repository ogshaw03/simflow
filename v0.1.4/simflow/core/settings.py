# -*- coding: utf-8 -*-
"""Settings data classes (pure Python, no Maya import).

UI values are kept in the *normalised* 0-100 range described in the spec.
The conversion to physical quantities lives in
:func:`simflow.solvers.spring_solver.physical_params` so UI percentages are
never used directly as physical values.
"""
from __future__ import annotations

import copy
import zlib
from dataclasses import dataclass, field, asdict

from . import curve

AIM_AXES = {
    "+X": (1.0, 0.0, 0.0), "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0), "-Y": (0.0, -1.0, 0.0),
    "+Z": (0.0, 0.0, 1.0), "-Z": (0.0, 0.0, -1.0),
}

LABELS = {"strength": "適用率", "stiffness": "硬さ", "damping": "減衰",
          "mass": "重さ", "follow": "追従性", "air_drag": "空気抵抗", "variation": "ばらつき",
          "chain_link": "横のつながり", "rope_stretch": "伸び", "side_damp": "横揺れ抑え", "shape_keep": "形を保つ",
          "delay": "タイミングのずれ", "gravity": "重力", "swing_limit": "振れ幅の上限",
          "overlap_delay": "遅れ", "overlap_bounce": "戻りの強さ", "overlap_tip": "先ほど大きく",
          "overlap_bend": "曲がりの上限"}

# values a chain of a group can have of its own (on top of the group's: "チェーンごとの値")
CHAIN_FIELDS = ("stiffness", "damping", "mass", "follow", "air_drag", "chain_link", "side_damp", "shape_keep",
                "strength", "gravity", "delay", "swing_limit", "overlap_delay", "overlap_bounce", "overlap_tip", "overlap_bend")
OVERLAP_CHAIN_FIELDS = ("overlap_delay", "overlap_bounce", "overlap_tip", "overlap_bend", "chain_link", "strength")
SWING_LIMIT_MAX = 180.0          # degrees
DELAY_LIMIT = 30.0              # frames
OVERLAP_DELAY_MIN = 0.5         # frames (オーバーラップの「遅れ」)
OVERLAP_DELAY_MAX = 60.0         # frames (with 曲がりの上限, longer lags change little: the bend is held)
OVERLAP_BEND_MAX = 360.0        # degrees (曲がりの上限 of a whole chain)

# "ばらつき" 100 % = stiffness +-VARIATION_STIFFNESS / damping +-VARIATION_DAMPING (UI points)
VARIATION_STIFFNESS = 15.0
VARIATION_DAMPING = 10.0
VARIATION_GRAVITY = 0.25        # fraction (gravity-dominated presets barely react to stiffness)

TARGET_MODES = ("animation", "rest")
START_STATES = ("animated", "settled")
INITIAL_VELOCITIES = ("animation", "zero")
BAKE_MODES = ("additive", "override", "overwrite")
PREROLL_MODES = ("animation", "hold")

# fields a preset stores
PRESET_FIELDS = ("strength", "stiffness", "stiffness_curve", "damping", "mass", "follow", "air_drag", "gravity_enabled",
                 "gravity", "pendulum_length", "start_state", "rope_spin",
                 "overlap", "overlap_delay", "overlap_bounce", "overlap_tip", "overlap_bend")
DEFAULT_GRAVITY_DIRECTION = (0.0, -1.0, 0.0)


def _parse_frames(text):
    from .warp import parse_frames
    return parse_frames(text)


def _reflect(v):
    """Keep a varied value inside 0-100 by reflecting at the limits (a clamp
    would give every chain that goes below 0 the same value, e.g. with the
    stiffness-0 "垂れる" preset)."""
    if v < 0.0:
        v = -v
    if v > 100.0:
        v = 200.0 - v
    return min(100.0, max(0.0, v))


@dataclass
class SimSettings:
    """Simulation parameters (spec 4.1 / 4.2 / 2.2)."""
    solver: str = "spring"            # "spring" | "nhair" (Phase 3)
    strength: float = 100.0           # 0-100  blend original -> simulated
    stiffness: float = 50.0           # 0-100  (log-mapped to natural frequency)
    # chain only: multiplier of ``stiffness`` along the chain, ((x, y), ...)
    # x: 0 = root .. 1 = tip, y: 0..1 (see simflow.core.curve)
    stiffness_curve: tuple = ((0.0, 1.0), (1.0, 1.0))
    damping: float = 30.0             # 0-100  (damping ratio 0..1)
    mass: float = 50.0                # 0-100  (log-mapped 0.1 .. 10, 50 = 1.0)
    follow: float = 0.0               # 0-100  inherit fraction of target motion
    air_drag: float = 0.0             # 0-100  drag against the WORLD velocity (floaty, lingers at the top)
    # 0-100: each chain (or controller) gets its own stiffness / damping offset
    variation: float = 0.0
    variation_seed: int = 1           # change to get another (deterministic) pattern
    # 0-100 chain only: neighbouring chains of the group (sleeve / skirt rows)
    # do not move further apart than in the pose ("横のつながり", cloth)
    chain_link: float = 0.0
    # 0-100: remove the sideways swing (world horizontal, across the bone);
    # the bone still swings up / down in its vertical plane ("横揺れ抑え")
    side_damp: float = 0.0
    # 横揺れ抑え measured from straight down below the root instead of the
    # animated pose (kimono sleeves: drop straight down)
    side_down: bool = False
    # 0-100 chain only: the joints BETWEEN controllers are stiffer (bend and
    # twist kept as animated: curls, a ponytail that goes back and then hangs
    # down) while the chain still swings softly at its root ("形を保つ")
    shape_keep: float = 0.0
    # chain only, degrees (0 = none): no segment turns further than this from
    # its animated world orientation (the animation carried by the animated
    # parent); a soft wall catches it and takes the whip's energy
    # (a fast head turn must not fold a soft ponytail back on itself) ("振れ幅の上限")
    swing_limit: float = 0.0
    # chain only: the swing (the result relative to the animated pose) comes
    # this many frames later (negative = earlier); the animation itself is
    # not shifted ("タイミングのずれ")
    delay: float = 0.0
    # 0-100 rope only ("縄・ひも"): how easily the rope stretches (0 = only
    # when the hands pull it longer than it is)
    rope_stretch: float = 0.0
    # rope only: the rope's length (internal units, cm).  0 = the length of
    # the start frame's shape (grip -> controllers -> grip)
    rope_length: float = 0.0
    # rope only: the rope turns with the circles of the hands (jump rope whose
    # rope has no keys); 硬さ pulls it round ("手の回転に合わせて回す")
    rope_spin: bool = False
    # rope only: the rope does not pass through itself ("縄どうしの当たり")
    rope_self_collision: bool = True
    collision: bool = True            # collide with the scene's SIMFLOW colliders
    collision_thickness: float = 0.5  # internal units (cm): stop this far outside a collider
    # chain only, with 横のつながり: the cloth between neighbouring chains is also
    # pushed out of the scene's spheres / capsules ("布の当たり").  Needs ``collision``.
    cloth_collision: bool = True
    # chain only: a segment inside a sphere / capsule comes out on the side its
    # animation is on (seen last outside it), not the nearest way: long hair
    # stays behind the body, a fringe in front ("アニメの側へ逃がす").  Needs ``collision``.
    collision_side: bool = False
    # 作り方「オーバーラップ」: no gravity, no physical values to set - the
    # controllers follow the animation late (``overlap_delay`` frames at the
    # tip), overshoot by ``overlap_bounce`` and lag more towards the tip by
    # ``overlap_tip`` (simflow.solvers.overlap).  Collision, 横のつながり,
    # 適用率 and the bake work as in the simulation.
    overlap: bool = False
    overlap_delay: float = 3.0        # frames: how late the tip follows (遅れ)
    overlap_bounce: float = 30.0      # 0-100: 0 = settles without overshoot, 100 = bounces back and forth (戻りの強さ)
    overlap_tip: float = 50.0         # 0-100 chain only: 0 = every joint as late, 100 = the tip joints much later (先ほど大きく)
    # degrees: how far the whole chain may bend off the animation (曲がりの
    # 上限; shared evenly by its joints, reached softly; 0 = no limit).  Like
    # the Max angle of overlap tools and Dynamic Bone's Stiffness: a long lag
    # alone let the tips of a user's long hair flail up to 85 deg off the animation
    overlap_bend: float = 60.0
    # SwingGroup ids whose simulated bones push this group out (one way; they
    # are solved first).  Needs ``collision``.
    collide_with: tuple = ()
    gravity_enabled: bool = False
    gravity: float = 100.0            # 0-1000 (100 = 1 G = 9.80665 m/s^2)
    gravity_direction: tuple = (0.0, -1.0, 0.0)   # world space
    pendulum_length: float = 10.0     # internal units (cm)
    aim_axis: str = "+X"              # controller-local axis pointing to the tip
    translate: tuple = (False, False, False)
    rotate: tuple = (True, True, True)
    target_mode: str = "animation"    # spring target: animated pose or Rest Pose
    start_state: str = "animated"     # initial state: animated pose or settled
    initial_velocity: str = "animation"
    substep_rate: float = 240.0       # Hz, maximum internal step size
    # warp absorption (camera cheats, see simflow.core.warp)
    warp_enabled: bool = False
    warp_distance: float = 10.0       # internal units (cm): a jump this large in one block is a warp
    warp_angle: float = 25.0          # degrees
    warp_frames: int = 1              # a warp may take up to this many frames (1-3)
    warp_strength: float = 100.0      # 0-100 how much of the jump is removed
    warp_camera: str = ""             # camera for the on-screen check ("" = no check)
    warp_screen: float = 5.0          # % of the frame width: a jump more visible than this is not absorbed
    warp_add: str = ""                # frames added by hand ("37, 52 60-62")
    warp_ignore: str = ""             # frames never absorbed

    def to_dict(self):
        d = asdict(self)
        d["gravity_direction"] = list(self.gravity_direction)
        d["translate"] = list(self.translate)
        d["rotate"] = list(self.rotate)
        d["collide_with"] = list(self.collide_with)
        d["stiffness_curve"] = [list(p) for p in self.stiffness_curve]
        return d

    @classmethod
    def from_dict(cls, data):
        s = cls()
        if not data:
            return s
        for key, value in data.items():
            if not hasattr(s, key):
                continue
            if key in ("gravity_direction", "translate", "rotate", "collide_with"):
                value = tuple(value)
            elif key == "stiffness_curve":
                value = curve.normalise(value)
            setattr(s, key, value)
        return s

    def retire_removed(self):
        """The panel no longer offers 重力の方向, 戻る姿勢 (レストポーズ) and
        the per-axis choice of the moved channels (2026-11-14).  Values set
        with them in older scenes go back to what the panel can show: gravity
        straight down, the animated pose, all three axes of a channel (on
        when any axis was on).  Returns the names of what was changed."""
        changed = []
        if tuple(float(x) for x in self.gravity_direction) != DEFAULT_GRAVITY_DIRECTION:
            self.gravity_direction = DEFAULT_GRAVITY_DIRECTION
            changed.append("重力の方向")
        if self.target_mode != "animation":
            self.target_mode = "animation"
            changed.append("戻る姿勢（レストポーズ）")
        for name, label in (("rotate", "回転の軸"), ("translate", "移動の軸")):
            v = tuple(bool(x) for x in getattr(self, name))
            if any(v) and not all(v):
                setattr(self, name, (True, True, True))
                changed.append(label)
        return changed

    def chain_stiffness(self, index, count):
        """Stiffness (UI value 0-100) of joint ``index`` of a ``count`` long
        chain: ``stiffness`` x the stiffness curve at index / (count - 1)."""
        x = float(index) / (count - 1) if count > 1 else 0.0
        return self.stiffness * curve.evaluate(self.stiffness_curve, x)

    def copy(self):
        return copy.deepcopy(self)

    def effective(self):
        """The values the calculation uses.  オーバーラップ: the simulation's own
        values (kept in the group for switching back) are taken out - no
        gravity, nothing that holds the shape or limits the swing, the
        animated pose to start from (a copy; self for the simulation)."""
        if not self.overlap:
            return self
        s = self.copy()
        s.gravity_enabled = False
        s.start_state = "animated"
        s.follow = s.air_drag = s.variation = 0.0
        s.side_damp = s.shape_keep = s.swing_limit = s.delay = 0.0
        s.stiffness_curve = curve.FLAT
        return s

    def with_overrides(self, values):
        """Copy with the chain's own values (``CHAIN_FIELDS``) put over the
        group's (self when there are none).  オーバーラップ: only the values it
        uses (a chain's simulation values stay for switching back)."""
        fields = OVERLAP_CHAIN_FIELDS if self.overlap else CHAIN_FIELDS
        values = {k: v for k, v in (values or {}).items() if k in fields}
        if not values:
            return self
        s = self.copy()
        for k, v in values.items():
            setattr(s, k, float(v))
        return s

    def variation_offsets(self, key):
        """Deterministic (stiffness, damping) offsets in UI points and a
        gravity factor for the chain / controller ``key`` (its root
        controller's name)."""
        if self.variation <= 0.0:
            return 0.0, 0.0, 1.0
        v = min(self.variation, 100.0) / 100.0

        def u(tag):
            h = zlib.crc32(("%d:%s:%s" % (int(self.variation_seed), key, tag)).encode("utf-8"))
            return h / float(0xFFFFFFFF) * 2.0 - 1.0
        return (VARIATION_STIFFNESS * v * u("stiffness"), VARIATION_DAMPING * v * u("damping"),
                1.0 + VARIATION_GRAVITY * v * u("gravity"))

    def varied(self, key):
        """Copy with the variation of ``key`` applied (self when none)."""
        dk, dc, fg = self.variation_offsets(key)
        if dk == 0.0 and dc == 0.0 and fg == 1.0:
            return self
        s = self.copy()
        s.stiffness = _reflect(s.stiffness + dk)
        s.damping = _reflect(s.damping + dc)
        s.gravity = s.gravity * fg
        return s

    def validate(self):
        """Return a list of human readable problems (empty if valid)."""
        problems = []
        for name in ("strength", "stiffness", "damping", "mass", "follow", "air_drag", "variation", "chain_link", "rope_stretch", "side_damp",
                     "shape_keep", "overlap_bounce", "overlap_tip"):
            v = getattr(self, name)
            if not 0.0 <= v <= 100.0:
                problems.append("%s は 0〜100 の範囲にしてください（現在 %s）" % (LABELS.get(name, name), v))
        if not OVERLAP_DELAY_MIN <= self.overlap_delay <= OVERLAP_DELAY_MAX:
            problems.append("遅れは %g〜%g フレームにしてください（現在 %s）"
                            % (OVERLAP_DELAY_MIN, OVERLAP_DELAY_MAX, self.overlap_delay))
        if not 0.0 <= self.overlap_bend <= OVERLAP_BEND_MAX:
            problems.append("曲がりの上限は 0〜%g 度にしてください（0 = なし、現在 %s）" % (OVERLAP_BEND_MAX, self.overlap_bend))
        if abs(self.delay) > DELAY_LIMIT:
            problems.append("タイミングのずれは -%g〜%g フレームにしてください（現在 %s）" % (DELAY_LIMIT, DELAY_LIMIT, self.delay))
        if not 0.0 <= self.swing_limit <= SWING_LIMIT_MAX:
            problems.append("振れ幅の上限は 0〜%g 度にしてください（0 = なし、現在 %s）" % (SWING_LIMIT_MAX, self.swing_limit))
        if self.gravity < 0.0:
            problems.append("重力は 0 以上にしてください")
        if self.collision_thickness < 0.0:
            problems.append("当たりの太さは 0 以上にしてください")
        if self.rope_length < 0.0:
            problems.append("縄の長さは 0 以上にしてください（0 = 開始フレームの形から）")
        if self.pendulum_length < 0.0:
            problems.append("振り子の長さは 0 以上にしてください")
        if self.aim_axis not in AIM_AXES:
            problems.append("先端の軸の値が不正です: %r" % self.aim_axis)
        if self.target_mode not in TARGET_MODES:
            problems.append("戻る姿勢の値が不正です: %r" % self.target_mode)
        if self.start_state not in START_STATES:
            problems.append("初期状態の値が不正です: %r" % self.start_state)
        if self.initial_velocity not in INITIAL_VELOCITIES:
            problems.append("初速度の値が不正です: %r" % self.initial_velocity)
        if self.substep_rate < 24.0:
            problems.append("内部ステップレートは 24 Hz 以上にしてください")
        if self.warp_distance <= 0.0 or self.warp_angle <= 0.0:
            problems.append("ワープとみなす大きさは 0 より大きくしてください")
        if not 1 <= int(self.warp_frames) <= 3:
            problems.append("ワープのフレーム数は 1〜3 にしてください")
        if not 0.0 <= self.warp_strength <= 100.0:
            problems.append("吸収の強さは 0〜100 の範囲にしてください")
        if self.warp_screen < 0.0:
            problems.append("画面での大きさは 0 以上にしてください")
        for name, label in (("warp_add", "ワープにするフレーム"), ("warp_ignore", "ワープにしないフレーム")):
            try:
                _parse_frames(getattr(self, name))
            except ValueError:
                problems.append("%sの書き方が正しくありません: %r（例: 37, 52, 60-62）" % (label, getattr(self, name)))
        if not any(self.translate) and not any(self.rotate):
            problems.append("移動・回転のチャンネルが 1 つも有効になっていません")
        return problems


@dataclass
class BakeOptions:
    """Bake settings (spec 8.1)."""
    start: float = 1.0
    end: float = 120.0
    sample_step: float = 1.0
    mode: str = "additive"            # additive / override layer, or overwrite base
    layer_name: str = "SIMFLOW_Bake"
    preroll: int = 0                  # frames simulated before ``start`` (not baked)
    preroll_mode: str = "animation"   # pre-roll input: evaluate animation / hold start pose
    # the range is one cycle of a looping animation (walk, idle): the
    # calculation runs through earlier cycles first (in place of the
    # pre-roll), so the result's last frame leads into its first (ループ)
    loop: bool = False

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        b = cls()
        for key, value in (data or {}).items():
            if hasattr(b, key):
                setattr(b, key, value)
        return b

    def frames(self):
        """Sample frames for the range (always includes ``end``)."""
        if self.end < self.start:
            raise ValueError("End frame must be >= start frame")
        if self.sample_step <= 0:
            raise ValueError("Sample step must be > 0")
        out = []
        f = float(self.start)
        i = 0
        while f < self.end - 1.0e-6:
            out.append(f)
            i += 1
            f = float(self.start) + i * float(self.sample_step)
        out.append(float(self.end))
        return out
