# -*- coding: utf-8 -*-
"""Common solver interface (spec 10.1).

Every solver follows the same life cycle so the UI and the bake manager do not
need to know which solver produced a result::

    solver.initialize()
    solver.assign(solver_input)
    solver.configure(settings)
    solver.calculate(progress)
    result = solver.get_result()
    solver.reset() / solver.cleanup()

The data exchanged is Maya independent: per-frame world-space pivot positions
and world orientations (quaternions).  Converting to / from Maya transforms is
done by :mod:`simflow.core.transform_utils`.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


class SimulationCancelled(Exception):
    """Raised when the user cancels a running calculation."""


class SolverError(RuntimeError):
    """Raised when the simulation cannot produce a valid result (user facing)."""


class Progress(object):
    """Minimal progress / cancel interface.  The UI subclasses this."""

    def update(self, fraction, message=""):
        pass

    def cancelled(self):
        return False

    def check(self, fraction, message=""):
        self.update(fraction, message)
        if self.cancelled():
            raise SimulationCancelled("計算をキャンセルしました")


@dataclass
class SolverInput:
    """Sampled, Maya independent input for one controller.

    ``target_*`` is what the spring is pulled towards (the animated pose, or
    the Rest Pose re-parented to the animated parent).  ``orig_*`` is the
    original animated pose and is used for the Strength blend and as pivot
    when translation is not simulated.
    """
    fps: float
    frames: List[float]
    target_pos: List[Vec3]
    target_rot: List[Quat]
    orig_pos: List[Vec3]
    orig_rot: List[Quat]
    tip_local: Vec3 = (1.0, 0.0, 0.0)   # unit vector in the orientation frame
    name: str = ""
    # per frame: list of colliders (center, axis, half_length, radius), world
    colliders: Optional[list] = None
    # chain root of a chain carried by an already simulated parent (thin
    # bundles of a ponytail): (positions, rotations) per frame of the root in
    # the SCENE, before the carry.  横揺れ抑え judges the end against the scene
    # animation with it (else the thin bundles follow their parent's drift).
    scene_root: Optional[tuple] = None
    # chain root only: the chain hangs from an already simulated controller
    # (thin bundles of a ponytail), so its root joint is a joint between
    # controllers too (形を保つ holds it as well)
    hangs: bool = False

    def validate(self):
        n = len(self.frames)
        if n < 2:
            raise ValueError("At least 2 frames are required to simulate")
        for attr in ("target_pos", "target_rot", "orig_pos", "orig_rot"):
            if len(getattr(self, attr)) != n:
                raise ValueError("SolverInput.%s length mismatch" % attr)
        if self.fps <= 0:
            raise ValueError("Invalid frame rate")


@dataclass
class SolverResult:
    frames: List[float]
    pos: List[Vec3]
    rot: List[Quat]
    info: dict = field(default_factory=dict)


class BaseSolver(object):
    """Abstract solver."""

    name = "base"

    def __init__(self):
        self._input: Optional[SolverInput] = None
        self._settings = None
        self._result: Optional[SolverResult] = None

    # life cycle ---------------------------------------------------------
    def initialize(self):
        self.reset()

    def assign(self, solver_input):
        solver_input.validate()
        self._input = solver_input
        self._result = None

    def configure(self, settings):
        self._settings = settings
        self._result = None

    def calculate(self, progress=None):
        raise NotImplementedError

    def get_result(self):
        return self._result

    def reset(self):
        self._result = None

    def cleanup(self):
        self._input = None
        self._settings = None
        self._result = None
