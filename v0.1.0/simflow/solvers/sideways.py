# -*- coding: utf-8 -*-
"""「横揺れ抑え」: remove the horizontal part of the swing.

Horizontal = world horizontal (perpendicular to gravity): left / right AND
front / back.  Only the up / down motion (the bounce of a jump, a flick up /
down) is left.  Measured against the ANIMATED pose of the same frame, so a
tilted head / neck is part of the reference (the hair is not pulled towards
some fixed pose).

Position based, like 横のつながり: after every sub step a fraction of the
horizontal distance from the animation (and of the horizontal speed relative
to the animation) is removed by turning the segments about their pivots.
100 = no horizontal deviation at all.

Pure Python, no Maya dependency.
"""
from __future__ import annotations

import math

from ..utils import math_utils as mu

SIDE_RATE = 4.0             # 1/s at 50; s / (1 - s) times this in general (100: all, every sub step)


def up_vector(gravity_direction):
    return mu.v_scale(mu.v_normalize(tuple(gravity_direction), (0.0, -1.0, 0.0)), -1.0)


def horizontal_axes(up):
    """Two unit axes spanning the plane perpendicular to ``up``."""
    ref = (1.0, 0.0, 0.0) if abs(up[0]) < 0.9 else (0.0, 0.0, 1.0)
    a = mu.v_normalize(mu.v_cross(up, ref))
    b = mu.v_normalize(mu.v_cross(up, a))
    return a, b


def alpha(strength, h):
    """Fraction removed in one sub step of ``h`` seconds."""
    if strength >= 1.0:
        return 1.0
    return 1.0 - math.exp(-h * SIDE_RATE * strength / (1.0 - strength))
