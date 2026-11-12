# -*- coding: utf-8 -*-
"""Rotation gravity / pendulum terms (spec 2.2).

The controller is treated as a pivot with a virtual point mass ``m`` at
distance ``L`` along ``tip_local`` (the "Pendulum Length" along the aim axis).

Angular acceleration produced by an acceleration field ``a`` acting on the tip
(gravity, and the inertial pseudo-force of an accelerating pivot)::

    torque  tau   = r x (m * a)          r = q * tip_local * L
    inertia I     = m * L^2
    alpha         = tau / I = (r x a) / L^2

Gravity is therefore applied as a *torque* computed from pivot, tip position
and the world gravity vector - it is never added to Euler angles.
"""
from __future__ import annotations

from ..utils import math_utils as mu


def tip_vector(q, tip_local, length):
    """World-space vector pivot -> tip."""
    return mu.v_scale(mu.q_rotate(q, tip_local), length)


def angular_acceleration(q, tip_local, length, accel):
    """Angular acceleration (rad/s^2, world) caused by acceleration field
    ``accel`` (scene units / s^2) acting on the virtual tip mass."""
    if length <= 1.0e-9:
        return (0.0, 0.0, 0.0)
    r = tip_vector(q, tip_local, length)
    inv_l2 = 1.0 / (length * length)
    return mu.v_scale(mu.v_cross(r, accel), inv_l2)


def effective_tip_acceleration(gravity_vec, pivot_accel, inertia_scale):
    """Acceleration felt by the tip in the pivot's (accelerating) frame:
    gravity minus the pivot acceleration (d'Alembert pseudo-force)."""
    return mu.v_sub(gravity_vec, mu.v_scale(pivot_accel, inertia_scale))


def static_sag_angle(gravity, length, omega2):
    """Small-angle equilibrium deflection (radians) for a horizontal arm:
    ``omega^2 * theta = g / L``  ->  ``theta = g / (L * omega^2)``.
    Used by tests / documentation only."""
    if length <= 0 or omega2 <= 0:
        return float("inf")
    return gravity / (length * omega2)
