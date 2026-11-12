# -*- coding: utf-8 -*-
"""Maya transform sampling / composition / channel solving.

All values are kept in Maya *internal* units (centimetres, radians).

Local matrix composition used by Maya (row vectors)::

    transform:  Sp^-1 S Sh Sp St Rp^-1 Ra R      Rp Rt    T
    joint:      Sp^-1 S Sh Sp St Rp^-1 Ra R JO   Rp Rt IS T

The parent space is never rebuilt by hand.  For every sampled frame the
*effective* parent matrix is measured as ``P = local^-1 * world`` so
offsetParentMatrix, inheritsTransform, parent scale etc. are honoured exactly
as Maya evaluated them.  Every sample is also checked by recomposing the local
matrix from the channels; a mismatch aborts safely instead of baking wrong
values.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import maya.api.OpenMaya as om

from ..utils import math_utils as mu
from ..utils.maya_utils import SimflowError, dg_time, get_mobject

COMPOSE_TOLERANCE = 1.0e-4

_VEC_ATTRS = {
    "t": "translate", "r": "rotate", "s": "scale", "sh": "shear",
    "ra": "rotateAxis", "rp": "rotatePivot", "rpt": "rotatePivotTranslate",
    "sp": "scalePivot", "spt": "scalePivotTranslate",
}
_JOINT_VEC_ATTRS = {"jo": "jointOrient", "is": "inverseScale"}


@dataclass
class FrameSample:
    frame: float
    world: list
    local: list
    ch: dict
    parent: list = field(default=None)        # space the pose lives in (world = local * parent)
    solve_parent: list = field(default=None)  # parent the channels are solved against after bake
    scene_world: list = field(default=None)   # world as evaluated in the scene (before any carry)

    def __post_init__(self):
        if self.parent is None:
            try:
                self.parent = mu.m_mul(mu.m_inverse(self.local), self.world)
            except ValueError:
                raise SimflowError("フレーム %s のローカル行列が逆行列を持ちません（スケールが 0 ですか？）" % self.frame)
        if self.solve_parent is None:
            self.solve_parent = self.parent
        if self.scene_world is None:
            self.scene_world = self.world

    def copy_at(self, frame):
        return FrameSample(frame=frame, world=list(self.world), local=list(self.local), ch=dict(self.ch),
                           parent=list(self.parent), solve_parent=list(self.solve_parent),
                           scene_world=list(self.scene_world))

    def carried(self, delta, structural):
        """Pose moved rigidly by ``delta`` (row matrix applied after the world).

        structural=True : the controller really is a descendant of the moved
                          node, so its channels must be solved against the moved
                          parent as well.
        structural=False: virtual carry (chain of unrelated nodes); channels are
                          still solved against the real parent."""
        world = mu.m_mul(self.world, delta)
        parent = mu.m_mul(self.parent, delta)
        solve_parent = parent if structural else self.solve_parent
        return FrameSample(frame=self.frame, world=world, local=list(self.local), ch=dict(self.ch),
                           parent=parent, solve_parent=solve_parent, scene_world=self.scene_world)


class ControllerSampler(object):
    """Fast om2 based sampler for one transform / joint."""

    def __init__(self, node):
        self.node = node
        mobj = get_mobject(node)
        if not mobj.hasFn(om.MFn.kTransform):
            raise SimflowError("%s はトランスフォームまたはジョイントではありません" % node)
        self.is_joint = mobj.hasFn(om.MFn.kJoint)
        fn = om.MFnDependencyNode(mobj)
        self._world = fn.findPlug("worldMatrix", False).elementByLogicalIndex(0)
        self._local = fn.findPlug("matrix", False)
        self._ro = fn.findPlug("rotateOrder", False)
        self._vec = {}
        attrs = dict(_VEC_ATTRS)
        if self.is_joint:
            attrs.update(_JOINT_VEC_ATTRS)
            self._ssc = fn.findPlug("segmentScaleCompensate", False)
        for key, attr in attrs.items():
            plug = fn.findPlug(attr, False)
            self._vec[key] = [plug.child(i) for i in range(3)]

    @staticmethod
    def _matrix(plug):
        return list(om.MFnMatrixData(plug.asMObject()).matrix())

    def read(self, frame):
        """Read the plugs in the CURRENT DG context (see :func:`sample_many`)."""
        world = self._matrix(self._world)
        local = self._matrix(self._local)
        ch = {k: tuple(p.asDouble() for p in plugs) for k, plugs in self._vec.items()}
        ch["ro"] = self._ro.asInt()
        ch["joint"] = self.is_joint
        if self.is_joint:
            ch["ssc"] = self._ssc.asBool()
        return FrameSample(frame=frame, world=world, local=local, ch=ch)

    def sample(self, frame):
        with dg_time(frame):
            return self.read(frame)


def sample_many(samplers, frames, progress=None, lo=0.0, hi=1.0):
    """Sample several controllers with ONE DG context per frame (cheaper than
    one context per controller and frame).  Returns a list per sampler."""
    out = [[] for _ in samplers]
    n = max(1, len(frames))
    for i, f in enumerate(frames):
        if progress is not None and i % 10 == 0:
            progress.check(lo + (hi - lo) * i / n, "サンプリング中")
        with dg_time(f):
            for k, sampler in enumerate(samplers):
                out[k].append(sampler.read(f))
    return out


# ---------------------------------------------------------------------------
# Composition
# ---------------------------------------------------------------------------

def _t(v):
    m = om.MMatrix()
    m.setElement(3, 0, v[0])
    m.setElement(3, 1, v[1])
    m.setElement(3, 2, v[2])
    return m


def _s(v):
    m = om.MMatrix()
    m.setElement(0, 0, v[0])
    m.setElement(1, 1, v[1])
    m.setElement(2, 2, v[2])
    return m


def _sh(v):
    m = om.MMatrix()
    m.setElement(1, 0, v[0])
    m.setElement(2, 0, v[1])
    m.setElement(2, 1, v[2])
    return m


def _neg(v):
    return (-v[0], -v[1], -v[2])


def euler_matrix(r, order=0):
    return om.MEulerRotation(r[0], r[1], r[2], order).asMatrix()


def _inverse_scale(ch):
    if ch.get("joint") and ch.get("ssc", True):
        iv = ch.get("is", (1.0, 1.0, 1.0))
        if any(abs(x) < 1.0e-12 for x in iv):
            raise SimflowError("ジョイントの inverseScale に 0 が含まれています")
        return _s((1.0 / iv[0], 1.0 / iv[1], 1.0 / iv[2]))
    return om.MMatrix()


def _jo(ch):
    return euler_matrix(ch["jo"]) if ch.get("joint") else om.MMatrix()


def compose_local(ch, r=None, t=None):
    """Recompose the local matrix from channels (optionally overriding
    rotate / translate).  Returns ``om.MMatrix``."""
    r = ch["r"] if r is None else r
    t = ch["t"] if t is None else t
    sp, rp = ch["sp"], ch["rp"]
    m = (_t(_neg(sp)) * _s(ch["s"]) * _sh(ch["sh"]) * _t(sp) * _t(ch["spt"]) *
         _t(_neg(rp)) * euler_matrix(ch["ra"]) * euler_matrix(r, ch["ro"]) * _jo(ch) *
         _t(rp) * _t(ch["rpt"]) * _inverse_scale(ch) * _t(t))
    return m


def verify_sample(sample):
    """Make sure our composition reproduces Maya's local matrix."""
    composed = compose_local(sample.ch)
    if not composed.isEquivalent(om.MMatrix(sample.local), COMPOSE_TOLERANCE):
        raise SimflowError(
            "フレーム %s で、コントローラーのローカル行列をチャンネルの値から再現できません"
            "（対応していないトランスフォームの構成です）。リグを守るため中止しました。" % sample.frame)


def pivot_parent_space(ch):
    """Rotate pivot (the point invariant under rotate channel changes) in
    the effective parent space."""
    p = mu.v_add(ch["rp"], ch["rpt"])
    if ch.get("joint") and ch.get("ssc", True):
        iv = ch.get("is", (1.0, 1.0, 1.0))
        p = (p[0] / iv[0], p[1] / iv[1], p[2] / iv[2])
    return mu.v_add(p, ch["t"])


def pivot_world(ch, parent):
    return mu.m_transform_point(parent, pivot_parent_space(ch))


def world_rotation(world):
    return mu.m_rotation_quat(world)


# ---------------------------------------------------------------------------
# Channel solve
# ---------------------------------------------------------------------------

def _rot3(m):
    out = om.MMatrix(m)
    out.setElement(3, 0, 0.0)
    out.setElement(3, 1, 0.0)
    out.setElement(3, 2, 0.0)
    return out


def _aim_first_orthonormal(rc, u):
    """Closest proper rotation to the (possibly non-orthogonal) 3x3 ``rc``
    that maps direction ``u`` exactly like ``rc`` does.

    With a non-uniformly scaled parent a world-space rotation cannot be
    represented by rotate channels alone (it would need shear).  Keeping the
    aim direction exact keeps the pendulum direction exact; only the twist
    about the aim axis is approximated.  For orthonormal ``rc`` the result is
    ``rc`` itself."""
    rows = [tuple(rc.getElement(i, j) for j in range(3)) for i in range(3)]

    def mul(v):   # row vector * rc
        return tuple(sum(v[k] * rows[k][j] for k in range(3)) for j in range(3))

    b0 = mu.v_normalize(u, (1.0, 0.0, 0.0))
    helper = (0.0, 1.0, 0.0) if abs(b0[1]) < 0.9 else (0.0, 0.0, 1.0)
    b1 = mu.v_normalize(mu.v_cross(helper, b0))
    b2 = mu.v_cross(b0, b1)
    basis = (b0, b1, b2)                      # orthonormal, first row = u
    # images of the basis under rc, Gram-Schmidt with the aim image first
    i0 = mu.v_normalize(mul(b0))
    i1 = mul(b1)
    i1 = mu.v_normalize(mu.v_sub(i1, mu.v_scale(i0, mu.v_dot(i0, i1))))
    i2 = mu.v_cross(i0, i1)
    if mu.v_dot(i2, mul(b2)) < 0.0:           # mirrored input: keep proper rotation
        i2 = mu.v_scale(i2, -1.0)
        i1 = mu.v_scale(i1, -1.0)
    images = (i0, i1, i2)
    # R = basis^T * images  (maps b_k -> i_k)
    out = om.MMatrix()
    for r in range(3):
        for c in range(3):
            out.setElement(r, c, sum(basis[k][r] * images[k][c] for k in range(3)))
    return out


def solve_channels(ch, desired_local, ref_r, aim=(1.0, 0.0, 0.0)):
    """Find rotate / translate channel values (internal units) so that the
    local matrix equals ``desired_local`` while every other channel keeps its
    current value.

    * Rotation: ``R = Ra^-1 (S Sh)^-1 M (IS)^-1 JO^-1`` orthonormalised so the
      controller's aim axis keeps its exact direction, converted to the
      controller's rotate order choosing the Euler solution closest to
      ``ref_r`` (Euler filter).
    * Translation: ``T = M.t - compose(R, T=0).t``.
    """
    desired = om.MMatrix(desired_local)
    a3 = _s(ch["s"]) * _sh(ch["sh"])
    rot = (euler_matrix(ch["ra"]).inverse() * a3.inverse() * _rot3(desired) *
           _inverse_scale(ch).inverse() * _jo(ch).inverse())
    # aim direction expressed in the space the rotate channels act on
    pre = _rot3(a3 * euler_matrix(ch["ra"]))
    u = tuple(sum(aim[k] * pre.getElement(k, j) for k in range(3)) for j in range(3))
    rot = _aim_first_orthonormal(rot, u)
    quat = om.MTransformationMatrix(rot).rotation(asQuaternion=True)
    euler = quat.asEulerRotation().reorder(ch["ro"])
    ref = om.MEulerRotation(ref_r[0], ref_r[1], ref_r[2], ch["ro"])
    euler = euler.closestSolution(ref)
    r = (euler.x, euler.y, euler.z)
    no_t = compose_local(ch, r=r, t=(0.0, 0.0, 0.0))
    t = (desired.getElement(3, 0) - no_t.getElement(3, 0),
         desired.getElement(3, 1) - no_t.getElement(3, 1),
         desired.getElement(3, 2) - no_t.getElement(3, 2))
    return t, r


def matrix_error(a, b):
    """(translation distance, rotation angle) between two world matrices."""
    ta, tb = mu.m_translation(a), mu.m_translation(b)
    dist = mu.v_length(mu.v_sub(ta, tb))
    angle = mu.q_angle(mu.m_rotation_quat(a), mu.m_rotation_quat(b))
    return dist, angle
