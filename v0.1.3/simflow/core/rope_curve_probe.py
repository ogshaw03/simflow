# -*- coding: utf-8 -*-
"""Find the curve that draws a rope and measure how its controllers move it
(縄の当たり判定をカーブで見る, see solvers/rope_curve.py).

Nothing about a particular rig is assumed: every curve in the scene that is
built / deformed by something (its ``create`` input is connected) and that
moves when the rope's controllers move is a candidate.  The rig is not read,
it is MEASURED: each controller is offset a little through its
``rotatePivotTranslate`` (a pure translation on top of its animation, layers
and constraints, rarely keyed or connected) and the change of every CV is
read.  The offsets are put back right away; the undo queue and the keys are
not touched.
"""
from __future__ import annotations

import logging

import maya.api.OpenMaya as om
import maya.cmds as cmds

from ..solvers import rope_curve
from ..utils import math_utils as mu
from ..utils.maya_utils import get_mobject

LOG = logging.getLogger("simflow")

PROBE_FRACTION = 0.01        # offset = this x the shortest distance between rope points


def _cvs(plug):
    data = plug.asMObject()
    fn = om.MFnNurbsCurve(data)
    return [(p.x, p.y, p.z) for p in fn.cvPositions(om.MSpace.kWorld)]


def _world_pos(plug):
    m = om.MFnMatrixData(plug.asMObject()).matrix()
    return (m[12], m[13], m[14])


def _candidates(rope_nodes):
    own = set()
    for n in rope_nodes:
        for s in cmds.listRelatives(n, shapes=True, fullPath=True) or []:
            own.add(s)
    out = []
    for s in cmds.ls(type="nurbsCurve", noIntermediate=True, long=True) or []:
        if s in own:
            continue
        if not cmds.listConnections(s + ".create", source=True, destination=False):
            continue                     # a plain curve (a controller's shape): not built from anything
        out.append(s)
    return out


class _Curve(object):
    def __init__(self, shape):
        self.shape = shape
        fn = om.MFnDependencyNode(get_mobject(shape))
        self.plug = fn.findPlug("worldSpace", False).elementByLogicalIndex(0)

    def read(self, frame=None):
        return _cvs(self.plug)


class CurveSampler(_Curve):
    """CVs of one curve per frame (for transform_utils.sample_many)."""


def _set_rpt(node, value):
    cmds.setAttr(node + ".rotatePivotTranslate", value[0], value[1], value[2], type="double3")


def find(rope_nodes, movable):
    """``rope_nodes``: the rope's points in rope order (grips first / last).
    ``movable``: indices of the points the solver moves.  Returns
    (shape, degree, knots, jac) or None and a message (Japanese)."""
    try:
        cands = _candidates(rope_nodes)
    except Exception as exc:                 # pragma: no cover - Maya errors
        return None, "カーブを探せませんでした（%s）" % exc
    if not cands:
        return None, "縄を描くカーブが見つかりませんでした（コントローラーの点で当たりを見ます）"
    curves = [_Curve(s) for s in cands]
    pos_plugs = [om.MFnDependencyNode(get_mobject(n)).findPlug("worldMatrix", False).elementByLogicalIndex(0)
                 for n in rope_nodes]
    pos0 = [_world_pos(p) for p in pos_plugs]
    gaps = [mu.v_length(mu.v_sub(pos0[i + 1], pos0[i])) for i in range(len(pos0) - 1)]
    gaps = [g for g in gaps if g > 1.0e-6]
    d = PROBE_FRACTION * (min(gaps) if gaps else 1.0)
    probe = [i for i in movable]
    saved = {}
    undo = cmds.undoInfo(query=True, stateWithoutFlush=True)
    auto = cmds.autoKeyframe(query=True, state=True)
    cmds.undoInfo(stateWithoutFlush=False)
    cmds.autoKeyframe(state=False)
    skipped = []
    try:
        for i in probe:
            n = rope_nodes[i]
            try:
                if cmds.getAttr(n + ".rotatePivotTranslate", lock=True) or \
                        cmds.listConnections(n + ".rotatePivotTranslate", source=True, destination=False) or \
                        any(cmds.listConnections(n + ".rotatePivotTranslate" + a, source=True, destination=False)
                            for a in ("X", "Y", "Z")):
                    skipped.append(n)
                    continue
                saved[i] = tuple(cmds.getAttr(n + ".rotatePivotTranslate")[0])
            except Exception:
                skipped.append(n)
        if not saved:
            return None, "コントローラーを動かしてカーブとの関係を調べられませんでした（コントローラーの点で当たりを見ます）"
        base = [c.read() for c in curves]
        # 1) all controllers at once: which curves move with the rope?
        for i, v in saved.items():
            _set_rpt(rope_nodes[i], (v[0] + d, v[1] + d, v[2] + d))
        moved = []
        for c, b in zip(curves, base):
            now = c.read()
            if len(now) == len(b) and max(mu.v_length(mu.v_sub(x, y))
                                          for x, y in zip(now, b)) > 1.0e-3 * d:
                moved.append((c, b))
        for i, v in saved.items():
            _set_rpt(rope_nodes[i], v)
        if not moved:
            return None, "コントローラーと一緒に動くカーブが見つかりませんでした（コントローラーの点で当たりを見ます）"
        c, b = _pick(moved)
        # 2) each controller, each axis (both ways): the 3x3 matrix per CV
        n_cv = len(b)
        jac = [[None] * len(rope_nodes) for _ in range(n_cv)]
        for i, v in saved.items():
            node = rope_nodes[i]
            dW, dC = [], []
            for ax in range(3):
                plus = list(v)
                plus[ax] += d
                minus = list(v)
                minus[ax] -= d
                _set_rpt(node, plus)
                pp, cp = _world_pos(pos_plugs[i]), c.read()
                _set_rpt(node, minus)
                pm, cm = _world_pos(pos_plugs[i]), c.read()
                _set_rpt(node, v)
                dW.append(mu.v_sub(pp, pm))
                dC.append([mu.v_sub(x, y) for x, y in zip(cp, cm)])
            # columns = the three probes: J * [dW0 dW1 dW2] = [dC0 dC1 dC2]
            Wm = (dW[0][0], dW[1][0], dW[2][0], dW[0][1], dW[1][1], dW[2][1], dW[0][2], dW[1][2], dW[2][2])
            inv = rope_curve.m3_inverse(Wm)
            if inv is None:
                skipped.append(node)
                continue
            for k in range(n_cv):
                Cm = (dC[0][k][0], dC[1][k][0], dC[2][k][0], dC[0][k][1], dC[1][k][1], dC[2][k][1],
                      dC[0][k][2], dC[1][k][2], dC[2][k][2])
                if max(abs(x) for x in Cm) <= 1.0e-4 * d:
                    continue
                jac[k][i] = rope_curve.m3_mul(Cm, inv)
    finally:
        for i, v in saved.items():
            try:
                _set_rpt(rope_nodes[i], v)
            except Exception:
                pass
        cmds.autoKeyframe(state=auto)
        cmds.undoInfo(stateWithoutFlush=undo)
    sel = om.MSelectionList()
    sel.add(c.shape)
    fn = om.MFnNurbsCurve(sel.getDagPath(0))        # a curve data object has no degree / knots
    knots = list(fn.knots())
    degree = fn.degree
    used = sum(1 for i in range(len(rope_nodes)) if any(jac[k][i] is not None for k in range(n_cv)))
    msg = "縄の当たり判定はカーブ %s で見ます（CV %d 個、動かすコントローラー %d 個）" % (
        c.shape.split("|")[-1], n_cv, used)
    if skipped:
        msg += "。調べられなかったコントローラー: %s" % "、".join(s.split("|")[-1] for s in skipped)
    if fn.form == om.MFnNurbsCurve.kPeriodic:
        return None, "カーブ %s は閉じたカーブなので使えません（コントローラーの点で当たりを見ます）" % c.shape
    return (c.shape, degree, knots, jac), msg


def _pick(moved):
    """The curve that draws the rope: the one a spline IK uses, else the last
    one downstream (a rig may rebuild a control curve into the IK curve)."""
    for c, b in moved:
        if cmds.listConnections(c.shape + ".worldSpace", destination=True, source=False, type="ikHandle"):
            return c, b
    shapes = [c.shape for c, _ in moved]
    for c, b in moved:
        upstream = False
        for other in shapes:
            if other == c.shape:
                continue
            hist = cmds.listHistory(other, pruneDagObjects=False) or []
            if any(h == c.shape or cmds.ls(h, long=True) == [c.shape] for h in hist):
                upstream = True
                break
        if not upstream:
            return c, b
    return moved[0]
