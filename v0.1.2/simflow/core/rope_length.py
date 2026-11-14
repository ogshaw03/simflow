# -*- coding: utf-8 -*-
"""The real length of a rope rig ("縄・ひも" 縄の長さ), read from its spline IK.

* a curve (or its transform): the length of its ORIGINAL shape - the shape
  before deformers (clusters / skin moved by the controllers), i.e. the rope's
  rest length; a curve without deformers: its current length;
* an ikHandle (spline IK): its curve, as above;
* a joint: the chain below it (first child joint each step): the sum of the
  joints' offsets (translate), i.e. the bone lengths - a stretchy spline IK
  that stretches with scale keeps these.

Lengths are in the scene's linear unit (what the UI shows).
"""
from __future__ import annotations

import math

import maya.cmds as cmds

from ..utils.maya_utils import SimflowError


def _curve_shapes(node):
    if cmds.nodeType(node) == "nurbsCurve":
        parent = cmds.listRelatives(node, parent=True, fullPath=True)
        node = parent[0] if parent else node
    return [s for s in cmds.listRelatives(node, shapes=True, fullPath=True) or [] if cmds.nodeType(s) == "nurbsCurve"]


def _curve_length(node):
    shapes = _curve_shapes(node)
    if not shapes:
        return None
    orig = [s for s in shapes if cmds.getAttr(s + ".intermediateObject")]
    shape = orig[0] if orig else shapes[0]
    # (cmds.arclen does not work on an intermediate shape)
    import maya.api.OpenMaya as om
    sel = om.MSelectionList()
    sel.add(shape)
    return om.MFnNurbsCurve(sel.getDagPath(0)).length(), ("カーブの元の形（変形前）の長さ" if orig else "カーブの長さ")


def _joint_length(joint):
    total, cur, n = 0.0, joint, 0
    while True:
        kids = cmds.listRelatives(cur, children=True, type="joint", fullPath=True) or []
        if not kids:
            break
        cur = kids[0]
        t = cmds.getAttr(cur + ".translate")[0]
        total += math.sqrt(sum(x * x for x in t))
        n += 1
    if n == 0:
        return None
    return total, "ジョイント %d 本の長さの合計" % n


def measure(node):
    """(length in the scene's linear unit, how it was measured) for ``node``."""
    if not node or not cmds.objExists(node):
        raise SimflowError("スプライン IK のカーブ、ルートのジョイント、または IK ハンドルを選んでください。")
    kind = cmds.nodeType(node)
    if kind == "ikHandle":
        crv = cmds.ikHandle(node, query=True, curve=True)
        got = _curve_length(crv) if crv else None
        if got is None:
            start = cmds.ikHandle(node, query=True, startJoint=True)
            got = _joint_length(start) if start else None
    elif kind == "joint":
        got = _joint_length(node)
    else:
        got = _curve_length(node)
    if got is None or got[0] <= 1.0e-6:
        raise SimflowError("選んだノードから縄の長さを読めませんでした（%s）。スプライン IK のカーブ、"
                           "ルートのジョイント、または IK ハンドルを選んでください。" % node.split("|")[-1])
    return got
