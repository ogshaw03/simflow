# -*- coding: utf-8 -*-
"""Camera sampling for the on-screen check of the warp absorption."""
from __future__ import annotations

import maya.api.OpenMaya as om
import maya.cmds as cmds

from ..utils import math_utils as mu
from ..utils.maya_utils import SimflowError, get_mobject

MM_PER_INCH = 25.4


def resolve(name):
    """Camera shape for a camera transform or shape name."""
    if not name or not cmds.objExists(name):
        raise SimflowError("ワープの判定に使うカメラ「%s」が見つかりません。カメラを選び直してください。" % name)
    if cmds.nodeType(name) == "camera":
        return cmds.ls(name, long=True)[0]
    shapes = cmds.listRelatives(name, shapes=True, type="camera", fullPath=True) or []
    if not shapes:
        raise SimflowError("「%s」はカメラではありません。" % name)
    return shapes[0]


def cameras():
    """Camera transforms in the scene (short names), persp first."""
    out = []
    for shape in cmds.ls(type="camera", long=True) or []:
        parent = cmds.listRelatives(shape, parent=True, fullPath=True)
        if parent:
            out.append(cmds.ls(parent[0])[0])
    startup = [c for c in out if c in ("persp", "top", "front", "side")]
    return [c for c in out if c not in startup] + [c for c in ("persp",) if c in startup]


class CameraSampler(object):
    """Reads (inverse world matrix, tan(horizontal fov / 2)) in the CURRENT
    DG context (see transform_utils.sample_many)."""

    def __init__(self, name):
        self.node = resolve(name)
        fn = om.MFnDependencyNode(get_mobject(self.node))
        self._world = fn.findPlug("worldMatrix", False).elementByLogicalIndex(0)
        self._focal = fn.findPlug("focalLength", False)
        self._hfa = fn.findPlug("horizontalFilmAperture", False)

    def read(self, frame=None):
        m = list(om.MFnMatrixData(self._world.asMObject()).matrix())
        focal = self._focal.asDouble()
        tan_h = (self._hfa.asDouble() * MM_PER_INCH * 0.5) / focal if focal > 1.0e-9 else 0.0
        return (mu.m_inverse(m), tan_h)


def signature(rows):
    import hashlib
    h = hashlib.sha1()
    for inv, tan_h in rows or ():
        h.update((",".join("%.5g" % v for v in inv) + "|%.5g;" % tan_h).encode())
    return h.hexdigest()
