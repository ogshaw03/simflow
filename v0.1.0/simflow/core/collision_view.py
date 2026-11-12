# -*- coding: utf-8 -*-
"""Show the collision shapes a swing group was pushed by ("押し出す揺れもの"):
the other groups' bones (cylinders, radius = their 太さ) and the cloth panels
between neighbouring chains (one mesh), keyed from the last calculation so
they move with the timeline.  Display only: nothing reads them back.

They live under ``SIMFLOW_ROOT|SIMFLOW_COLLISION_VIEW`` (removed by Clean Up).
"""
from __future__ import annotations

import maya.cmds as cmds
import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma

from ..utils import math_utils as mu
from ..utils.maya_utils import suspend_refresh, undo_chunk
from . import scene_manager

GROUP = "SIMFLOW_COLLISION_VIEW"
ROLE = "collisionView"
COLOR = (1.0, 0.25, 0.2)


def remove():
    nodes = scene_manager.tool_nodes(ROLE)
    with undo_chunk("SIMFLOW Collision View Off", rollback=True):
        scene_manager.delete_nodes([n for n in nodes if cmds.objExists(n)])


def shown():
    return bool(scene_manager.tool_nodes(ROLE))


def _plug(name):
    sel = om.MSelectionList()
    sel.add(name)
    return sel.getPlug(0)


def _key_plug(name, frames, values, kind):
    """One animation curve on plug ``name`` (values in internal units)."""
    plug = _plug(name)
    times = om.MTimeArray([om.MTime(f, om.MTime.uiUnit()) for f in frames])
    fn = oma.MFnAnimCurve()
    fn.create(plug, kind)
    fn.addKeys(times, om.MDoubleArray(values), oma.MFnAnimCurve.kTangentLinear,
               oma.MFnAnimCurve.kTangentLinear)


def _material():
    for n in scene_manager.tool_nodes(ROLE):
        if cmds.nodeType(n) == "shadingEngine":
            return n
    mat = cmds.shadingNode("lambert", asShader=True, name="SIMFLOW_collisionView_mat", skipSelect=True)
    cmds.setAttr(mat + ".color", *COLOR, type="double3")
    cmds.setAttr(mat + ".transparency", 0.55, 0.55, 0.55, type="double3")
    sg = cmds.sets(renderable=True, noSurfaceShader=True, empty=True, name="SIMFLOW_collisionView_SG")
    cmds.connectAttr(mat + ".outColor", sg + ".surfaceShader", force=True)
    scene_manager.tag(mat, ROLE)
    scene_manager.tag(sg, ROLE)
    return sg


def build(view, label=""):
    """``view``: {"frames": [...], "cols": per frame [collider, ...]} from
    SimulationManager.group_view.  Returns the number of bones / panels shown."""
    frames = view["frames"]
    cols = view["cols"]
    if not frames or not cols or not cols[0]:
        return 0
    remove()
    with undo_chunk("SIMFLOW Collision View", rollback=True), suspend_refresh():
        group = scene_manager.get_group(GROUP)
        sg = _material()
        bones = [k for k, c in enumerate(cols[0]) if not (len(c) > 5 and c[4] == "tri")]
        tris = [k for k, c in enumerate(cols[0]) if len(c) > 5 and c[4] == "tri"]
        shapes = []
        # bones: a cylinder along +Y, height 1 / radius 1, moved / turned / stretched
        for n, k in enumerate(bones):
            tr = cmds.polyCylinder(name="SIMFLOW_bone%d_view" % n, height=1.0, radius=1.0, axis=(0, 1, 0),
                                   subdivisionsAxis=10, roundCap=False, constructionHistory=False)[0]
            tr = cmds.parent(tr, group)[0]
            tr = cmds.ls(tr, long=True)[0]
            scene_manager.tag(tr, ROLE)
            radius = max(cols[0][k][3], 0.05)
            tx, ty, tz, rx, ry, rz, sy = [], [], [], [], [], [], []
            prev = None
            for row in cols:
                center, axis, half = row[k][0], row[k][1], row[k][2]
                q = _between((0.0, 1.0, 0.0), axis)
                e = om.MQuaternion(*q).asEulerRotation()
                if prev is not None:
                    e = e.closestSolution(prev)
                prev = e
                tx.append(center[0])
                ty.append(center[1])
                tz.append(center[2])
                rx.append(e.x)
                ry.append(e.y)
                rz.append(e.z)
                sy.append(max(2.0 * half, 1.0e-3))
            for attr, vals, kind in (("tx", tx, oma.MFnAnimCurve.kAnimCurveTL), ("ty", ty, oma.MFnAnimCurve.kAnimCurveTL),
                                     ("tz", tz, oma.MFnAnimCurve.kAnimCurveTL), ("rx", rx, oma.MFnAnimCurve.kAnimCurveTA),
                                     ("ry", ry, oma.MFnAnimCurve.kAnimCurveTA), ("rz", rz, oma.MFnAnimCurve.kAnimCurveTA),
                                     ("sy", sy, oma.MFnAnimCurve.kAnimCurveTU)):
                _key_plug(tr + "." + attr, frames, vals, kind)
            cmds.setAttr(tr + ".sx", radius)
            cmds.setAttr(tr + ".sz", radius)
            shapes.append(tr)
        # panels: one mesh, 3 own vertices per triangle, the vertex offsets keyed
        if tris:
            pts = om.MPointArray()
            counts, conn = [], []
            for k in tris:
                for p in cols[0][k][5]:
                    conn.append(len(pts))
                    pts.append(om.MPoint(*p))
                counts.append(3)
            fn = om.MFnMesh()
            mesh_tr = fn.create(pts, counts, conn)
            mesh_tr = om.MFnDagNode(mesh_tr).fullPathName()
            mesh_tr = cmds.rename(mesh_tr, "SIMFLOW_panels_view")
            mesh_tr = cmds.parent(mesh_tr, group)[0]
            mesh_tr = cmds.ls(mesh_tr, long=True)[0]
            scene_manager.tag(mesh_tr, ROLE)
            shape = cmds.listRelatives(mesh_tr, shapes=True, fullPath=True)[0]
            v = 0
            for k in tris:
                for j in range(3):
                    base = cols[0][k][5][j]
                    for axis_i, attr in enumerate(("pntx", "pnty", "pntz")):
                        _key_plug("%s.pnts[%d].%s" % (shape, v, attr), frames,
                                  [row[k][5][j][axis_i] - base[axis_i] for row in cols],
                                  oma.MFnAnimCurve.kAnimCurveTL)
                    v += 1
            shapes.append(mesh_tr)
        for tr in shapes:
            cmds.sets(tr, edit=True, forceElement=sg)
            for sh in cmds.listRelatives(tr, shapes=True, fullPath=True) or []:
                cmds.setAttr(sh + ".castsShadows", 0)
                cmds.setAttr(sh + ".receiveShadows", 0)
                cmds.setAttr(sh + ".primaryVisibility", 0)
        cmds.setAttr(group + ".overrideEnabled", 1)
        cmds.setAttr(group + ".overrideDisplayType", 2)      # reference: not selectable in the viewport
    cmds.currentTime(cmds.currentTime(query=True), update=True)
    return len(bones) + len(tris)


def _between(a, b):
    from ..solvers.collision import rotation_between
    return rotation_between(a, b)
