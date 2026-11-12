# -*- coding: utf-8 -*-
"""Proxy creation (spec 6).

Proxies live under ``SIMFLOW_ROOT`` and never touch the rig: no parenting,
no constraints, no connections to the controllers.  They are keyed from the
sampled / simulated data so timeline scrubbing simply plays back the cache.

* Input Proxy  (grey)  - the original animated pose (pivot + orientation)
* Output Proxy (yellow) - the final simulated pose
Both show a cross at the pivot and a line to the virtual pendulum tip.
"""
from __future__ import annotations

import maya.cmds as cmds
import maya.api.OpenMaya as om

from ..utils import math_utils as mu
from ..utils.maya_utils import cm_to_ui, rad_to_ui, short_name, suspend_refresh
from . import scene_manager

INPUT_COLOR = (0.55, 0.55, 0.55)
OUTPUT_COLOR = (1.0, 0.78, 0.1)


def _safe(name):
    return short_name(name).replace(":", "_")


def _make_shape(parent, tip_local, length, size, color):
    lines = [
        [(-size, 0, 0), (size, 0, 0)],
        [(0, -size, 0), (0, size, 0)],
        [(0, 0, -size), (0, 0, size)],
    ]
    if length > 1.0e-6:
        tip = mu.v_scale(tip_local, cm_to_ui(length))
        d = size * 0.35
        lines.append([(0, 0, 0), tip])
        lines.append([mu.v_add(tip, (d, 0, 0)), mu.v_add(tip, (0, d, 0)), mu.v_add(tip, (-d, 0, 0)),
                      mu.v_add(tip, (0, -d, 0)), mu.v_add(tip, (d, 0, 0))])
    for pts in lines:
        tmp = cmds.curve(degree=1, point=pts)
        shape = cmds.listRelatives(tmp, shapes=True, fullPath=True)[0]
        cmds.parent(shape, parent, relative=True, shape=True)
        cmds.delete(tmp)
    for shape in cmds.listRelatives(parent, shapes=True, fullPath=True) or []:
        cmds.setAttr(shape + ".overrideEnabled", 1)
        cmds.setAttr(shape + ".overrideRGBColors", 1)
        cmds.setAttr(shape + ".overrideColorRGB", *color)
        cmds.setAttr(shape + ".lineWidth", 2.0)


def _euler_sequence(quats):
    out = []
    prev = None
    for q in quats:
        e = om.MQuaternion(q[0], q[1], q[2], q[3]).asEulerRotation()
        if prev is not None:
            e = e.closestSolution(prev)
        prev = e
        out.append((e.x, e.y, e.z))
    return out


def _key(node, frames, positions, quats):
    eulers = _euler_sequence(quats)
    values = {
        "translateX": [cm_to_ui(p[0]) for p in positions],
        "translateY": [cm_to_ui(p[1]) for p in positions],
        "translateZ": [cm_to_ui(p[2]) for p in positions],
        "rotateX": [rad_to_ui(e[0]) for e in eulers],
        "rotateY": [rad_to_ui(e[1]) for e in eulers],
        "rotateZ": [rad_to_ui(e[2]) for e in eulers],
    }
    for attr, vals in values.items():
        for f, v in zip(frames, vals):
            cmds.setKeyframe(node, attribute=attr, time=f, value=v,
                             inTangentType="linear", outTangentType="linear")


def delete_proxies(entry_id=None):
    nodes = (scene_manager.tool_nodes("inputProxy", entry_id) +
             scene_manager.tool_nodes("outputProxy", entry_id))
    return scene_manager.delete_nodes(nodes)


def build_proxies(entry, result):
    """(Re)create Input / Output proxies for one entry from a SimResult."""
    delete_proxies(entry.id)
    length = result.pendulum_length
    size = max(0.5, cm_to_ui(length) * 0.25 if length > 0 else 2.0)
    created = {}
    frames = [fr.frame for fr in result.frame_results]
    with suspend_refresh():
        for role, group, color, prefix, pos_q in (
                ("inputProxy", "SIMFLOW_INPUT", INPUT_COLOR, "SIMFLOW_IN_",
                 [(fr.orig_pivot, fr.orig_quat) for fr in result.frame_results]),
                ("outputProxy", "SIMFLOW_OUTPUT", OUTPUT_COLOR, "SIMFLOW_OUT_",
                 [(fr.pivot, fr.quat) for fr in result.frame_results])):
            parent = scene_manager.get_group(group)
            node = cmds.createNode("transform", name=prefix + _safe(entry.path),
                                   parent=parent, skipSelect=True)
            node = cmds.ls(node, long=True)[0]
            scene_manager.tag(node, role, entry.id)
            _make_shape(node, result.tip_local, length, size, color)
            _key(node, frames, [p for p, _ in pos_q], [q for _, q in pos_q])
            created[role] = node
    return created


def set_group_visibility(group, visible):
    root = scene_manager.get_root(create=False)
    if not root:
        return
    grp = scene_manager.get_group(group)
    cmds.setAttr(grp + ".visibility", bool(visible))
