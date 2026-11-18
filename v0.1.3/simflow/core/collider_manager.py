# -*- coding: utf-8 -*-
"""Collision shapes (spheres / capsules / floors) that follow rig nodes.

Scene layout (all tagged SIMFLOW nodes, the rig is never modified)::

    SIMFLOW_COLLIDERS                     (role "colliderRoot", top level)
      <name>_follow                       (role "colliderFollow"; parent / scale
                                           constrained to the rig node)
        <name>                            (role "collider"; the user moves /
                                           rotates it, size = attributes
                                           "radius" / "length")

The collider's local origin is the sphere centre / capsule centre, its local
Y axis the capsule axis.  A floor is an infinite plane through the origin,
local Y = up (attribute "size" only sets how big it is drawn).  The display is a wireframe mesh driven by the
size attributes.  Clean Up keeps colliders (they are set-up, not results);
they are removed with :func:`delete_collider`.
"""
from __future__ import annotations

import hashlib
import math

import maya.api.OpenMaya as om
import maya.cmds as cmds

from ..utils.logger import get_logger
from ..utils.maya_utils import SimflowError, add_string_attr, get_mobject, get_string_attr, short_name
from . import scene_manager

LOG = get_logger("colliders")

ROOT_NAME = "SIMFLOW_COLLIDERS"
KINDS = ("sphere", "capsule", "floor")
KIND_NAMES = {"sphere": "球", "capsule": "カプセル", "floor": "床"}
COLOR = (0.25, 0.75, 1.0)
NONUNIFORM = 1.0e-4      # relative difference of the axis scales above which a collider is an ellipsoid ("ell")


def _root():
    nodes = scene_manager.tool_nodes("colliderRoot")
    if nodes:
        return nodes[0]
    root = cmds.createNode("transform", name=ROOT_NAME, skipSelect=True)
    root = cmds.ls(root, long=True)[0]
    scene_manager.tag(root, "colliderRoot")
    for attr in ("t", "r", "s"):
        cmds.setAttr("%s.%s" % (root, attr), lock=True)
    return root


def _wire(shape):
    """Wireframe, coloured, not renderable."""
    cmds.setAttr(shape + ".overrideEnabled", 1)
    cmds.setAttr(shape + ".overrideShading", 0)
    cmds.setAttr(shape + ".overrideRGBColors", 1)
    cmds.setAttr(shape + ".overrideColorRGB", *COLOR)
    for attr in ("castsShadows", "receiveShadows", "primaryVisibility", "visibleInReflections",
                 "visibleInRefractions"):
        if cmds.attributeQuery(attr, node=shape, exists=True):
            cmds.setAttr("%s.%s" % (shape, attr), 0)


def _mesh_under(parent, make, name):
    """Create a primitive with history and move its mesh shape under ``parent``.
    Returns (shape, construction node)."""
    tr, node = make(name=name)
    shape = cmds.listRelatives(tr, shapes=True, fullPath=True)[0]
    shape = cmds.parent(shape, parent, shape=True, relative=True)[0]
    cmds.delete(tr)
    shape = cmds.ls(shape, long=True)[0]
    _wire(shape)
    scene_manager.tag(shape, "colliderShape")
    scene_manager.tag(node, "colliderShape")
    return shape, node


def create_collider(kind="sphere", attach=None, name=None, radius=None, length=None):
    """Create a collider following ``attach`` (a transform / joint, may be
    None = world).  Returns the collider's long name."""
    if kind not in KINDS:
        raise SimflowError("不明な当たりの種類です: %r" % kind)
    if attach and not cmds.objExists(attach):
        raise SimflowError("付けるノードが見つかりません: %s" % attach)
    if attach and scene_manager.is_tool_node(attach):
        raise SimflowError("SIMFLOW のノードには当たりを付けられません。リグのジョイントやコントローラーを選んでください。")
    base = name or "%s_%s" % (short_name(attach).split(":")[-1] if attach else "world", kind)
    root = _root()
    follow = cmds.createNode("transform", name=base + "_follow", parent=root, skipSelect=True)
    follow = cmds.ls(follow, long=True)[0]
    scene_manager.tag(follow, "colliderFollow")
    if attach:
        for fn in (cmds.parentConstraint, cmds.scaleConstraint):
            con = fn(attach, follow, maintainOffset=False)[0]
            scene_manager.tag(con, "colliderConstraint")
        add_string_attr(follow, "simflowAttach", cmds.ls(attach, long=True)[0])
    col = cmds.createNode("transform", name=base, parent=follow, skipSelect=True)
    col = cmds.ls(col, long=True)[0]
    scene_manager.tag(col, "collider")
    add_string_attr(col, "simflowKind", kind)
    if kind == "floor":
        # an infinite plane through the origin, local Y up; "size" is only drawn
        cmds.addAttr(col, longName="size", niceName="表示の大きさ", attributeType="doubleLinear", min=0.01,
                     defaultValue=200.0, keyable=True)
        if radius is not None:
            cmds.setAttr(col + ".size", radius)
        for a in ("sx", "sy", "sz"):
            cmds.setAttr("%s.%s" % (col, a), keyable=False, channelBox=False)
        shape, node = _mesh_under(col, lambda name: cmds.polyPlane(name=name, width=1, height=1, subdivisionsX=10,
                                                                     subdivisionsY=10), base + "_shape")
        cmds.connectAttr(col + ".size", node + ".width")
        cmds.connectAttr(col + ".size", node + ".height")
        cmds.select(col, replace=True)
        LOG.info("当たり（床）を作りました: %s%s。床の Y 軸が上向きで、どこまでも広がる平面として扱います", short_name(col),
                 "（%s に付いて動きます）" % short_name(attach) if attach else "")
        return col
    cmds.addAttr(col, longName="radius", niceName="半径", attributeType="doubleLinear", min=0.01,
                 defaultValue=1.0, keyable=True)
    cmds.setAttr(col + ".radius", radius if radius is not None else (5.0 if kind == "sphere" else 3.0))
    for a in ("sx", "sy", "sz"):
        cmds.setAttr("%s.%s" % (col, a), keyable=False, channelBox=False)
    if kind == "sphere":
        shape, node = _mesh_under(col, lambda name: cmds.polySphere(name=name, radius=1, subdivisionsAxis=16,
                                                                      subdivisionsHeight=10), base + "_shape")
        cmds.connectAttr(col + ".radius", node + ".radius")
    else:
        cmds.addAttr(col, longName="length", niceName="長さ", attributeType="doubleLinear", min=0.0,
                     defaultValue=10.0, keyable=True)
        cmds.setAttr(col + ".length", length if length is not None else 10.0)
        shape, cyl = _mesh_under(col, lambda name: cmds.polyCylinder(name=name, radius=1, height=1,
                                                                       subdivisionsAxis=16, subdivisionsCaps=0,
                                                                       subdivisionsHeight=1), base + "_body")
        cmds.connectAttr(col + ".radius", cyl + ".radius")
        cmds.connectAttr(col + ".length", cyl + ".height")
        half = cmds.createNode("multDoubleLinear", name=base + "_half", skipSelect=True)
        scene_manager.tag(half, "colliderShape")
        cmds.setAttr(half + ".input2", 0.5)
        cmds.connectAttr(col + ".length", half + ".input1")
        neg = cmds.createNode("multDoubleLinear", name=base + "_halfNeg", skipSelect=True)
        scene_manager.tag(neg, "colliderShape")
        cmds.setAttr(neg + ".input2", -0.5)
        cmds.connectAttr(col + ".length", neg + ".input1")
        for end, src in (("top", half), ("bottom", neg)):
            cap = cmds.createNode("transform", name="%s_%s" % (base, end), parent=col, skipSelect=True)
            cap = cmds.ls(cap, long=True)[0]
            scene_manager.tag(cap, "colliderShape")
            cmds.connectAttr(src + ".output", cap + ".ty")
            cshape, sph = _mesh_under(cap, lambda name: cmds.polySphere(name=name, radius=1, subdivisionsAxis=16,
                                                                          subdivisionsHeight=10), "%s_%sCap" % (base, end))
            cmds.connectAttr(col + ".radius", sph + ".radius")
            # the caps are only drawn; clicking the body selects the collider
            cmds.setAttr(cshape + ".overrideDisplayType", 2)
    cmds.select(col, replace=True)
    LOG.info("当たり（%s）を作りました: %s%s", KIND_NAMES[kind], short_name(col),
             "（%s に付いて動きます）" % short_name(attach) if attach else "")
    return col


def colliders():
    """Long names of all colliders in the scene, in name order.  (The order
    matters a little: overlapping colliders - a knee - push one after the
    other.  Maya's listing order is not stable (it differed between a scene
    and the same set-up rebuilt), so the calculation always takes them by name.)"""
    nodes = [n for n in scene_manager.tool_nodes("collider") if cmds.nodeType(n) == "transform"]
    return sorted(nodes, key=lambda n: (short_name(n), n))


def info(col):
    follow = (cmds.listRelatives(col, parent=True, fullPath=True) or [None])[0]
    attach = get_string_attr(follow, "simflowAttach") if follow else None
    kind = get_string_attr(col, "simflowKind") or "sphere"
    return {"node": col, "name": short_name(col), "kind": kind,
            "attach": short_name(attach) if attach else "",
            "radius": cmds.getAttr(col + ".radius") if kind != "floor" else 0.0,
            "length": cmds.getAttr(col + ".length") if kind == "capsule" else 0.0}


def delete_collider(col):
    if not cmds.objExists(col) or get_string_attr(col, scene_manager.ROLE_ATTR) != "collider":
        raise SimflowError("当たりではありません: %s" % col)
    follow = (cmds.listRelatives(col, parent=True, fullPath=True) or [None])[0]
    victims = []
    for node in [follow] + (cmds.listHistory(col, allConnections=True) or []):
        if node and cmds.objExists(node) and get_string_attr(node, scene_manager.ROLE_ATTR) == "colliderShape":
            victims.append(node)
    if follow:
        cmds.delete(follow)
    for v in victims:
        if cmds.objExists(v):
            cmds.delete(v)
    root = scene_manager.tool_nodes("colliderRoot")
    if root and not cmds.listRelatives(root[0], children=True):
        for attr in ("t", "r", "s"):
            cmds.setAttr("%s.%s" % (root[0], attr), lock=False)
        cmds.delete(root[0])


# ---------------------------------------------------------------------------
# sampling
# ---------------------------------------------------------------------------
class ColliderSampler(object):
    """Reads one collider in the CURRENT DG context (see sample_many)."""

    def __init__(self, col):
        self.node = col
        fn = om.MFnDependencyNode(get_mobject(col))
        self._world = fn.findPlug("worldMatrix", False).elementByLogicalIndex(0)
        self.kind = get_string_attr(col, "simflowKind") or "sphere"
        self._radius = fn.findPlug("radius", False) if self.kind != "floor" else None
        self._length = fn.findPlug("length", False) if self.kind == "capsule" else None

    def read(self, frame=None):
        m = list(om.MFnMatrixData(self._world.asMObject()).matrix())
        xs = [m[0:3], m[4:7], m[8:11]]
        sx, sy, sz = (math.sqrt(sum(v * v for v in row)) for row in xs)
        center = (m[12], m[13], m[14])
        axis = tuple(v / sy for v in xs[1]) if sy > 1.0e-9 else (0.0, 1.0, 0.0)
        if self.kind == "floor":
            return (center, axis, 0.0, 0.0, "floor")
        r = self._radius.asDouble()
        top = max(sx, sy, sz)
        if top > 1.0e-9 and top - min(sx, sy, sz) > NONUNIFORM * top:
            # stretched: the drawn ellipsoid / capsule, not a sphere of the mean radius
            from ..solvers import collision as colmod
            return colmod.ellipsoid(center, (tuple(xs[0]), tuple(xs[1]), tuple(xs[2])), r,
                                    0.5 * self._length.asDouble() if self.kind == "capsule" else 0.0)
        if self.kind == "capsule":
            half = 0.5 * self._length.asDouble() * sy
            radius = r * 0.5 * (sx + sz)
        else:
            half = 0.0
            radius = r * (sx + sy + sz) / 3.0
        return (center, axis, half, radius)


def signature(frames_data):
    """Short hash of sampled collider data (for the calculation cache)."""
    h = hashlib.sha1()
    for cols in frames_data or ():
        for col in cols:
            c, a, half, r = col[:4]
            h.update(("%.5g,%.5g,%.5g|%.4g,%.4g,%.4g|%.5g|%.5g;" % (tuple(c) + tuple(a) + (half, r))).encode())
            if len(col) > 4:
                h.update(("%r;" % (col[4:],)).encode())
    return h.hexdigest()
