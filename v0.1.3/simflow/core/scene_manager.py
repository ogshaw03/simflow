# -*- coding: utf-8 -*-
"""Tool-owned scene nodes and persistent metadata (spec 6 / 11).

Every node SIMFLOW creates carries::

    simflowOwner = "SIMFLOW"
    simflowRole  = "root" | "group" | "metadata" | "inputProxy" | "outputProxy"
                   | "layer" | "backup" | "previewLayer"
    simflowId    = "<entry id>" (or "" for shared nodes)

Clean-up only ever deletes nodes that carry this tag, so user / other tool
nodes are never touched.  Settings live as JSON in the ``simflowData``
attribute of the ``SIMFLOW_METADATA`` network node and are saved with the
scene.
"""
from __future__ import annotations

import contextlib
import json

import maya.cmds as cmds

from ..utils.logger import get_logger
from ..utils.maya_utils import add_string_attr, get_string_attr

LOG = get_logger("scene")

OWNER = "SIMFLOW"
OWNER_ATTR = "simflowOwner"
ROLE_ATTR = "simflowRole"
ID_ATTR = "simflowId"
DATA_ATTR = "simflowData"
DATA_VERSION = 1

ROOT_NAME = "SIMFLOW_ROOT"
GROUPS = ("SIMFLOW_INPUT", "SIMFLOW_SIMULATION", "SIMFLOW_OUTPUT")
META_NAME = "SIMFLOW_METADATA"


def tag(node, role, entry_id=""):
    add_string_attr(node, OWNER_ATTR, OWNER)
    add_string_attr(node, ROLE_ATTR, role)
    add_string_attr(node, ID_ATTR, entry_id)


def is_tool_node(node):
    return get_string_attr(node, OWNER_ATTR) == OWNER


def _plug_value(node, attr):
    """The string attribute ``attr`` of ``node`` (a name ls gave), or None.
    getAttr straight away (no objExists / attributeQuery: those made a
    preview of 52 controllers spend 6 s here)."""
    try:
        return cmds.getAttr("%s.%s" % (node, attr))
    except (ValueError, RuntimeError):
        return None


def tool_nodes(role=None, entry_id=None, entry_ids=None):
    """All nodes owned by SIMFLOW (long names), optionally filtered by role and
    by one entry id (``entry_id``) or a set of them (``entry_ids``)."""
    out = []
    for plug in cmds.ls("*.%s" % OWNER_ATTR, recursive=True) or []:
        node = plug.rsplit(".", 1)[0]
        if _plug_value(node, OWNER_ATTR) != OWNER:
            continue
        if role is not None and _plug_value(node, ROLE_ATTR) != role:
            continue
        if entry_id is not None or entry_ids is not None:
            i = _plug_value(node, ID_ATTR)
            if (entry_id is not None and i != entry_id) or (entry_ids is not None and i not in entry_ids):
                continue
        nodes = cmds.ls(node, long=True) or [node]
        out.append(nodes[0])
    return out


_SINGLE = {}      # role -> long name of the node last found (checked before it is used)


def _find_single(role):
    node = _SINGLE.get(role)
    if node and cmds.objExists(node) and _plug_value(node, OWNER_ATTR) == OWNER \
            and _plug_value(node, ROLE_ATTR) == role:
        return node
    nodes = tool_nodes(role)
    _SINGLE[role] = nodes[0] if nodes else None
    return _SINGLE[role]


# ---------------------------------------------------------------------------
# Root / groups
# ---------------------------------------------------------------------------

def get_root(create=True):
    root = _find_single("root")
    if root or not create:
        return root
    root = cmds.createNode("transform", name=ROOT_NAME, skipSelect=True)
    root = cmds.ls(root, long=True)[0]
    tag(root, "root")
    for name in GROUPS:
        grp = cmds.createNode("transform", name=name, parent=root, skipSelect=True)
        tag(grp, "group:%s" % name)
    for attr in ("t", "r", "s"):
        cmds.setAttr("%s.%s" % (root, attr), lock=True)
    return root


def get_group(name):
    root = get_root(create=True)
    for child in cmds.listRelatives(root, children=True, fullPath=True, type="transform") or []:
        if get_string_attr(child, ROLE_ATTR) == "group:%s" % name:
            return child
    grp = cmds.createNode("transform", name=name, parent=root, skipSelect=True)
    grp = cmds.ls(grp, long=True)[0]
    tag(grp, "group:%s" % name)
    return grp


# ---------------------------------------------------------------------------
# Metadata
# ---------------------------------------------------------------------------

def get_metadata_node(create=True):
    node = _find_single("metadata")
    if node or not create:
        return node
    node = cmds.createNode("network", name=META_NAME, skipSelect=True)
    tag(node, "metadata")
    add_string_attr(node, DATA_ATTR, json.dumps({"version": DATA_VERSION}))
    return node


def load_data():
    node = get_metadata_node(create=False)
    if not node:
        return None
    raw = get_string_attr(node, DATA_ATTR) or "{}"
    try:
        data = json.loads(raw)
    except ValueError:
        LOG.warning("SIMFLOW のメタデータが壊れているため無視しました")
        return None
    if data.get("version", 0) > DATA_VERSION:
        LOG.warning("SIMFLOW のメタデータは新しいバージョンで書かれています")
    return data


def save_data(data):
    data = dict(data)
    data["version"] = DATA_VERSION
    node = get_metadata_node(create=True)
    add_string_attr(node, DATA_ATTR, json.dumps(data, sort_keys=True))


# ---------------------------------------------------------------------------
# Animation layers owned by SIMFLOW
# ---------------------------------------------------------------------------

def tool_layers():
    return [n for n in tool_nodes("layer") if cmds.nodeType(n) == "animLayer"]


@contextlib.contextmanager
def tool_layers_muted(bakes=True, preview=True):
    """Temporarily mute SIMFLOW bake layers so sampling sees the ORIGINAL
    animation (needed for re-calculation / re-bake).  ``bakes=False`` keeps
    the bake layers on (only the preview muted: the scene as it stays)."""
    states = []
    layers = ((tool_layers() if bakes else []) +
              ([n for n in tool_nodes("previewLayer") if cmds.nodeType(n) == "animLayer"] if preview else []))
    for layer in layers:
        try:
            states.append((layer, cmds.animLayer(layer, query=True, mute=True)))
            cmds.animLayer(layer, edit=True, mute=True)
        except RuntimeError:
            pass
    dirty_layers([l for l, _ in states])
    try:
        yield
    finally:
        for layer, muted in states:
            if cmds.objExists(layer):
                cmds.animLayer(layer, edit=True, mute=muted)
        dirty_layers([l for l, _ in states])


def raise_layer(layer):
    """Put a SIMFLOW layer above its sibling layers.

    The keys are solved against everything below: the base animation and the
    user's layers (sampled with only the SIMFLOW layers muted).  A user layer
    ABOVE would be applied after them - an override layer replaces the
    result, and an additive one in quaternion mode («レイヤー» rotation
    accumulation) does not commute with the per-channel delta - so the bake
    would not show the calculated result (seen: 11-14 degrees).  Returns
    True when the layer was moved."""
    try:
        parent = cmds.animLayer(layer, query=True, parent=True)
        siblings = cmds.animLayer(parent, query=True, children=True) or [] if parent else []
    except RuntimeError:
        return False
    if not siblings or siblings[-1] == layer:
        return False
    cmds.animLayer(layer, edit=True, moveLayerAfter=siblings[-1])
    return True


def dirty_layers(layers):
    """Muting a layer or keying a new one does not always dirty the
    controllers at the CURRENT frame (seen with override layers on joints:
    the old value stayed cached there while every other frame was right).
    Dirtying the layer node itself propagates correctly."""
    layers = [l for l in layers if cmds.objExists(l)]
    if not layers:
        return
    # the layer node fixes the CURRENT frame; time-context evaluations
    # (getAttr -time, om2 DG contexts used by the sampler / validation) keep a
    # stale value until the layer's blend nodes are dirtied too
    blends = []
    for l in layers:
        try:
            blends += cmds.animLayer(l, query=True, blendNodes=True) or []
        except RuntimeError:
            pass
    cmds.dgdirty(layers + blends)


# ---------------------------------------------------------------------------
# Clean up
# ---------------------------------------------------------------------------

def delete_nodes(nodes):
    nodes = [n for n in nodes if cmds.objExists(n) and is_tool_node(n)]
    for node in nodes:
        if not cmds.objExists(node):
            continue
        if cmds.lockNode(node, query=True, lock=True)[0]:
            cmds.lockNode(node, lock=False)
        cmds.delete(node)
    return len(nodes)


def cleanup(include_layers=False, include_backups=False):
    """Delete SIMFLOW proxies / groups / metadata.  Bake layers and overwrite
    backups are only removed when explicitly requested."""
    roles_kept = set()
    if not include_layers:
        roles_kept.add("layer")
    if not include_backups:
        roles_kept.add("backup")
    victims = []
    for node in tool_nodes():
        role = get_string_attr(node, ROLE_ATTR)
        if role in roles_kept or (role or "").startswith("collider"):
            # colliders are set-up (like the rig), not results: keep them
            continue
        if role == "metadata" and (not include_layers or not include_backups):
            # metadata still describes kept layers / backups -> keep it so
            # Restore Original keeps working.
            if tool_layers() and not include_layers:
                continue
            if tool_nodes("backup") and not include_backups:
                continue
        victims.append(node)
    # delete root last (its children are proxies)
    victims.sort(key=lambda n: (get_string_attr(n, ROLE_ATTR) == "root", -n.count("|")))
    count = 0
    for node in victims:
        if cmds.objExists(node):
            count += delete_nodes([node])
    LOG.info("Clean Up removed %d SIMFLOW node(s)", count)
    return count
