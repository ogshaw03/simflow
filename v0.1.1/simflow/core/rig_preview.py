# -*- coding: utf-8 -*-
"""Preview on the real rig ("リグで見る").

The simulated result is written into one SIMFLOW-owned *override* animation
layer (``SIMFLOW_Preview``) so the actual controllers - and with them the
skinned meshes - move before anything is baked.

Why a layer and not constraints: constraining channels that already carry
animation inserts pairBlend nodes and changes the rig's connections; a layer
leaves the original curves and connections untouched and is removed by
deleting one node.

* The layer is an override layer with the final channel values (the same
  values the override bake writes and validates), so it shows exactly the
  simulation regardless of SIMFLOW bake layers below it.  Its weight is keyed
  0 / 1 / 1 / 0 around the range, so frames outside pass the original through.
* Sampling mutes it (``scene_manager.tool_layers_muted``), so the simulation
  always reads the original animation.
* Baking deletes it first (the bake then shows the same result).
* Channels that cannot be keyed safely (locked, constraints, expressions ...)
  are skipped and reported.
"""
from __future__ import annotations

import maya.cmds as cmds

from ..utils.maya_utils import add_string_attr, get_string_attr
from . import scene_manager

ROLE = "previewLayer"
LAYER_NAME = "SIMFLOW_Preview"


def layer():
    nodes = [n for n in scene_manager.tool_nodes(ROLE) if cmds.nodeType(n) == "animLayer"]
    return nodes[0] if nodes else None


def remove():
    """Delete the preview layer.

    The channels are taken out of the layer FIRST: deleting a layer node that
    still holds a channel without animation curves (a static value) leaves
    the channel at the layer's last evaluated value instead of its own value
    (seen in Maya 2025: the original pose was lost).  ``removeAttribute``
    gives the static value back - the same way Restore Original handles the
    bake layers."""
    node = layer()
    if not node:
        return False
    attrs = cmds.animLayer(node, query=True, attribute=True) or []
    if attrs:
        cmds.animLayer(node, edit=True, removeAttribute=attrs)
    scene_manager.delete_nodes([node])
    return True


def _create():
    name = LAYER_NAME
    if cmds.objExists(name):            # a user node with that name: pick a free one
        name = LAYER_NAME + "#"
    node = cmds.animLayer(name, override=True)
    scene_manager.tag(node, ROLE)
    return node


def update(batches, results, nodes, options, progress=None):
    """Rebuild the preview layer from ``results``.

    ``nodes``: entry id -> controller path.  Returns a list of skipped
    channels / controllers (messages)."""
    from .bake_manager import check_channels, enabled_attrs, key_layer_curves
    # Only the calculated groups are replaced: the controllers of groups that
    # were previewed before and are not calculated now (check box off) keep
    # their result.  A different range rebuilds everything (the layer weight
    # is keyed for one range).
    old = layer()
    rng = "%g:%g" % (options.start, options.end)
    if old and get_string_attr(old, "simflowRange") == rng:
        remove_nodes([nodes[e.id] for b in batches for e in b.entries if e.id in nodes])
    else:
        remove()
    work = []
    skipped = []
    for b in batches:
        attrs = enabled_attrs(b.settings)
        for e in b.entries:
            res = results.get(e.id)
            node = nodes.get(e.id)
            if res is None or node is None or not attrs:
                continue
            problems = check_channels(node, attrs, "override")
            if problems:
                skipped.extend(problems)
                continue
            work.append((node, attrs, res))
    if not work:
        return skipped
    frames = options.frames()
    lay = layer() or _create()
    for k, (node, attrs, res) in enumerate(work):
        if progress is not None:
            progress.update(float(k) / len(work), "%s（%d / %d）" % (node.split("|")[-1], k + 1, len(work)))
        by_frame = res.by_frame()
        frs = [by_frame[f] for f in frames]
        key_layer_curves(node, attrs, frs, options, lay, additive=False)
    plug = lay + ".weight"
    for f, v in ((options.start - 1.0, 0.0), (options.start, 1.0),
                 (options.end, 1.0), (options.end + 1.0, 0.0)):
        cmds.setKeyframe(plug, time=f, value=v, outTangentType="step")
    add_string_attr(lay, "simflowRange", "%g:%g" % (options.start, options.end))
    scene_manager.dirty_layers([lay])
    return skipped


def remove_nodes(node_paths):
    """Take controllers out of the preview layer (unregistered / removed)."""
    lay = layer()
    if not lay:
        return
    current = cmds.animLayer(lay, query=True, attribute=True) or []
    wanted = set()
    for p in node_paths:
        wanted.update(cmds.ls(p, long=True) or [])
    drop = [p for p in current if (cmds.ls(p.rsplit(".", 1)[0], long=True) or [""])[0] in wanted]
    if drop:
        cmds.animLayer(lay, edit=True, removeAttribute=drop)
    if not (cmds.animLayer(lay, query=True, attribute=True) or []):
        scene_manager.delete_nodes([lay])
