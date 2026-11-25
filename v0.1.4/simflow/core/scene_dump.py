# -*- coding: utf-8 -*-
"""検証用のデータを書き出す: the motion and the SIMFLOW set-up of a scene, with
every name replaced, so the developer can rebuild the same calculation without
the rig (``verify/chain_replay.py``).

What is written (format "SIMFLOW verify data", version 2)::

    frames / units / bake range and pre-roll
    labels, nodes      every registered controller as c1, c2 ... : its world
                       and parent world matrix per frame, rotate order, the
                       nearest REGISTERED ancestor (or none) and how many
                       unregistered transforms are in between, joint / keyed /
                       constrained flags
    groups             the checked 揺れもの (and the ones pushing them out) as g1, g2 ...: its controllers (labels),
                       chains, all settings, per-chain values (by root label),
                       押し出す揺れもの (by label), the chosen one ("current")
    colliders          kind, size, world matrix per frame, the controller it
                       follows when that is a registered one (label)
    cameras            the カメラ合わせ camera as cam1 ...: world matrix per
                       frame and lens (only when used)
    result             (version 3) what the scene shows NOW with the SIMFLOW
                       layers as they are (「リグで見る」 / bake): per
                       controller label the world matrix per frame; only when
                       a SIMFLOW layer is in the scene.  The animation itself
                       is ``nodes[...]["world"]`` (layers muted).
    result_calc        (version 3) the last calculation SIMFLOW holds (what the
                       preview shows): per controller label {"frames", "quat"
                       (world orientation x, y, z, w)}; only when calculated.
    simflow_version    the tool's version that wrote the file

Not written: node / group / collider / camera names, namespaces, the scene's
file name, the user name.  The scene is not changed (the values are read per
frame; the SIMFLOW layers are muted while reading and restored)."""
from __future__ import annotations

import json
import os

FORMAT = "SIMFLOW verify data"
VERSION = 3


def default_path():
    import maya.cmds as cmds
    scene = cmds.file(query=True, sceneName=True) or ""
    folder = os.path.dirname(scene) if scene else os.path.expanduser("~")
    return os.path.join(folder, "simflow_verify.json")


def chosen_groups(session):
    """The checked groups, plus the groups that push them out (押し出す揺れもの,
    also when those are not checked: the calculation needs them).  In the
    panel's order.  Returns (groups, names of the unchecked ones added)."""
    groups = session.controllers.groups
    by_id = {g.id: g for g in groups}
    want = {g.id for g in groups if g.enabled}
    todo = list(want)
    while todo:
        g = by_id[todo.pop()]
        for other in g.settings.collide_with if g.settings.collision else ():
            if other in by_id and other not in want:
                want.add(other)
                todo.append(other)
    added = [g.name for g in groups if g.id in want and not g.enabled]
    return [g for g in groups if g.id in want], added


def dump(session, path, groups=None, current=None):
    """Write the verify data of ``groups`` (SwingGroups; default: the checked
    ones and the ones pushing them, see chosen_groups) to ``path``.  Returns a
    short summary (counts)."""
    import maya.cmds as cmds
    from . import collider_manager, scene_manager
    from .simulation_manager import SimulationManager
    from ..utils.maya_utils import get_string_attr, SimflowError
    added = []
    if groups is None:
        groups, added = chosen_groups(session)
    groups = list(groups)
    if not groups:
        raise SimflowError("チェックの付いた揺れものがありません。書き出したい揺れものにチェックを付けてください。")
    g_label = {g.id: "g%d" % (i + 1) for i, g in enumerate(groups)}
    nodes, owner = [], []
    entries_of = {}
    for g in groups:
        entries = session.controllers.group_entries(g.id)
        entries_of[g.id] = entries
        for e in entries:
            nodes.append(session.controllers.resolve(e))
            owner.append(g.id)
    labels = ["c%d" % (i + 1) for i in range(len(nodes))]
    lab_of = dict(zip(nodes, labels))
    lab_of_entry = {}
    k = 0
    for g in groups:
        for e in entries_of[g.id]:
            lab_of_entry[e.id] = labels[k]
            k += 1
    bo = session.bake_options
    start, end = int(round(bo.start)), int(round(bo.end))
    pre = int(bo.preroll or 0)
    frames = list(range(start - pre, end + 1))
    cams = {}
    out = {"format": FORMAT, "version": VERSION,
           "time_unit": cmds.currentUnit(query=True, time=True),
           "linear_unit": cmds.currentUnit(query=True, linear=True),
           "frames": frames, "preroll": pre, "preroll_mode": bo.preroll_mode,
           "labels": labels, "nodes": {}, "groups": [], "colliders": [], "cameras": {}}
    for g in groups:
        entries = entries_of[g.id]
        try:
            chains = SimulationManager.split_chains(entries, {e.id: session.controllers.resolve(e) for e in entries})
            chains = [[lab_of_entry[e.id] for e in c] for c in chains]
        except Exception as exc:
            chains = "error: %s" % exc
        s = g.settings.to_dict()
        s["collide_with"] = [g_label[x] for x in g.settings.collide_with if x in g_label]
        cam = g.settings.warp_camera
        if cam:
            hits = cmds.ls(cam, long=True) or []
            if hits:
                cams.setdefault(hits[0], "cam%d" % (len(cams) + 1))
                s["warp_camera"] = cams[hits[0]]
            else:
                s["warp_camera"] = ""
        out["groups"].append({
            "label": g_label[g.id], "enabled": bool(g.enabled), "chain": bool(g.chain),
            "chain_auto": bool(g.chain_auto), "rope": bool(g.rope), "settings": s,
            "controllers": [lab_of_entry[e.id] for e in entries], "chains": chains,
            "chain_values": [{"root": lab_of_entry[rid], "values": dict(v)}
                             for rid, v in g.chain_overrides.items() if rid in lab_of_entry and v],
        })
    if current is not None and current.id in g_label:
        out["current"] = g_label[current.id]
    else:
        out["current"] = "g1"
    with scene_manager.tool_layers_muted():
        for lab, node in zip(labels, nodes):
            anc = None
            for other in nodes:
                if other != node and node.startswith(other + "|") and (anc is None or len(other) > len(anc)):
                    anc = other
            between = node[len(anc):].count("|") - 1 if anc is not None else 0
            info = {"registered_parent": lab_of.get(anc), "unregistered_between": between,
                    "depth": node.count("|"), "is_joint": cmds.nodeType(node) == "joint",
                    "rotate_order": cmds.getAttr(node + ".rotateOrder"),
                    "constrained": bool(cmds.listConnections(node, type="constraint", source=True, destination=False)),
                    "animated": {a: bool(cmds.listConnections("%s.%s" % (node, a), source=True, destination=False))
                                 for a in ("tx", "ty", "tz", "rx", "ry", "rz")},
                    "group": g_label[owner[labels.index(lab)]],
                    "world": [list(cmds.getAttr(node + ".worldMatrix[0]", time=f)) for f in frames],
                    "parent_world": [list(cmds.getAttr(node + ".parentMatrix[0]", time=f)) for f in frames],
                    "rotate_pivot_local": list(cmds.getAttr(node + ".rotatePivot")[0])}
            out["nodes"][lab] = info
        for c in collider_manager.colliders():
            ci = collider_manager.info(c)
            follow = (cmds.listRelatives(c, parent=True, fullPath=True) or [None])[0]
            attach = get_string_attr(follow, "simflowAttach") if follow else None
            attach = (cmds.ls(attach, long=True) or [None])[0] if attach else None
            d = {"kind": ci["kind"], "radius": ci["radius"], "length": ci["length"],
                 "attach": lab_of.get(attach, "") if attach else "",
                 "world": [list(cmds.getAttr(c + ".worldMatrix[0]", time=f)) for f in frames]}
            if ci["kind"] == "floor":
                d["size"] = cmds.getAttr(c + ".size")
            out["colliders"].append(d)
        for node, lab in cams.items():
            shape = (cmds.listRelatives(node, shapes=True, fullPath=True, type="camera") or [node])[0]
            out["cameras"][lab] = {
                "world": [list(cmds.getAttr(node + ".worldMatrix[0]", time=f)) for f in frames],
                "focal": [cmds.getAttr(shape + ".focalLength", time=f) for f in frames],
                "hfa": cmds.getAttr(shape + ".horizontalFilmAperture"),
                "vfa": cmds.getAttr(shape + ".verticalFilmAperture"),
                "fit": cmds.getAttr(shape + ".filmFit"),
                "resolution": [cmds.getAttr("defaultResolution.width"), cmds.getAttr("defaultResolution.height")]}
    # what the user sees: the scene with the SIMFLOW layers as they are, and
    # the calculation SIMFLOW holds (the preview)
    from .. import __version__ as _v
    out["simflow_version"] = _v
    layers = scene_manager.tool_layers() + [n for n in scene_manager.tool_nodes("previewLayer")
                                            if cmds.nodeType(n) == "animLayer"]
    if layers:
        out["result"] = {lab: [list(cmds.getAttr(node + ".worldMatrix[0]", time=f)) for f in frames]
                         for lab, node in zip(labels, nodes)}
    calc = {}
    cache = getattr(session.sim, "cache", None) or {}
    for g in groups:
        for e in entries_of[g.id]:
            r = cache.get(e.id)
            frs = getattr(r, "frame_results", None) if r is not None else None
            if frs:
                calc[lab_of_entry[e.id]] = {"frames": [x.frame for x in frs], "quat": [list(x.quat) for x in frs]}
    if calc:
        out["result_calc"] = calc
    folder = os.path.dirname(path)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(out, fh)
    return {"path": path, "groups": len(groups), "controllers": len(nodes), "frames": len(frames),
            "colliders": len(out["colliders"]), "cameras": len(cams), "added": added}


def summary_text(info):
    return ("検証用のデータを書き出しました: %s（揺れもの %d 個、コントローラー %d 個、%d フレーム、当たり %d 個%s。"
            "名前はすべて番号に置き換えています）"
            % (info["path"], info["groups"], info["controllers"], info["frames"], info["colliders"],
               "、カメラ %d 個" % info["cameras"] if info["cameras"] else "")
            + ("。チェックの付いた揺れものを押し出す%sも入れました" % "、".join("「%s」" % n for n in info["added"])
               if info.get("added") else ""))
