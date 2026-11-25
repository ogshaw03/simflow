# -*- coding: utf-8 -*-
"""Controller registration (spec 3.1) and swing groups ("揺れもの").

A *group* is one swinging thing (hair, tail, an earring ...): a name, its own
simulation settings and whether its controllers form a chain.  Every
registered controller belongs to exactly one group; the order of a group's
controllers is the chain order (root first).

Controllers are identified by Maya UUID first and long path second so that
same-named objects are distinguished and renames / reference reloads are
detected.
"""
from __future__ import annotations

import uuid as _uuid
from dataclasses import dataclass, field
from typing import List, Optional

import maya.cmds as cmds

from ..utils.logger import get_logger
from ..utils.maya_utils import SimflowError, long_name, node_from_uuid, node_uuid, short_name
from . import scene_manager
from .settings import SimSettings

LOG = get_logger("controllers")


def _new_id():
    return _uuid.uuid4().hex[:12]


# ---------------------------------------------------------------------------
# "＋ 新規": read the chain from the hierarchy below the selected root
# ---------------------------------------------------------------------------
CONTROL_SHAPES = ("nurbsCurve", "locator")


def is_control(node):
    """A transform / joint with a visible curve (or locator) shape: what an
    animator picks in the viewport.  Offset groups and plain joints are not."""
    for s in cmds.listRelatives(node, shapes=True, fullPath=True, noIntermediate=True) or []:
        if cmds.nodeType(s) in CONTROL_SHAPES:
            return True
    return False


def is_fk_name(node):
    """'fk' anywhere in the short name (namespace off), any case."""
    return "fk" in node.split("|")[-1].split(":")[-1].lower()


def fk_descendants(root):
    """The FK controllers below ``root``, depth first in the outliner's order
    (root -> tip of each chain, chain after chain).  The hierarchy may hold an
    IK setup (IK controllers, ikHandles, IK / FK joints, offset groups): only
    controllers with 'fk' in their name are read; the walk goes on through
    everything else.  Returns (fk controllers, skipped non-fk controllers)."""
    found, skipped = [], []
    stack = list(reversed(cmds.listRelatives(root, children=True, fullPath=True, type="transform") or []))
    while stack:
        node = stack.pop()
        if scene_manager.is_tool_node(node):
            continue
        if is_control(node):
            (found if is_fk_name(node) else skipped).append(node)
        stack.extend(reversed(cmds.listRelatives(node, children=True, fullPath=True, type="transform") or []))
    return found, skipped


def expand_fk_hierarchy(selection):
    """The selected nodes plus the FK controllers below each, in order and
    without duplicates.  A selected node that is not a controller (a group)
    is left out when FK controllers were found below it.  Returns (nodes,
    messages)."""
    out, seen, info = [], set(), []
    n_read, skipped = 0, []
    for node in selection:
        below, skip = fk_descendants(node)
        skipped.extend(s for s in skip if s not in skipped)
        keep = [node] if (is_control(node) or not below) else []
        for n in keep + below:
            if n not in seen:
                seen.add(n)
                out.append(n)
                n_read += n is not node
    skipped = [s for s in skipped if s not in seen]      # selected by hand: registered
    if n_read:
        info.append("階層から fk のコントローラーを %d 個読みました" % n_read)
    if skipped:
        names = [short_name(s) for s in skipped]
        info.append("fk の付かないコントローラー %d 個は読みませんでした（%s%s）"
                    % (len(names), "、".join(names[:5]), " ほか" if len(names) > 5 else ""))
    return out, info


@dataclass
class SwingGroup:
    name: str
    id: str = field(default_factory=_new_id)
    enabled: bool = True
    chain: bool = True                   # chain (root -> tip) / each controller on its own
    chain_auto: bool = True              # chain: length / direction from the next controller
    settings: SimSettings = field(default_factory=SimSettings)
    # "縄・ひも（両端を持つ）": the controllers along a rope, end to end; the
    # first and the last are the grips (follow the hands, not changed)
    rope: bool = False
    # "チェーンごとの値": chain (its root controller's entry id) -> {field: value}
    # put over ``settings`` for that chain only (see SimSettings.CHAIN_FIELDS)
    chain_overrides: dict = field(default_factory=dict)

    def to_dict(self):
        return {"id": self.id, "name": self.name, "enabled": self.enabled, "chain": self.chain,
                "chain_auto": self.chain_auto, "rope": self.rope, "settings": self.settings.to_dict(),
                "chain_overrides": {k: dict(v) for k, v in self.chain_overrides.items() if v}}

    @classmethod
    def from_dict(cls, d):
        return cls(name=d.get("name") or "揺れもの", id=d.get("id") or _new_id(),
                   enabled=d.get("enabled", True), chain=d.get("chain", True),
                   chain_auto=d.get("chain_auto", True), settings=SimSettings.from_dict(d.get("settings")),
                   rope=bool(d.get("rope", False)),
                   chain_overrides={k: {n: float(x) for n, x in v.items()}
                                    for k, v in (d.get("chain_overrides") or {}).items() if v})


@dataclass
class ControllerEntry:
    path: str
    uuid: str
    id: str = field(default_factory=_new_id)
    group: str = ""                      # SwingGroup.id
    enabled: bool = True
    rest: Optional[dict] = None          # {"frame": f, "ch": {...}} internal units
    bake: Optional[dict] = None          # bake state (mode, layer, range, attrs, backups)

    @property
    def label(self):
        return short_name(self.path)

    def to_dict(self):
        return {"id": self.id, "path": self.path, "uuid": self.uuid, "group": self.group,
                "enabled": self.enabled, "rest": self.rest, "bake": self.bake}

    @classmethod
    def from_dict(cls, d):
        return cls(path=d["path"], uuid=d.get("uuid", ""), id=d.get("id") or _new_id(),
                   group=d.get("group", ""), enabled=True, rest=d.get("rest"),
                   bake=d.get("bake"))


class ControllerManager(object):

    def __init__(self):
        self.groups: List[SwingGroup] = []
        self.entries: List[ControllerEntry] = []

    # ------------------------------------------------------------------
    # groups
    # ------------------------------------------------------------------
    def add_group(self, name=None, chain=True, settings=None):
        if not name:
            n = len(self.groups) + 1
            names = {g.name for g in self.groups}
            while "揺れもの %d" % n in names:
                n += 1
            name = "揺れもの %d" % n
        g = SwingGroup(name=name, chain=chain, settings=settings.copy() if settings else SimSettings())
        self.groups.append(g)
        return g

    def get_group(self, group_id):
        for g in self.groups:
            if g.id == group_id:
                return g
        return None

    def group_of(self, entry):
        return self.get_group(entry.group)

    def group_entries(self, group_id):
        return [e for e in self.entries if e.group == group_id]

    def remove_group(self, group_id):
        self.entries = [e for e in self.entries if e.group != group_id]
        self.groups = [g for g in self.groups if g.id != group_id]

    def move_group(self, group_id, offset):
        i = next((k for k, g in enumerate(self.groups) if g.id == group_id), None)
        if i is None:
            return
        j = i + offset
        if 0 <= j < len(self.groups):
            self.groups[i], self.groups[j] = self.groups[j], self.groups[i]
            self._normalise_order()

    # ------------------------------------------------------------------
    # controllers
    # ------------------------------------------------------------------
    def add_selected(self, group_id, hierarchy=False):
        """Register the selected transforms in ``group_id`` in selection
        order.  With ``hierarchy`` ("＋ 新規") the FK controllers below each
        selected node are registered too (see :func:`fk_descendants`).
        Returns (added entries, warnings)."""
        if self.get_group(group_id) is None:
            raise SimflowError("揺れものが選ばれていません。")
        selection = cmds.ls(orderedSelection=True, long=True, type="transform") or []
        if not selection:
            raise SimflowError("先にコントローラー（トランスフォーム）を選択してください。")
        added, warnings = [], []
        if hierarchy:
            selection, info = expand_fk_hierarchy(selection)
            warnings.extend(info)
        for node in selection:
            if scene_manager.is_tool_node(node) or "|%s" % scene_manager.ROOT_NAME in node:
                warnings.append("%s は SIMFLOW のノードなので追加しませんでした" % short_name(node))
                continue
            uid = node_uuid(node)
            other = next((e for e in self.entries if e.uuid == uid), None)
            if other is not None:
                g = self.group_of(other)
                warnings.append("%s はすでに「%s」に登録されています" % (short_name(node), g.name if g else "?"))
                continue
            entry = ControllerEntry(path=node, uuid=uid, group=group_id)
            self.entries.append(entry)
            added.append(entry)
        self._normalise_order()
        return added, warnings

    def remove(self, ids):
        self.entries = [e for e in self.entries if e.id not in set(ids)]

    def clear(self):
        self.entries = []
        self.groups = []

    def get(self, entry_id):
        for e in self.entries:
            if e.id == entry_id:
                return e
        return None

    def enabled_entries(self, group_id=None):
        out = []
        for e in self.entries:
            g = self.group_of(e)
            if e.enabled and g is not None and (g.id == group_id if group_id else g.enabled):
                out.append(e)
        return out

    def move(self, ids, offset):
        """Move entries up (-1) / down (+1) inside their group (chain order)."""
        ids = set(ids)
        for g in self.groups:
            order = self.group_entries(g.id)
            rng = range(len(order)) if offset < 0 else range(len(order) - 1, -1, -1)
            for i in rng:
                j = i + offset
                if order[i].id in ids and 0 <= j < len(order) and order[j].id not in ids:
                    order[i], order[j] = order[j], order[i]
            others = [e for e in self.entries if e.group != g.id]
            self.entries = others + order
        self._normalise_order()

    def _normalise_order(self):
        """Keep ``entries`` grouped in group order (chain order inside)."""
        out = []
        for g in self.groups:
            out.extend(self.group_entries(g.id))
        orphans = [e for e in self.entries if self.get_group(e.group) is None]
        self.entries = out + orphans

    # ------------------------------------------------------------------
    def resolve(self, entry):
        """Return the current long name of the controller or raise.

        UUID first, then the stored long path.  When the node was found via
        the path but its UUID changed (e.g. reference reload) the entry is
        updated and a warning is logged."""
        node = node_from_uuid(entry.uuid)
        if node:
            if node != entry.path:
                LOG.info("Controller renamed/moved: %s -> %s", entry.path, node)
                entry.path = node
            return node
        if cmds.objExists(entry.path) and len(cmds.ls(entry.path) or []) == 1:
            node = long_name(entry.path)
            new_uid = node_uuid(node)
            LOG.warning("コントローラー %s をパスで見つけました（UUID が変わっています。リファレンスを再読み込みしましたか？）",
                        entry.label)
            entry.uuid = new_uid
            return node
        raise SimflowError("コントローラーが見つかりません: %s（削除されたか、リファレンスがアンロードされています）"
                           % entry.path)

    def status(self, entry):
        try:
            self.resolve(entry)
            return "ok"
        except SimflowError:
            return "missing"

    # ------------------------------------------------------------------
    def to_list(self):
        return [e.to_dict() for e in self.entries]

    def groups_to_list(self):
        return [g.to_dict() for g in self.groups]

    def from_data(self, groups, controllers):
        self.groups = [SwingGroup.from_dict(d) for d in (groups or [])]
        self.entries = [ControllerEntry.from_dict(d) for d in (controllers or [])]
        self._normalise_order()

    def entries_from_list(self, controllers):
        """Undo / redo: controller and bake state only (group settings stay)."""
        self.entries = [ControllerEntry.from_dict(d) for d in (controllers or [])]
        self._normalise_order()
