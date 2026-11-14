# -*- coding: utf-8 -*-
"""Swing group list and controller list widgets."""
from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets

ID_ROLE = QtCore.Qt.UserRole + 1
MODE_NAMES = {"additive": "加算", "override": "オーバーライド", "overwrite": "上書き"}


def _chain_count(entries):
    """How many chains the group is split into (by hierarchy)."""
    from ..core.simulation_manager import SimulationManager
    try:
        return len(SimulationManager.split_chains(entries, {e.id: e.path for e in entries}))
    except Exception:          # e.g. an order problem: reported when calculating
        return 1


CHAIN_ROLE = QtCore.Qt.UserRole + 2


class GroupListWidget(QtWidgets.QTreeWidget):
    """One row per swing group (the check box = calculate / bake it).  A
    chain group opens (▸) into one row per chain (also a single chain):
    choosing a chain selects its controllers in Maya and shows that chain's
    own values ("チェーンごとの値")."""

    enabledChanged = QtCore.Signal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setHeaderHidden(True)
        self.setColumnCount(1)
        self.setIndentation(14)
        self.setRootIsDecorated(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setMinimumHeight(70)
        self.setMaximumHeight(200)
        self.setTextElideMode(QtCore.Qt.ElideRight)
        self._expanded = set()
        self.itemExpanded.connect(lambda it: self._expanded.add(it.data(0, ID_ROLE)))
        self.itemCollapsed.connect(lambda it: self._expanded.discard(it.data(0, ID_ROLE)))
        self.itemChanged.connect(self._changed)

    def _changed(self, it, _col):
        if it.parent() is None:
            self.enabledChanged.emit(it.data(0, ID_ROLE), it.checkState(0) == QtCore.Qt.Checked)

    def set_groups(self, groups, entries_of, current_id=None, chains_of=None, current_chain=None):
        self.blockSignals(True)
        self.clear()
        pick = None
        for g in groups:
            entries = entries_of(g.id)
            mode = "チェーン" if g.chain and len(entries) > 1 else "1 つずつ"
            chains = chains_of(g.id) if chains_of and g.chain and not g.rope and entries else []
            if g.rope:
                mode = "縄"
            elif mode == "チェーン" and len(chains) > 1:
                mode = "チェーン %d 本" % len(chains)
            baked = any(e.bake for e in entries)
            text = "%s    %s・%d 個%s" % (g.name, mode, len(entries), "   [ベイク済み]" if baked else "")
            item = QtWidgets.QTreeWidgetItem([text])
            item.setData(0, ID_ROLE, g.id)
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(0, QtCore.Qt.Checked if g.enabled else QtCore.Qt.Unchecked)
            item.setToolTip(0, "チェックを入れた揺れものが、計算とベイクの対象になります。\n"
                               "▸ で開くと、チェーンが 1 本ずつ並びます")
            self.addTopLevelItem(item)
            for k, chain in enumerate(chains):
                own = g.chain_overrides.get(chain[0].id) or {}
                ctext = "%d. %s〜%s（%d 個）" % (k + 1, chain[0].label, chain[-1].label, len(chain))
                if own:
                    ctext += "   [個別の値 %d]" % len(own)
                child = QtWidgets.QTreeWidgetItem([ctext])
                child.setData(0, ID_ROLE, g.id)
                child.setData(0, CHAIN_ROLE, chain[0].id)
                child.setToolTip(0, "選ぶと、このチェーンのコントローラーを Maya で選択し、\n"
                                    "スライダーがこのチェーンだけの値になります（揺れもの全体の値の上に重ねます）")
                item.addChild(child)
                if g.id == current_id and chain[0].id == current_chain:
                    pick = child
            if g.id in self._expanded or (pick is not None and pick.parent() is item):
                item.setExpanded(True)
            if g.id == current_id and pick is None:
                pick = item
        if pick is None and self.topLevelItemCount() and current_id is None:
            pick = self.topLevelItem(0)
        if pick is not None:
            pick.setSelected(True)
            self.setCurrentItem(pick)
        self.blockSignals(False)

    def current_id(self):
        items = self.selectedItems()
        return items[0].data(0, ID_ROLE) if items else None

    def current_chain(self):
        """Root entry id of the chosen chain (None: the group itself)."""
        items = self.selectedItems()
        return items[0].data(0, CHAIN_ROLE) if items else None


class ControllerListWidget(QtWidgets.QListWidget):
    """Controllers of one group in chain order (root first)."""

    selectInMaya = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.setMinimumHeight(70)
        self.setMaximumHeight(130)
        self.itemDoubleClicked.connect(lambda it: self.selectInMaya.emit(it.data(ID_ROLE)))

    def set_entries(self, entries, status_fn=None, rope=False):
        selected = set(self.selected_ids())
        self.blockSignals(True)
        self.clear()
        for idx, e in enumerate(entries):
            status = status_fn(e) if status_fn else "ok"
            tags = []
            if rope and idx in (0, len(entries) - 1):
                tags.append("持ち手")
            if e.rest:
                tags.append("レスト@%g" % e.rest.get("frame", 0))
            if e.bake:
                tags.append("ベイク済み:%s" % MODE_NAMES.get(e.bake.get("mode"), e.bake.get("mode")))
            if status != "ok":
                tags.append("見つかりません" if status == "missing" else status)
            text = "%d. %s" % (idx + 1, e.label)
            if tags:
                text += "   [" + "、".join(tags) + "]"
            item = QtWidgets.QListWidgetItem(text)
            item.setData(ID_ROLE, e.id)
            item.setToolTip(e.path + "\nUUID: " + e.uuid + "\nダブルクリックで Maya 上で選択")
            if status != "ok":
                item.setForeground(QtGui.QBrush(QtGui.QColor("#e06c5a")))
            self.addItem(item)
            if e.id in selected:
                item.setSelected(True)
        self.blockSignals(False)

    def selected_ids(self):
        return [it.data(ID_ROLE) for it in self.selectedItems()]
