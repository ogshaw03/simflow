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


class GroupListWidget(QtWidgets.QListWidget):
    """One row per swing group; the check box = calculate / bake it."""

    enabledChanged = QtCore.Signal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setMinimumHeight(70)
        self.setMaximumHeight(130)
        self.itemChanged.connect(
            lambda it: self.enabledChanged.emit(it.data(ID_ROLE), it.checkState() == QtCore.Qt.Checked))

    def set_groups(self, groups, entries_of, current_id=None):
        self.blockSignals(True)
        self.clear()
        for g in groups:
            entries = entries_of(g.id)
            mode = "チェーン" if g.chain and len(entries) > 1 else "1 つずつ"
            if g.rope:
                mode = "縄"
            elif mode == "チェーン":
                n = _chain_count(entries)
                if n > 1:
                    mode = "チェーン %d 本" % n
            baked = any(e.bake for e in entries)
            text = "%s    %s・%d 個%s" % (g.name, mode, len(entries), "   [ベイク済み]" if baked else "")
            item = QtWidgets.QListWidgetItem(text)
            item.setData(ID_ROLE, g.id)
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Checked if g.enabled else QtCore.Qt.Unchecked)
            item.setToolTip("チェックを入れた揺れものが、計算とベイクの対象になります")
            self.addItem(item)
            if g.id == current_id:
                item.setSelected(True)
                self.setCurrentItem(item)
        if current_id is None and self.count():
            self.item(0).setSelected(True)
            self.setCurrentRow(0)
        self.blockSignals(False)

    def current_id(self):
        items = self.selectedItems()
        return items[0].data(ID_ROLE) if items else None


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
