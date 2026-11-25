# -*- coding: utf-8 -*-
"""ライブラリの画面 (studioLibrary style): folders on the left, the items as
thumbnail tiles, the chosen item's details and how to load it on the right.

Opened from the preset row's「…」menu of the main window.  It only calls
:mod:`simflow.core.library`; the main window is refreshed after loading."""
from __future__ import annotations

import os
import subprocess
import tempfile

import maya.cmds as cmds
from PySide6 import QtCore, QtGui, QtWidgets

from ..core import collider_manager
from ..core import library as lib
from ..utils.maya_utils import SimflowError
from . import widgets as W

TILE = 112
_WINDOW = None


def show(main):
    """Open (or bring back) the library window of the main window ``main``."""
    global _WINDOW
    try:
        if _WINDOW is not None and _WINDOW.main is main:
            _WINDOW.refresh()
            _WINDOW.show()
            return _WINDOW
    except RuntimeError:          # deleted with its parent
        pass
    _WINDOW = LibraryWindow(main)
    _WINDOW.show()
    return _WINDOW


def _btn(text, fn, accent=False):
    b = (W.accent_button if accent else W.button)(text)
    b.clicked.connect(lambda *_: fn())
    return b


def placeholder(text, size=TILE):
    pm = QtGui.QPixmap(size, size)
    pm.fill(QtGui.QColor("#3a3a3a"))
    p = QtGui.QPainter(pm)
    p.setPen(QtGui.QColor(W.MUTED))
    p.drawText(pm.rect(), QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, text)
    p.end()
    return pm


def thumb_pixmap(path, size=TILE, badge=False):
    t = lib.thumbnail(path) if path else None
    pm = QtGui.QPixmap(t) if t else QtGui.QPixmap()
    if pm.isNull():
        seq = lib.sequence(path) if path else []
        pm = QtGui.QPixmap(seq[0]) if seq else QtGui.QPixmap()
    if pm.isNull():
        return placeholder("画像なし", size)
    pm = pm.scaled(size, size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
    if badge:
        # a small "play" mark: this tile has a sequence (move the mouse over it)
        p = QtGui.QPainter(pm)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        s = max(10, size // 7)
        x, y = pm.width() - s - 4, pm.height() - s - 4
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(0, 0, 0, 150))
        p.drawEllipse(QtCore.QRectF(x - 2, y - 2, s + 4, s + 4))
        p.setBrush(QtGui.QColor("#ffffff"))
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x + s * 0.32, y + s * 0.2), QtCore.QPointF(x + s * 0.32, y + s * 0.8),
                                       QtCore.QPointF(x + s * 0.84, y + s * 0.5)]))
        p.end()
    return pm


class Frames(object):
    """The scaled images of a sequence, read on first use."""

    def __init__(self, files, size):
        self.files = list(files)
        self.size = size
        self._cache = {}

    def __len__(self):
        return len(self.files)

    def get(self, i):
        i = max(0, min(len(self.files) - 1, int(i)))
        pm = self._cache.get(i)
        if pm is None:
            pm = QtGui.QPixmap(self.files[i])
            if not pm.isNull():
                pm = pm.scaled(self.size, self.size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self._cache[i] = pm
        return pm


class Player(QtCore.QObject):
    """Plays a sequence on a label over and over (the details / the save dialog)."""

    def __init__(self, label, parent=None):
        super().__init__(parent)
        self.label = label
        self.frames = None
        self.index = 0
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._next)

    def play(self, frames, fps=24.0, step=1):
        self.frames = frames if frames is not None and len(frames) > 1 else None
        self.index = 0
        self.timer.stop()
        if self.frames is None:
            return
        self.timer.setInterval(max(15, int(round(1000.0 * max(1, step) / max(1.0, fps or 24.0)))))
        self._next()
        self.timer.start()

    def stop(self):
        self.timer.stop()
        self.frames = None

    def pause(self, on):
        if on:
            self.timer.stop()
        elif self.frames is not None:
            self.timer.start()

    def _next(self):
        if self.frames is None:
            return
        try:
            self.label.setPixmap(self.frames.get(self.index))
        except RuntimeError:          # the label is gone
            self.timer.stop()
            return
        self.index = (self.index + 1) % len(self.frames)


def seq_info_text(info):
    if not info:
        return ""
    return "連番: %d〜%d フレーム（%d 枚%s）" % (info.get("start", 0), info.get("end", 0), info.get("count", 0),
                                         "・%d フレームおき" % info["step"] if info.get("step", 1) > 1 else "")


class LibraryWindow(QtWidgets.QWidget):

    def __init__(self, main):
        super().__init__(main, QtCore.Qt.Window)
        self.main = main
        self.setWindowTitle("SIMFLOW - ライブラリ")
        self.setObjectName("SimflowRoot")
        self.setStyleSheet(W.STYLE)
        self.resize(880, 560)
        self.root = lib.library_path()
        self._seq = {}          # (path, size) -> Frames
        self._static = {}       # path -> the tile's still icon
        self._hover = None      # the tile being scrubbed
        self._build()
        W.no_wheel(self)
        self.refresh()

    # ------------------------------------------------------------------
    def _build(self):
        lay = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("保存先"))
        self.ed_path = QtWidgets.QLineEdit()
        self.ed_path.setReadOnly(True)
        self.ed_path.setToolTip("ライブラリのフォルダ。共有フォルダを選ぶと、チームで同じライブラリを使えます")
        top.addWidget(self.ed_path, 1)
        b = _btn("フォルダを選ぶ...", self._choose_root)
        b.setToolTip("ライブラリのフォルダを変えます（共有フォルダも選べます）。次から開くときもこのフォルダです")
        top.addWidget(b)
        top.addWidget(_btn("エクスプローラー", self._open_explorer))
        lay.addLayout(top)

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        # folders
        left = QtWidgets.QWidget()
        ll = QtWidgets.QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.currentItemChanged.connect(lambda *_: self.refresh_items())
        ll.addWidget(self.tree, 1)
        ll.addWidget(_btn("＋ フォルダ", self._new_folder))
        split.addWidget(left)
        # tiles
        mid = QtWidgets.QWidget()
        ml = QtWidgets.QVBoxLayout(mid)
        ml.setContentsMargins(0, 0, 0, 0)
        self.ed_search = QtWidgets.QLineEdit()
        self.ed_search.setPlaceholderText("検索（名前・コメント・フォルダ）")
        self.ed_search.textChanged.connect(lambda *_: self.refresh_items())
        ml.addWidget(self.ed_search)
        self.tiles = QtWidgets.QListWidget()
        self.tiles.setViewMode(QtWidgets.QListView.IconMode)
        self.tiles.setIconSize(QtCore.QSize(TILE, TILE))
        self.tiles.setGridSize(QtCore.QSize(TILE + 18, TILE + 38))
        self.tiles.setResizeMode(QtWidgets.QListView.Adjust)
        self.tiles.setMovement(QtWidgets.QListView.Static)
        self.tiles.setWordWrap(True)
        self.tiles.setSpacing(4)
        self.tiles.currentItemChanged.connect(lambda *_: self.show_details())
        self.tiles.itemDoubleClicked.connect(lambda *_: self._apply())
        self.tiles.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.tiles.customContextMenuRequested.connect(self._tile_menu)
        # moving the mouse across a tile plays its sequence (left = first frame, right = last)
        self.tiles.setMouseTracking(True)
        self.tiles.viewport().setMouseTracking(True)
        self.tiles.viewport().installEventFilter(self)
        ml.addWidget(self.tiles, 1)
        b_save = _btn("＋ 揺れものを保存...", self._save, accent=True)
        b_save.setToolTip("揺れものの値と、当たり判定（形・位置・付いているノード）を、このフォルダに保存します。\n"
                          "揺れものは複数まとめて 1 つの項目にできます（ポニーテール・前髪・アホ毛など、キャラの髪を全部）")
        ml.addWidget(b_save)
        split.addWidget(mid)
        # details
        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        self.lbl_thumb = QtWidgets.QLabel()
        self.lbl_thumb.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_thumb.setFixedHeight(180)
        self.lbl_thumb.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        rl.addWidget(self.lbl_thumb)
        self.player = Player(self.lbl_thumb, self)
        # the name and details scroll on their own: a long list of colliders
        # never pushes into the thumbnail or the buttons
        info = QtWidgets.QWidget()
        il = QtWidgets.QVBoxLayout(info)
        il.setContentsMargins(0, 0, 4, 0)
        self.lbl_name = QtWidgets.QLabel()
        self.lbl_name.setStyleSheet("font-weight: bold;")
        self.lbl_name.setWordWrap(True)
        il.addWidget(self.lbl_name)
        self.lbl_info = QtWidgets.QLabel()
        self.lbl_info.setWordWrap(True)
        self.lbl_info.setProperty("muted", True)
        self.lbl_info.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
        self.lbl_info.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        il.addWidget(self.lbl_info)
        il.addStretch(1)
        self.info_scroll = QtWidgets.QScrollArea()
        self.info_scroll.setWidgetResizable(True)
        self.info_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.info_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.info_scroll.setMinimumHeight(60)
        self.info_scroll.setWidget(info)
        rl.addWidget(self.info_scroll, 1)
        nrow = QtWidgets.QHBoxLayout()
        nrow.addWidget(QtWidgets.QLabel("ネームスペース"))
        self.ed_ns = QtWidgets.QLineEdit()
        self.ed_ns.setPlaceholderText("なし")
        self.ed_ns.setToolTip("読み込むリグのネームスペース。保存した名前の前に付けて、コントローラーと当たりの付け先を探します")
        nrow.addWidget(self.ed_ns, 1)
        b_ns = _btn("選択から", self._ns_from_selection)
        b_ns.setToolTip("選んでいるノードのネームスペースにします")
        nrow.addWidget(b_ns)
        rl.addLayout(nrow)
        self.chk_values = QtWidgets.QCheckBox("値を入れる")
        self.chk_values.setChecked(True)
        rl.addWidget(self.chk_values)
        self.rb_saved = QtWidgets.QRadioButton("保存したコントローラーで作る")
        self.rb_saved.setToolTip("保存したときのコントローラーを、名前（＋ネームスペース）で探して揺れものを作ります。\n"
                                 "選ぶ必要はありません。同じコントローラーの揺れものがもうあれば、その値を入れ直します。\n"
                                 "見つからないコントローラーがあれば、見つかった分で作って知らせます")
        self.rb_saved.setChecked(True)
        self.rb_new = QtWidgets.QRadioButton("選んだコントローラーで作る")
        self.rb_new.setToolTip("選んでいるコントローラーで新しい揺れものを作ります（別の名前のリグに使うとき）")
        self.rb_cur = QtWidgets.QRadioButton("選んでいる揺れものに入れる")
        self.rb_cur.setToolTip("パネルで選んでいる揺れものに値だけを入れます")
        for rb in (self.rb_saved, self.rb_new, self.rb_cur):
            row = QtWidgets.QHBoxLayout()
            row.addSpacing(16)
            row.addWidget(rb)
            rl.addLayout(row)
        self.lbl_multi = QtWidgets.QLabel("揺れものが複数入った項目です。保存したコントローラーの名前（＋ネームスペース）で"
                                          "揺れものごとに探して作ります。同じコントローラーの揺れものがあれば、その値を入れ直します")
        self.lbl_multi.setWordWrap(True)
        self.lbl_multi.setProperty("muted", True)
        self.lbl_multi.hide()
        rl.addWidget(self.lbl_multi)
        self.chk_values.toggled.connect(lambda on: self._sync_targets())
        self.chk_cols = QtWidgets.QCheckBox("当たり判定を作る")
        self.chk_cols.setChecked(True)
        self.chk_cols.setToolTip("保存した当たり判定を、ネームスペースの付いた同じ名前のノードに付けて作ります。\n"
                                 "同じ名前の当たりが同じノードにもうあるときは作りません")
        rl.addWidget(self.chk_cols)
        self.b_apply = _btn("適用", self._apply, accent=True)
        rl.addWidget(self.b_apply)
        row = QtWidgets.QHBoxLayout()
        b_sel = _btn("コントローラーを選択", self._select)
        b_sel.setToolTip("保存したときのコントローラーを、ネームスペースを付けて選択します")
        row.addWidget(b_sel)
        row.addWidget(_btn("削除", self._delete))
        rl.addLayout(row)
        right.setMinimumWidth(250)
        split.addWidget(right)
        split.setSizes([160, 440, 260])
        lay.addWidget(split, 1)
        self.status = QtWidgets.QLabel()
        self.status.setObjectName("Status")
        self.status.setWordWrap(True)
        lay.addWidget(self.status)
        self._ns_from_selection(quiet=True)

    # ------------------------------------------------------------------
    def set_status(self, text):
        self.status.setText(text)

    def folder(self):
        it = self.tree.currentItem()
        return it.data(0, QtCore.Qt.UserRole) if it else ""

    def current_path(self):
        it = self.tiles.currentItem()
        return it.data(QtCore.Qt.UserRole) if it else None

    def refresh(self):
        self.root = lib.library_path()
        self.ed_path.setText(self.root)
        keep = self.folder()
        self.tree.blockSignals(True)
        self.tree.clear()
        nodes = {}
        top = QtWidgets.QTreeWidgetItem(["ライブラリ"])
        top.setData(0, QtCore.Qt.UserRole, "")
        self.tree.addTopLevelItem(top)
        nodes[""] = top
        for rel in lib.folders(self.root)[1:]:
            parent = nodes.get(rel.rpartition("/")[0], top)
            it = QtWidgets.QTreeWidgetItem(parent, [rel.split("/")[-1]])
            it.setData(0, QtCore.Qt.UserRole, rel)
            nodes[rel] = it
        self.tree.expandAll()
        self.tree.setCurrentItem(nodes.get(keep, top))
        self.tree.blockSignals(False)
        self.refresh_items()

    def frames(self, path, size):
        key = (os.path.normcase(path or ""), size)
        if key not in self._seq:
            self._seq[key] = Frames(lib.sequence(path) if path else [], size)
        return self._seq[key]

    def eventFilter(self, obj, ev):
        try:
            if obj is self.tiles.viewport():
                if ev.type() == QtCore.QEvent.MouseMove:
                    self._scrub(ev.position().toPoint() if hasattr(ev, "position") else ev.pos())
                elif ev.type() == QtCore.QEvent.Leave:
                    self._unscrub()
        except RuntimeError:
            pass
        return super().eventFilter(obj, ev)

    def _scrub(self, pos):
        it = self.tiles.itemAt(pos)
        if it is not self._hover:
            self._unscrub()
        if it is None:
            return
        path = it.data(QtCore.Qt.UserRole)
        frames = self.frames(path, TILE)
        if len(frames) < 2:
            return
        self._hover = it
        rect = self.tiles.visualItemRect(it)
        frac = (pos.x() - rect.left()) / float(max(1, rect.width()))
        it.setIcon(QtGui.QIcon(frames.get(int(frac * len(frames)))))

    def _unscrub(self):
        it, self._hover = self._hover, None
        if it is None:
            return
        try:
            icon = self._static.get(os.path.normcase(it.data(QtCore.Qt.UserRole) or ""))
            if icon is not None:
                it.setIcon(icon)
        except RuntimeError:          # the tile was removed
            pass

    def hideEvent(self, ev):
        self.player.pause(True)
        super().hideEvent(ev)

    def showEvent(self, ev):
        super().showEvent(ev)
        self.player.pause(False)

    def refresh_items(self, select=None):
        keep = select or self.current_path()
        self._hover = None
        self._seq.clear()
        self._static.clear()
        self.tiles.clear()
        found = lib.items(self.root, self.folder(), self.ed_search.text())
        for path, data in found:
            icon = QtGui.QIcon(thumb_pixmap(path, badge=bool(lib.sequence(path))))
            self._static[os.path.normcase(path)] = icon
            it = QtWidgets.QListWidgetItem(icon, data.get("name", ""))
            it.setData(QtCore.Qt.UserRole, path)
            it.setToolTip("%s\n%s%s" % (data.get("name", ""), lib.summary(data),
                                        "\nマウスを左右に動かすと揺れ方が見られます" if lib.sequence(path) else "")
                          + "\n右クリック：上書き保存・名前を変える・削除など")
            self.tiles.addItem(it)
            if keep and os.path.normcase(path) == os.path.normcase(keep):
                self.tiles.setCurrentItem(it)
        if not os.path.isdir(self.root):
            self.set_status("ライブラリのフォルダはまだありません。保存すると作ります: %s" % self.root)
        else:
            self.set_status("%d 件" % len(found))
        self.show_details()

    def show_details(self):
        path = self.current_path()
        data = None
        if path:
            try:
                data = lib.read_item(path)
            except lib.LibraryError as exc:
                self.set_status(str(exc))
        self.b_apply.setEnabled(data is not None)
        self.player.stop()
        if data is None:
            self.lbl_thumb.setPixmap(placeholder("項目を選んでください", 170))
            self.lbl_name.setText("")
            self.lbl_info.setText("")
            return
        self.lbl_thumb.setPixmap(thumb_pixmap(path, 170))
        seq = data.get("sequence") or {}
        self.player.play(self.frames(path, 170), seq.get("fps", 24.0), seq.get("step", 1))
        self.lbl_name.setText(data.get("name", ""))
        lines = [lib.summary(data)]
        if self.player.frames is not None:
            lines.append(seq_info_text(seq) or "連番 %d 枚" % len(self.player.frames))
        cols = data.get("colliders") or []
        if cols:
            lines.append("当たり: " + "、".join("%s（%s）" % (c["name"], c.get("attach") or "ワールド") for c in cols))
        if data.get("comment"):
            lines.append(data["comment"])
        lines.append("%s %s%s" % (data.get("created", ""), data.get("user", ""),
                                  "（%s）" % data["scene"] if data.get("scene") else ""))
        self.lbl_info.setText("\n".join(lines))
        self.info_scroll.verticalScrollBar().setValue(0)
        self.chk_values.setEnabled(bool(lib.groups_of(data)))
        self.chk_cols.setEnabled(bool(cols))
        self._sync_targets()

    def _multi(self):
        path = self.current_path()
        try:
            return len(lib.groups_of(lib.read_item(path))) > 1 if path else False
        except lib.LibraryError:
            return False

    def _sync_targets(self):
        """「保存した / 選んだコントローラー / 選んでいる揺れもの」 only for an item of one group."""
        multi = self._multi()
        on = self.chk_values.isChecked() and self.chk_values.isEnabled()
        for rb in (self.rb_saved, self.rb_new, self.rb_cur):
            rb.setVisible(not multi)
            rb.setEnabled(on)
        self.lbl_multi.setVisible(multi)

    # ------------------------------------------------------------------
    def _choose_root(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(self, "SIMFLOW - ライブラリのフォルダ", self.root)
        if path:
            lib.set_library_path(os.path.normpath(path))
            self.refresh()
            self.set_status("ライブラリのフォルダを変えました: %s" % os.path.normpath(path))

    def _open_explorer(self):
        path = os.path.join(self.root, self.folder())
        if not os.path.isdir(path):
            self.set_status("フォルダがまだありません: %s" % path)
            return
        subprocess.Popen(["explorer", os.path.normpath(path)])

    def _new_folder(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "SIMFLOW - フォルダ", "フォルダの名前：")
        if not ok:
            return
        try:
            rel = lib.make_folder(self.root, self.folder(), name)
        except (lib.LibraryError, OSError) as exc:
            self.set_status(str(exc))
            return
        self.refresh()
        for it in self.tree.findItems(name, QtCore.Qt.MatchExactly | QtCore.Qt.MatchRecursive):
            if it.data(0, QtCore.Qt.UserRole) == rel:
                self.tree.setCurrentItem(it)

    def _ns_from_selection(self, quiet=False):
        ns = lib.guess_namespace()
        self.ed_ns.setText(ns)
        if not quiet:
            self.set_status("ネームスペース: %s" % (ns or "なし"))

    def _select(self):
        path = self.current_path()
        if not path:
            return
        found, missing = lib.select_controllers(lib.read_item(path), self.ed_ns.text().strip())
        self.set_status("%d 個選択しました。%s" % (len(found), ("見つからない: " + "、".join(missing)) if missing else ""))

    def tile_menu(self):
        """The right-click menu of the chosen tile (built here so the check can read it)."""
        m = QtWidgets.QMenu(self)
        m.addAction("適用", self._apply)
        m.addAction("コントローラーを選択", self._select)
        m.addSeparator()
        m.addAction("上書き保存...", self._overwrite)
        m.addAction("サムネだけ撮り直す...", self._retake_pictures)
        m.addAction("名前を変える...", self._rename)
        m.addAction("エクスプローラーで開く", self._open_item)
        m.addSeparator()
        m.addAction("削除", self._delete)
        return m

    def _tile_menu(self, pos):
        it = self.tiles.itemAt(pos)
        if it is None:
            return
        self.tiles.setCurrentItem(it)
        self.tile_menu().exec(self.tiles.viewport().mapToGlobal(pos))

    def _open_item(self):
        path = self.current_path()
        if path and os.path.isdir(path):
            subprocess.Popen(["explorer", os.path.normpath(path)])

    def _rename(self, name=None):
        path = self.current_path()
        if not path:
            return None
        old = os.path.basename(path)[:-len(lib.EXT)]
        if name is None:
            name, ok = QtWidgets.QInputDialog.getText(self, "SIMFLOW - 名前を変える", "新しい名前：", text=old)
            if not ok or name.strip() == old:
                return None
        try:
            new = lib.rename_item(path, name)
        except (lib.LibraryError, OSError) as exc:
            self.set_status(str(exc))
            QtWidgets.QMessageBox.warning(self, "SIMFLOW", str(exc))
            return None
        self.refresh_items(select=new)
        self.set_status("「%s」を「%s」に変えました。" % (old, name.strip()))
        return new

    def _overwrite(self):
        path = self.current_path()
        if not path:
            return None
        main = self.main
        g = main.current_group() if hasattr(main, "current_group") else None
        try:
            dlg = SaveDialog(self, main.session, g, lib.folders(self.root), self.folder(), item=path)
        except lib.LibraryError as exc:
            self.set_status(str(exc))
            return None
        self._last_dialog = dlg
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return None
        self.refresh_items(select=dlg.saved)
        self.set_status("上書き保存しました: %s" % dlg.saved)
        return dlg.saved

    def _retake_pictures(self):
        """サムネだけ撮り直す: the still / sequence of the chosen item from the
        viewport now; its values and colliders are kept."""
        path = self.current_path()
        if not path:
            return None
        main = self.main
        try:
            dlg = SaveDialog(self, main.session, None, lib.folders(self.root), self.folder(), item=path,
                             pictures_only=True)
        except lib.LibraryError as exc:
            self.set_status(str(exc))
            return None
        self._last_dialog = dlg
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return None
        self.refresh_items(select=dlg.saved)
        self.set_status("サムネを撮り直しました: %s" % os.path.basename(dlg.saved)[:-len(lib.EXT)])
        return dlg.saved

    def _delete(self):
        path = self.current_path()
        if not path:
            return
        name = os.path.basename(path)[:-len(lib.EXT)]
        if QtWidgets.QMessageBox.question(self, "SIMFLOW", "「%s」をライブラリから削除しますか？\n（元に戻せません）" % name) \
                != QtWidgets.QMessageBox.Yes:
            return
        try:
            lib.delete_item(path)
        except (lib.LibraryError, OSError) as exc:
            self.set_status(str(exc))
            return
        self.refresh_items()
        self.set_status("「%s」を削除しました。" % name)

    def _apply(self):
        path = self.current_path()
        if not path:
            return
        data = lib.read_item(path)
        values = self.chk_values.isChecked() and self.chk_values.isEnabled()
        cols = self.chk_cols.isChecked() and self.chk_cols.isEnabled()
        if not (values or cols):
            self.set_status("「値を入れる」か「当たり判定を作る」を選んでください。")
            return
        main = self.main
        g = main.current_group() if hasattr(main, "current_group") else None
        try:
            new_g, report = lib.apply(main.session, data, values=values, colliders=cols,
                                      target=("saved" if self.rb_saved.isChecked() else
                                              "new" if self.rb_new.isChecked() else "current"),
                                      group_id=g.id if g else None, ns=self.ed_ns.text().strip(),
                                      find_fk=main.chk_find_fk.isChecked() if hasattr(main, "chk_find_fk") else True)
        except (SimflowError, lib.LibraryError) as exc:
            self.set_status(str(exc))
            QtWidgets.QMessageBox.warning(self, "SIMFLOW", str(exc))
            return
        try:
            if new_g is not None:
                main._group_id = new_g.id
            main.refresh_all()
        except Exception:
            pass
        text = "「%s」を読み込みました。%s" % (data.get("name", ""), " ".join(report))
        self.set_status(text)
        if hasattr(main, "set_status"):
            main.set_status(text)
        # found by the saved names: say plainly what was left out
        left = [r for r in report if "作りませんでした" in r or "見つからないコントローラー" in r]
        if left and values and (len(lib.groups_of(data)) > 1 or self.rb_saved.isChecked()):
            QtWidgets.QMessageBox.information(self, "SIMFLOW", "読み込みました。次のものは見つかりませんでした：\n\n" + "\n".join(left))

    def _save(self):
        main = self.main
        g = main.current_group() if hasattr(main, "current_group") else None
        dlg = SaveDialog(self, main.session, g, lib.folders(self.root), self.folder())
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return
        self.refresh()
        self.refresh_items(select=dlg.saved)
        self.set_status("保存しました: %s" % dlg.saved)


class SaveDialog(QtWidgets.QDialog):
    """Name, folder, comment, what to save (values / which colliders), thumbnail."""

    def __init__(self, parent, session, group, folders, folder, item=None, pictures_only=False):
        """``item``: the path of an item to overwrite (name and folder fixed,
        the rest filled in from it).  ``pictures_only`` (with ``item``): only
        the thumbnail / sequence are taken again (サムネだけ撮り直す)."""
        super().__init__(parent)
        self.session = session
        self.saved = None
        self.item = item
        self.pictures_only = bool(pictures_only and item)
        old = lib.read_item(item) if item else None
        if old is not None:
            root = lib.library_path()
            folder = os.path.relpath(os.path.dirname(item), root).replace("\\", "/")
            folder = "" if folder == "." else folder
            if folder not in folders:
                folders = list(folders) + [folder]
            # the groups saved before, by name (else the one chosen in the panel)
            old_names = [x.get("name") for x in lib.groups_of(old)]
            same = [x for x in session.controllers.groups if x.name in old_names]
            checked = set(x.id for x in same) if same else (set([group.id]) if group and old_names else set())
            group = same[0] if same else (group if old_names else None)
        else:
            checked = set([group.id]) if group else set()
        self.setWindowTitle("SIMFLOW - ライブラリに上書き保存" if item else "SIMFLOW - ライブラリに保存")
        self.setObjectName("SimflowRoot")
        self.setStyleSheet(W.STYLE)
        self.tmp = tempfile.mkdtemp(prefix="simflow_thumb_")
        self.thumb = None
        self.seq = []             # the sequence images taken
        self.seq_info = None
        self.seq_range = None     # (start, end) they were taken for
        lay = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        self.lbl_thumb = QtWidgets.QLabel()
        self.lbl_thumb.setFixedSize(180, 180)
        self.lbl_thumb.setAlignment(QtCore.Qt.AlignCenter)
        left.addWidget(self.lbl_thumb)
        self.player = Player(self.lbl_thumb, self)
        b = _btn("サムネを撮り直す", lambda: self.retake(sequence=True))
        b.setToolTip("今のビューポートをサムネにします（今のフレームの 1 枚と、連番を選んでいれば連番も）")
        left.addWidget(b)
        self.lbl_seq = QtWidgets.QLabel()
        self.lbl_seq.setProperty("muted", True)
        self.lbl_seq.setWordWrap(True)
        self.lbl_seq.setFixedWidth(180)
        left.addWidget(self.lbl_seq)
        left.addStretch(1)
        lay.addLayout(left)
        form = QtWidgets.QFormLayout()
        self.ed_name = QtWidgets.QLineEdit(old.get("name", "") if old else (group.name if group else ""))
        form.addRow("名前", self.ed_name)
        self.cb_folder = QtWidgets.QComboBox()
        for rel in folders:
            self.cb_folder.addItem(rel or "（ライブラリ）", rel)
        self.cb_folder.setCurrentIndex(max(0, self.cb_folder.findData(folder)))
        form.addRow("フォルダ", self.cb_folder)
        self.ed_comment = QtWidgets.QLineEdit(old.get("comment", "") if old else "")
        form.addRow("コメント", self.ed_comment)
        if old is not None:
            self.ed_name.setText(os.path.basename(item)[:-len(lib.EXT)])
            self.ed_name.setEnabled(False)
            self.ed_name.setToolTip("上書き保存では名前は変わりません（「名前を変える...」で変えます）")
            self.cb_folder.setEnabled(False)
        self.grp_list = QtWidgets.QListWidget()
        for x in session.controllers.groups:
            it = QtWidgets.QListWidgetItem(x.name)
            it.setData(QtCore.Qt.UserRole, x.id)
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Checked if x.id in checked else QtCore.Qt.Unchecked)
            self.grp_list.addItem(it)
        self.grp_list.setMaximumHeight(110)
        self.grp_list.setToolTip("チェックを入れた揺れものの値をすべて保存します（チェーンごとの値・当たり判定の設定も）。\n"
                                 "複数にチェックを入れると、1 つの項目にまとめて保存します（キャラの髪を全部など）。\n"
                                 "読み込むときは、保存したコントローラーの名前で揺れものごとに探して作ります。\n"
                                 "どれにもチェックがなければ、値は保存しません（当たり判定だけ）")
        form.addRow("揺れもの", self.grp_list)
        self.col_list = QtWidgets.QListWidget()
        for info in sorted((collider_manager.info(c) for c in collider_manager.colliders()), key=lambda i: i["name"]):
            it = QtWidgets.QListWidgetItem("%s（%s・%s）" % (info["name"], collider_manager.KIND_NAMES.get(info["kind"], ""),
                                                          info["attach"] or "ワールド"))
            it.setData(QtCore.Qt.UserRole, info["node"])
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            on = True
            if old is not None:      # the colliders saved before, by name
                on = lib.strip_ns(info["name"]) in set(c.get("name") for c in old.get("colliders") or [])
            it.setCheckState(QtCore.Qt.Checked if on else QtCore.Qt.Unchecked)
            self.col_list.addItem(it)
        self.col_list.setToolTip("保存する当たり判定（形・大きさ・位置・付いているノード）")
        form.addRow("当たり判定", self.col_list)
        srow = QtWidgets.QHBoxLayout()
        self.chk_seq = QtWidgets.QCheckBox("連番")
        self.chk_seq.setChecked(True)
        self.chk_seq.setToolTip("この範囲を再生したときのビューポートを連番で保存します。\n"
                                "ライブラリのタイルの上でマウスを左右に動かすと、揺れ方が見られます。\n"
                                "揺れが写るよう、プレビューしてから保存してください")
        srow.addWidget(self.chk_seq)
        lo = int(round(cmds.playbackOptions(query=True, minTime=True)))
        hi = int(round(cmds.playbackOptions(query=True, maxTime=True)))
        if old is not None:
            oseq = old.get("sequence") or {}
            if oseq:
                lo, hi = int(oseq.get("start", lo)), int(oseq.get("end", hi))
            self.chk_seq.setChecked(bool(oseq) or not lib.thumbnail(item))
        self.sp_start = QtWidgets.QSpinBox()
        self.sp_end = QtWidgets.QSpinBox()
        for sp, v in ((self.sp_start, lo), (self.sp_end, hi)):
            sp.setRange(-100000, 100000)
            sp.setValue(v)
            sp.setToolTip("連番に撮るフレームの範囲（最初は再生範囲）。長い範囲は %d 枚以内になるよう間を飛ばします" % lib.SEQ_MAX)
        srow.addWidget(self.sp_start)
        srow.addWidget(QtWidgets.QLabel("〜"))
        srow.addWidget(self.sp_end)
        srow.addStretch(1)
        self.chk_seq.toggled.connect(lambda on: (self.sp_start.setEnabled(on), self.sp_end.setEnabled(on)))
        form.addRow("サムネ", srow)
        box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        if old is not None:
            box.button(QtWidgets.QDialogButtonBox.Save).setText("上書き保存")
        if self.pictures_only:
            # only the pictures: the values / colliders of the item are not touched
            self.setWindowTitle("SIMFLOW - サムネだけ撮り直す")
            for w in (self.ed_comment, self.grp_list, self.col_list):
                form.setRowVisible(w, False)
            box.button(QtWidgets.QDialogButtonBox.Save).setText("サムネを保存")
            note = QtWidgets.QLabel("サムネ（と連番）だけを今のビューポートで撮り直します。値・当たり判定は変わりません。")
            note.setWordWrap(True)
            note.setProperty("muted", True)
            form.insertRow(0, note)
        box.accepted.connect(self._save)
        box.rejected.connect(self.reject)
        form.addRow(box)
        lay.addLayout(form, 1)
        W.no_wheel(self)          # the wheel must not change the frame range / folder by accident
        self.resize(560, 260) if self.pictures_only else self.resize(640, 400)
        self.retake(sequence=False)

    def retake(self, sequence=True):
        """The still of the current frame; with ``sequence`` also the sequence
        (when 連番 is on), which is then played in the box."""
        self.player.stop()
        self.thumb = lib.grab_thumbnail(os.path.join(self.tmp, "thumb.png"))
        pm = QtGui.QPixmap(self.thumb) if self.thumb else QtGui.QPixmap()
        self.lbl_thumb.setPixmap(placeholder("画像なし", 170) if pm.isNull()
                                 else pm.scaled(170, 170, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        if sequence and self.chk_seq.isChecked():
            self.grab_sequence()

    def wanted_range(self):
        return (min(self.sp_start.value(), self.sp_end.value()), max(self.sp_start.value(), self.sp_end.value()))

    def grab_sequence(self):
        rng = self.wanted_range()
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            self.seq, self.seq_info = lib.grab_sequence(os.path.join(self.tmp, "seq"), rng[0], rng[1])
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self.seq_range = rng if self.seq else None
        if self.seq:
            self.player.play(Frames(self.seq, 170), self.seq_info["fps"], self.seq_info["step"])
            self.lbl_seq.setText(seq_info_text(self.seq_info))
        else:
            self.lbl_seq.setText("連番を撮れませんでした（ビューポートがありません）")
        return bool(self.seq)

    def done(self, r):
        self.player.stop()
        super().done(r)

    def checked_groups(self):
        return [self.grp_list.item(i).data(QtCore.Qt.UserRole) for i in range(self.grp_list.count())
                if self.grp_list.item(i).checkState() == QtCore.Qt.Checked]

    def checked_colliders(self):
        return [self.col_list.item(i).data(QtCore.Qt.UserRole) for i in range(self.col_list.count())
                if self.col_list.item(i).checkState() == QtCore.Qt.Checked]

    def _save(self):
        if self.pictures_only:
            try:
                seq, info = [], None
                if self.chk_seq.isChecked():
                    if self.seq_range != self.wanted_range():
                        self.grab_sequence()
                    seq, info = self.seq, self.seq_info
                lib.set_pictures(self.item, self.thumb, seq, info)
            except (lib.LibraryError, OSError) as exc:
                QtWidgets.QMessageBox.warning(self, "SIMFLOW", str(exc))
                return
            self.saved = self.item
            self.accept()
            return
        try:
            root = lib.library_path()
            path = lib.item_path(root, self.cb_folder.currentData() or "", self.ed_name.text())
            gid = self.checked_groups()
            cols = self.checked_colliders()
            if not gid and not cols:
                raise lib.LibraryError("揺れものか当たり判定のどちらかにチェックを入れてください。")
            over = False
            if self.item and os.path.normcase(os.path.abspath(path)) == os.path.normcase(os.path.abspath(self.item)):
                over = True              # 上書き保存 was chosen for this item
            elif os.path.exists(path):
                if QtWidgets.QMessageBox.question(self, "SIMFLOW", "「%s」はもうあります。上書きしますか？"
                                                  % self.ed_name.text().strip()) != QtWidgets.QMessageBox.Yes:
                    return
                over = True
            data = lib.capture(self.session, gid, cols, self.ed_name.text(), self.ed_comment.text())
            seq = []
            if self.chk_seq.isChecked():
                if self.seq_range != self.wanted_range():
                    self.grab_sequence()
                seq = self.seq
                if seq:
                    data["sequence"] = dict(self.seq_info)
            lib.write_item(path, data, self.thumb, overwrite=over, seq=seq)
        except (lib.LibraryError, SimflowError, OSError) as exc:
            QtWidgets.QMessageBox.warning(self, "SIMFLOW", str(exc))
            return
        self.saved = path
        self.accept()
