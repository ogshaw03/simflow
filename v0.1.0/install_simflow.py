# -*- coding: utf-8 -*-
"""SIMFLOW のインストーラー。

このファイルを Maya 2025 のビューポートにドラッグ＆ドロップしてください。

ドロップすると、次のことをします。
1. Maya のモジュールファイル <ユーザーの Maya フォルダ>/modules/simflow.mod を作ります。
   これで Maya を再起動しても simflow が読み込めます（ドッキングした SIMFLOW パネルの復元にも必要です）。
   SIMFLOW のファイルはコピーしません。このファイルがあるフォルダを、そのまま使います。
2. 今開いているシェルフに「SIMFLOW」ボタンを追加します（右クリックで「読み込み直して起動」）。
3. SIMFLOW を開きます。

すでにインストールされているときは、「再インストール」と「アンインストール」を選べます。
"""
from __future__ import annotations

import os
import sys

TOOL = "SIMFLOW"
DOC_TAG = "SIMFLOW"
MOD_NAME = "simflow.mod"
ROOT = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
ICON = ROOT + "/simflow/icons/simflow.png"

LAUNCH = '''import sys
_p = r"{root}"
if _p not in sys.path:
    sys.path.append(_p)
import simflow
simflow.show()
'''

RELOAD = '''import sys
_p = r"{root}"
if _p not in sys.path:
    sys.path.append(_p)
for _k in [k for k in list(sys.modules) if k == "simflow" or k.startswith("simflow.")]:
    del sys.modules[_k]
import simflow
simflow.show()
'''


# ---------------------------------------------------------------------------
def _version():
    try:
        with open(os.path.join(ROOT, "simflow", "__init__.py"), encoding="utf-8") as f:
            for line in f:
                if line.startswith("__version__"):
                    return line.split("=")[1].strip().strip("\"'")
    except OSError:
        pass
    return "0.0.0"


def mod_path():
    import maya.cmds as cmds
    return os.path.join(cmds.internalVar(userAppDir=True), "modules", MOD_NAME).replace("\\", "/")


def installed_root():
    """Root written in an existing simflow.mod (None when not installed)."""
    path = mod_path()
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("+"):
                parts = line.split(None, 3)
                return parts[3] if len(parts) > 3 else ""
    return ""


def _write_mod():
    path = mod_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("+ %s %s %s\n" % (TOOL, _version(), ROOT))
        f.write("PYTHONPATH+:=.\n")
    return path


def _shelf_top():
    import maya.mel as mel
    return mel.eval("$_sf_tmp = $gShelfTopLevel")


def _remove_buttons():
    import maya.cmds as cmds
    top = _shelf_top()
    removed = 0
    for shelf in cmds.tabLayout(top, query=True, childArray=True) or []:
        for b in cmds.shelfLayout(shelf, query=True, childArray=True) or []:
            if cmds.objectTypeUI(b) == "shelfButton" and cmds.shelfButton(b, query=True, docTag=True) == DOC_TAG:
                cmds.deleteUI(b)
                removed += 1
    return removed


def _add_button():
    import maya.cmds as cmds
    top = _shelf_top()
    shelf = cmds.tabLayout(top, query=True, selectTab=True)
    cmds.setParent(top + "|" + shelf)
    kwargs = dict(label=TOOL, annotation="SIMFLOW（揺れもののシミュレーション）を開きます。右クリック：読み込み直して起動",
                  command=LAUNCH.format(root=ROOT), sourceType="python", docTag=DOC_TAG,
                  menuItem=[("読み込み直して起動", RELOAD.format(root=ROOT))], menuItemPython=[0])
    if os.path.isfile(ICON):
        kwargs["image"] = ICON
    else:
        kwargs.update(image="pythonFamily.png", imageOverlayLabel="SIM")
    button = cmds.shelfButton(**kwargs)
    cmds.saveAllShelves(top)
    return shelf, button


def _message(text):
    import maya.cmds as cmds
    print("[SIMFLOW] " + text.replace("\n", " "))
    try:
        cmds.inViewMessage(assistMessage=text.replace("\n", "<br>"), position="midCenter", fade=True,
                           fadeStayTime=4000)
    except RuntimeError:
        pass


def _ask(text, buttons):
    """Modal question on Maya's main window.  Returns the clicked label."""
    from PySide6 import QtWidgets
    from shiboken6 import wrapInstance
    import maya.OpenMayaUI as omui
    parent = wrapInstance(int(omui.MQtUtil.mainWindow()), QtWidgets.QWidget)
    box = QtWidgets.QMessageBox(parent)
    box.setWindowTitle("SIMFLOW のインストール")
    box.setText(text)
    made = {box.addButton(label, role): label for label, role in buttons}
    box.exec()
    return made.get(box.clickedButton())


# ---------------------------------------------------------------------------
def install(launch=True):
    """Register the module, add the shelf button and (optionally) open SIMFLOW."""
    if not os.path.isfile(os.path.join(ROOT, "simflow", "__init__.py")):
        raise RuntimeError("simflow フォルダが見つかりません: %s（install_simflow.py は simflow フォルダの隣に置いてください）" % ROOT)
    path = _write_mod()
    if ROOT not in sys.path:
        sys.path.append(ROOT)
    _remove_buttons()
    shelf, _button = _add_button()
    _message("SIMFLOW をインストールしました。\nシェルフ「%s」にボタンを追加しました。" % shelf)
    print("[SIMFLOW] モジュールファイル: %s" % path)
    if launch:
        import simflow
        simflow.show()
    return path


def uninstall():
    """Remove the module file and the shelf buttons (the SIMFLOW files are kept)."""
    import maya.cmds as cmds
    path = mod_path()
    if os.path.isfile(path):
        os.remove(path)
    removed = _remove_buttons()
    cmds.saveAllShelves(_shelf_top())
    ctrl = "SimflowMainWindowWorkspaceControl"
    if cmds.workspaceControl(ctrl, query=True, exists=True):
        cmds.workspaceControl(ctrl, edit=True, close=True)
        cmds.deleteUI(ctrl, control=True)
    _message("SIMFLOW をアンインストールしました。\nシェルフのボタン %d 個と、モジュールファイルを削除しました。" % removed)
    return removed


def onMayaDroppedPythonFile(obj):
    """Called by Maya when this file is dropped onto the viewport."""
    from PySide6 import QtWidgets
    current = installed_root()
    if current is None:
        install()
        return True
    if os.path.normcase(os.path.normpath(current)) == os.path.normcase(os.path.normpath(ROOT)):
        text = "SIMFLOW はすでにインストールされています。\n%s" % ROOT
    else:
        text = ("SIMFLOW は別の場所からインストールされています。\n現在: %s\nこのファイル: %s\n\n"
                "「再インストール」で、このファイルの場所に切り替えます。" % (current, ROOT))
    choice = _ask(text, [("再インストール", QtWidgets.QMessageBox.AcceptRole),
                         ("アンインストール", QtWidgets.QMessageBox.DestructiveRole),
                         ("キャンセル", QtWidgets.QMessageBox.RejectRole)])
    if choice == "再インストール":
        install()
    elif choice == "アンインストール":
        uninstall()
    return True
