# -*- coding: utf-8 -*-
"""SIMFLOW のインストーラー。

このファイルを Maya 2025 のビューポートにドラッグ＆ドロップしてください。

ドロップすると「SIMFLOW のインストール」の画面が出ます。
* GitHub（https://github.com/ogshaw03/simflow）にあるバージョンの一覧から、入れるバージョンを選べます
  （新しいものも古いものも選べます）。選んだバージョンは <ユーザーの Maya フォルダ>/simflow/versions/ にダウンロードします。
* このファイルの隣に simflow フォルダがあるときは、そのフォルダをそのまま使うこともできます（インターネットなし）。

インストールでは、次のことをします。
1. Maya のモジュールファイル <ユーザーの Maya フォルダ>/modules/simflow.mod を作ります。
   これで Maya を再起動しても simflow が読み込めます（ドッキングした SIMFLOW パネルの復元にも必要です）。
2. 今開いているシェルフに「SIMFLOW」ボタンを追加します。
3. SIMFLOW を開きます（開いていた SIMFLOW は閉じて、選んだバージョンで開き直します）。

SIMFLOW の右上の ⋮ →「バージョン...」からも、同じ画面を開けます。
"""
from __future__ import annotations

import json
import os
import re
import shutil
import sys
import urllib.parse
import urllib.request

TOOL = "SIMFLOW"
DOC_TAG = "SIMFLOW"
MOD_NAME = "simflow.mod"
HERE = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
GITHUB_REPO = "ogshaw03/simflow"
GITHUB_BRANCH = "main"
API = "https://api.github.com/repos/" + GITHUB_REPO
RAW = "https://raw.githubusercontent.com/%s/%s/" % (GITHUB_REPO, GITHUB_BRANCH)
TIMEOUT = 20

LAUNCH = '''import sys
_p = r"{root}"
if _p not in sys.path:
    sys.path.append(_p)
import simflow
simflow.show()
'''


# ---------------------------------------------------------------------------
# versions
# ---------------------------------------------------------------------------
def version_of(root):
    """__version__ of the simflow folder under ``root`` (None when there is none)."""
    try:
        with open(os.path.join(root, "simflow", "__init__.py"), encoding="utf-8") as f:
            for line in f:
                if line.startswith("__version__"):
                    return line.split("=")[1].strip().strip("\"'")
    except OSError:
        pass
    return None


def version_key(v):
    return tuple(int(x) for x in re.findall(r"\d+", v or ""))


def is_dev(root):
    """The development source (the release folders have no tests)."""
    return os.path.isdir(os.path.join(root, "simflow", "tests"))


def _get(url):
    req = urllib.request.Request(url, headers={"User-Agent": "SIMFLOW-installer",
                                               "Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
        return r.read()


def remote_versions():
    """Versions on GitHub, newest first: the release folders v<version>/."""
    items = json.loads(_get(API + "/contents/?ref=" + GITHUB_BRANCH).decode("utf-8"))
    vers = [d["name"][1:] for d in items if d.get("type") == "dir" and re.match(r"^v\d+(\.\d+)*$", d.get("name", ""))]
    return sorted(vers, key=version_key, reverse=True)


def release_notes():
    """{version: text} from the GitHub release pages (empty when none / offline)."""
    try:
        rels = json.loads(_get(API + "/releases?per_page=100").decode("utf-8"))
    except Exception:
        return {}
    return {r.get("tag_name", "").lstrip("v"): (r.get("body") or "").strip() for r in rels if r.get("tag_name")}


NOTES_FILE = "変更点.txt"           # in each release folder (make_release.py writes it from CHANGELOG.md)


def folder_notes(version):
    """The 変更点.txt of the release folder v<version>/ on GitHub ("" when none)."""
    try:
        return _get(RAW + urllib.parse.quote("v%s/%s" % (version, NOTES_FILE))).decode("utf-8").strip()
    except Exception:
        return ""


def local_notes(root):
    try:
        with open(os.path.join(root, NOTES_FILE), encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def versions_dir():
    import maya.cmds as cmds
    return os.path.join(cmds.internalVar(userAppDir=True), "simflow", "versions").replace("\\", "/")


def download(version, progress=None, dest=None):
    """Download the release folder v<version>/ from GitHub to
    <versions_dir>/v<version>/ and return that folder.  An existing complete
    download is reused.  ``progress(done, total)``."""
    dest = dest or versions_dir()
    target = os.path.join(dest, "v" + version).replace("\\", "/")
    if version_of(target) == version and os.path.isfile(os.path.join(target, "install_simflow.py")):
        return target
    tree = json.loads(_get(API + "/git/trees/%s?recursive=1" % GITHUB_BRANCH).decode("utf-8"))
    prefix = "v%s/" % version
    files = [t["path"] for t in tree.get("tree", []) if t.get("type") == "blob" and t["path"].startswith(prefix)]
    if not files:
        raise RuntimeError("GitHub に v%s がありません" % version)
    if tree.get("truncated"):
        raise RuntimeError("GitHub のファイルの一覧が大きすぎて、全部を取れませんでした")
    part = target + ".part"
    if os.path.isdir(part):
        shutil.rmtree(part)
    for i, path in enumerate(files):
        data = _get(RAW + urllib.parse.quote(path))
        out = os.path.join(part, path[len(prefix):])
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "wb") as f:
            f.write(data)
        if progress:
            progress(i + 1, len(files))
    if version_of(part) != version:
        shutil.rmtree(part, ignore_errors=True)
        raise RuntimeError("ダウンロードした v%s の中身が正しくありません（simflow/__init__.py のバージョンが %s）"
                           % (version, version_of(part)))
    if os.path.isdir(target):
        shutil.rmtree(target)
    os.rename(part, target)
    return target


# ---------------------------------------------------------------------------
# module file / shelf
# ---------------------------------------------------------------------------
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


def _write_mod(root):
    path = mod_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("+ %s %s %s\n" % (TOOL, version_of(root) or "0.0.0", root))
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


def _menu_items(root):
    """Right-click items of the shelf button."""
    dev = globals().get("_dev_menu_items")
    return dev(root) if dev is not None and is_dev(root) else []




def _add_button(root):
    import maya.cmds as cmds
    top = _shelf_top()
    shelf = cmds.tabLayout(top, query=True, selectTab=True)
    cmds.setParent(top + "|" + shelf)
    items = _menu_items(root)
    kwargs = dict(label=TOOL, annotation="SIMFLOW（揺れもののシミュレーション）を開きます",
                  command=LAUNCH.format(root=root), sourceType="python", docTag=DOC_TAG)
    if items:
        kwargs.update(annotation=kwargs["annotation"] + "。右クリック：" + "・".join(t for t, _ in items),
                      menuItem=items, menuItemPython=list(range(len(items))))
    icon = root + "/simflow/icons/simflow.png"
    if os.path.isfile(icon):
        kwargs["image"] = icon
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


def _close_panel():
    import maya.cmds as cmds
    ctrl = "SimflowMainWindowWorkspaceControl"
    if cmds.workspaceControl(ctrl, query=True, exists=True):
        cmds.workspaceControl(ctrl, edit=True, close=True)
        cmds.deleteUI(ctrl, control=True)


def _load(root):
    """Use the simflow under ``root`` in this Maya session: other SIMFLOW
    folders leave sys.path and the loaded modules are dropped."""
    norm = os.path.normcase(os.path.normpath(root))
    sys.path[:] = [p for p in sys.path if os.path.normcase(os.path.normpath(p or ".")) == norm
                   or not os.path.isfile(os.path.join(p or ".", "simflow", "__init__.py"))]
    if root not in sys.path:
        sys.path.insert(0, root)
    for k in [k for k in list(sys.modules) if k == "simflow" or k.startswith("simflow.")]:
        del sys.modules[k]
    import simflow
    return simflow


# ---------------------------------------------------------------------------
def install(root=HERE, launch=True):
    """Register ``root`` (a folder with simflow/), add the shelf button and
    (optionally) open SIMFLOW from it."""
    root = root.replace("\\", "/")
    if not version_of(root):
        raise RuntimeError("simflow フォルダが見つかりません: %s" % root)
    _close_panel()
    path = _write_mod(root)
    _remove_buttons()
    shelf, _button = _add_button(root)
    simflow = _load(root)
    _message("SIMFLOW v%s をインストールしました。\nシェルフ「%s」にボタンを追加しました。" % (version_of(root), shelf))
    print("[SIMFLOW] モジュールファイル: %s（%s）" % (path, root))
    if launch:
        simflow.show()
    return path


def uninstall():
    """Remove the module file and the shelf buttons (downloaded versions are kept)."""
    import maya.cmds as cmds
    path = mod_path()
    if os.path.isfile(path):
        os.remove(path)
    removed = _remove_buttons()
    cmds.saveAllShelves(_shelf_top())
    _close_panel()
    _message("SIMFLOW をアンインストールしました。\nシェルフのボタン %d 個と、モジュールファイルを削除しました。" % removed)
    return removed


# ---------------------------------------------------------------------------
# the dialog
# ---------------------------------------------------------------------------
def _main_window():
    from PySide6 import QtWidgets
    from shiboken6 import wrapInstance
    import maya.OpenMayaUI as omui
    return wrapInstance(int(omui.MQtUtil.mainWindow()), QtWidgets.QWidget)


def make_dialog(parent=None, remote=None, notes=None):
    """The 「SIMFLOW のインストール」 dialog.  ``remote`` / ``notes``: the
    GitHub versions / release notes (read from GitHub when None)."""
    from PySide6 import QtCore, QtWidgets

    dlg = QtWidgets.QDialog(parent or _main_window())
    dlg.setWindowTitle("SIMFLOW のインストール")
    dlg.resize(460, 420)
    lay = QtWidgets.QVBoxLayout(dlg)
    current = installed_root()
    cur_ver = version_of(current) if current else None
    if current:
        now = "インストール済み: v%s\n%s" % (cur_ver or "?", current)
    else:
        now = "まだインストールされていません。"
    lay.addWidget(QtWidgets.QLabel(now))
    lay.addWidget(QtWidgets.QLabel("入れるバージョンを選んでください。"))
    lst = QtWidgets.QListWidget()
    lay.addWidget(lst, 1)
    info = QtWidgets.QPlainTextEdit()
    info.setReadOnly(True)
    info.setMaximumHeight(130)
    lay.addWidget(info)
    error = ""
    if remote is None:
        try:
            remote = remote_versions()
        except Exception as exc:
            remote, error = [], str(exc)
    if notes is None:
        notes = release_notes() if remote else {}

    def add(text, kind, value, note):
        it = QtWidgets.QListWidgetItem(text)
        it.setData(QtCore.Qt.UserRole, (kind, value, note))
        lst.addItem(it)
        return it

    def same(a, b):
        return a and b and os.path.normcase(os.path.normpath(a)) == os.path.normcase(os.path.normpath(b))

    for i, v in enumerate(remote):
        tags = (["最新"] if i == 0 else []) + (["インストール済み"] if v == cur_ver and current
                                               and not is_dev(current) else [])
        # the release page's text, else the folder's 変更点.txt (read when chosen)
        add("v%s%s" % (v, ("（%s）" % "・".join(tags)) if tags else ""), "remote", v, notes.get(v) or None)
    local = version_of(HERE)
    if local:
        tags = ["このファイルのフォルダ"] + (["開発用"] if is_dev(HERE) else []) + (["インストール済み"] if same(current, HERE) else [])
        text = "ダウンロードせずに、このフォルダをそのまま使います:\n%s" % HERE
        if local_notes(HERE):
            text = local_notes(HERE) + "\n\n" + text
        add("v%s（%s）" % (local, "・".join(tags)), "local", HERE, text)
    if not remote:
        msg = "GitHub のバージョンの一覧を読めませんでした。" + ("\n" + error if error else "")
        it = add("（GitHub に接続できません）", "none", None, msg)
        it.setFlags(QtCore.Qt.NoItemFlags)

    def show_notes(it, _prev=None):
        if it is None:
            info.setPlainText("")
            return
        kind, value, note = it.data(QtCore.Qt.UserRole)
        if note is None:
            note = folder_notes(value) or "（変更点の説明はありません）"
            note += "\n\nGitHub の v%s/ をダウンロードしてインストールします。" % value
            it.setData(QtCore.Qt.UserRole, (kind, value, note))
        info.setPlainText(note)
    lst.currentItemChanged.connect(show_notes)
    for i in range(lst.count()):
        if lst.item(i).flags() & QtCore.Qt.ItemIsEnabled:
            lst.setCurrentRow(i)
            break

    row = QtWidgets.QHBoxLayout()
    b_install = QtWidgets.QPushButton("インストール")
    b_uninstall = QtWidgets.QPushButton("アンインストール")
    b_uninstall.setEnabled(current is not None)
    b_close = QtWidgets.QPushButton("閉じる")
    row.addWidget(b_uninstall)
    row.addStretch(1)
    row.addWidget(b_install)
    row.addWidget(b_close)
    lay.addLayout(row)
    dlg.result_root = None

    def do_install():
        it = lst.currentItem()
        if it is None or not (it.flags() & QtCore.Qt.ItemIsEnabled):
            return
        kind, value, _note = it.data(QtCore.Qt.UserRole)
        root = value
        try:
            if kind == "remote":
                bar = QtWidgets.QProgressDialog("v%s をダウンロードしています..." % value, None, 0, 100, dlg)
                bar.setWindowTitle("SIMFLOW")
                bar.setMinimumDuration(0)

                def progress(done, total):
                    bar.setValue(int(100.0 * done / total))
                    QtWidgets.QApplication.processEvents()
                try:
                    root = download(value, progress)
                finally:
                    bar.close()
            install(root)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(dlg, "SIMFLOW", "インストールできませんでした:\n%s" % exc)
            return
        dlg.result_root = root
        dlg.accept()

    def do_uninstall():
        if QtWidgets.QMessageBox.question(dlg, "SIMFLOW", "SIMFLOW をアンインストールしますか？\n"
                                          "（シェルフのボタンとモジュールファイルを削除します）") \
                != QtWidgets.QMessageBox.Yes:
            return
        uninstall()
        dlg.accept()

    b_install.clicked.connect(do_install)
    b_uninstall.clicked.connect(do_uninstall)
    b_close.clicked.connect(dlg.reject)
    lst.itemDoubleClicked.connect(lambda _it: do_install())
    dlg.list_widget, dlg.info, dlg.install_button = lst, info, b_install
    return dlg


def show_dialog(parent=None):
    return make_dialog(parent).exec()


def onMayaDroppedPythonFile(obj):
    """Called by Maya when this file is dropped onto the viewport."""
    show_dialog()
    return True
