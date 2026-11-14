# -*- coding: utf-8 -*-
"""Presets (spec 4.3).

* Built-in presets: ``simflow/presets/default_presets.json`` (read only).
* User presets: ``<MAYA_APP_DIR>/simflow/presets.json`` (added / overwritten /
  deleted from the UI).  The file is plain JSON.
* Sharing: :meth:`PresetManager.export` writes chosen user presets to a file
  of their own; :meth:`~PresetManager.read_file` + :meth:`~PresetManager.import_entries`
  add a file's presets to the user's (never replacing the whole user file).

Pure Python: the user file path is passed in, so this module is testable
without Maya.
"""
from __future__ import annotations

import json
import os

from . import curve
from .settings import PRESET_FIELDS

BUILTIN_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "presets", "default_presets.json")


class PresetError(ValueError):
    """User facing preset problem."""


def default_user_path():
    import maya.cmds as cmds
    return os.path.join(cmds.internalVar(userAppDir=True), "simflow", "presets.json")


def _read(path):
    if not path or not os.path.exists(path):
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError) as exc:
        raise PresetError("プリセットのファイルを読めません: %s (%s)" % (path, exc))
    if not isinstance(data, dict):
        raise PresetError("プリセットのファイルではありません: %s" % path)
    return {k: v for k, v in data.items() if isinstance(v, dict) and "values" in v}


class PresetManager(object):

    def __init__(self, user_path=None, builtin_path=BUILTIN_PATH):
        self.user_path = user_path
        self.builtin_path = builtin_path

    def builtin(self):
        return _read(self.builtin_path)

    def user(self):
        return _read(self.user_path)

    def names(self):
        """[(key, label, is_user)] built-ins first."""
        out = [(k, v.get("label", k), False) for k, v in self.builtin().items()]
        out += [(k, v.get("label", k), True) for k, v in self.user().items()]
        return out

    def values(self, key):
        user = self.user()
        if key in user:
            return dict(user[key]["values"])
        builtin = self.builtin()
        if key in builtin:
            return dict(builtin[key]["values"])
        raise PresetError("プリセット「%s」が見つかりません" % key)

    def apply(self, key, settings):
        """Apply preset values to a SimSettings (in place).  Returns the values."""
        values = self.values(key)
        for k, v in values.items():
            if k in PRESET_FIELDS:
                setattr(settings, k, tuple(v) if isinstance(v, list) else v)
        if "air_drag" not in values:
            settings.air_drag = 0.0           # presets made before air drag existed
        if "stiffness_curve" in values:
            settings.stiffness_curve = curve.normalise(values["stiffness_curve"])
        else:
            # a preset without a curve: the same stiffness along the whole chain
            settings.stiffness_curve = curve.FLAT
        return values

    def save(self, name, settings):
        name = (name or "").strip()
        if not name:
            raise PresetError("プリセット名を入力してください")
        if name in self.builtin() or name in {v.get("label") for v in self.builtin().values()}:
            raise PresetError("「%s」は標準のプリセットと同じ名前です。別の名前にしてください" % name)
        data = self.user()
        values = {}
        for k in PRESET_FIELDS:
            v = getattr(settings, k)
            values[k] = list(v) if isinstance(v, tuple) else v
        data[name] = {"label": name, "values": values}
        self._write(data)
        return name

    def delete(self, key):
        data = self.user()
        if key not in data:
            raise PresetError("ユーザーのプリセット「%s」はありません（標準のプリセットは削除できません）" % key)
        del data[key]
        self._write(data)

    # ---- sharing (書き出し / 読み込み) -------------------------------------------
    def export(self, keys, path):
        """Write the user presets ``keys`` to a file of their own (the same
        format as the user file, so a whole presets.json can be read too)."""
        data = self.user()
        keys = list(keys)
        if not keys:
            raise PresetError("書き出すプリセットを選んでください")
        missing = [k for k in keys if k not in data]
        if missing:
            raise PresetError("ユーザーのプリセット「%s」はありません（標準のプリセットは書き出せません）"
                              % "」「".join(missing))
        out = {k: data[k] for k in keys}
        out["_format"] = FILE_FORMAT
        _write_json(path, out)
        return keys

    @staticmethod
    def read_file(path):
        """The presets in a shared file: {name: entry}.  Raises PresetError when
        the file has none."""
        if not path or not os.path.exists(path):
            raise PresetError("ファイルがありません: %s" % path)
        entries = {}
        for k, v in _read(path).items():
            name = str(k).strip()
            if name and isinstance(v.get("values"), dict):
                entries[name] = {"label": name, "values": dict(v["values"])}
        if not entries:
            raise PresetError("このファイルには SIMFLOW のプリセットが入っていません: %s" % path)
        return entries

    def conflicts(self, entries):
        """Names in ``entries`` that a user preset already has."""
        user = self.user()
        return [k for k in entries if k in user]

    def _builtin_names(self):
        b = self.builtin()
        return set(b) | {v.get("label") for v in b.values()}

    def import_entries(self, entries, overwrite=False):
        """Add ``entries`` (from :meth:`read_file`) to the user presets.  A name
        the user already has is overwritten (``overwrite``) or added under a
        new name 「名前 (2)」; a built-in preset's name is always renamed.
        Returns [(name in the file, saved name)]."""
        data = self.user()
        builtin = self._builtin_names()
        done = []
        for name, entry in entries.items():
            new = name
            if new in builtin or (new in data and not overwrite):
                n = 2
                while ("%s (%d)" % (name, n)) in data or ("%s (%d)" % (name, n)) in builtin:
                    n += 1
                new = "%s (%d)" % (name, n)
            data[new] = {"label": new, "values": dict(entry["values"])}
            done.append((name, new))
        self._write(data)
        return done

    def _write(self, data):
        if not self.user_path:
            raise PresetError("ユーザープリセットの保存先が決まっていません")
        _write_json(self.user_path, data)


FILE_FORMAT = "SIMFLOW presets"


def _write_json(path, data):
    folder = os.path.dirname(path)
    tmp = path + ".tmp"
    try:
        if folder and not os.path.isdir(folder):
            os.makedirs(folder)
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2, sort_keys=True)
        os.replace(tmp, path)
    except OSError as exc:
        raise PresetError("プリセットのファイルに書き込めません: %s (%s)" % (path, exc))
