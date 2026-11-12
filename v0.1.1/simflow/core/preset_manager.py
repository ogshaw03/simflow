# -*- coding: utf-8 -*-
"""Presets (spec 4.3).

* Built-in presets: ``simflow/presets/default_presets.json`` (read only).
* User presets: ``<MAYA_APP_DIR>/simflow/presets.json`` (added / overwritten /
  deleted from the UI).  The file is plain JSON so presets can be shared.

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

    def _write(self, data):
        if not self.user_path:
            raise PresetError("ユーザープリセットの保存先が決まっていません")
        folder = os.path.dirname(self.user_path)
        if folder and not os.path.isdir(folder):
            os.makedirs(folder)
        tmp = self.user_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2, sort_keys=True)
        os.replace(tmp, self.user_path)
