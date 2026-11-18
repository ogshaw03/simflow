# -*- coding: utf-8 -*-
"""Bake simulated results back onto the controllers (spec 8).

Modes
-----
``additive`` (default)
    A SIMFLOW-owned *additive* animation layer (rotation accumulation "By
    Component") receives ``final - base`` per channel.  Zero keys with step
    tangents at ``start-1`` / ``end+1`` keep frames outside the range
    untouched.
``override``
    A SIMFLOW-owned *override* layer receives the final channel values; its
    weight is keyed 0 / 1 / 1 / 0 (step) around the range so frames outside
    the range pass the original animation through.
``overwrite``
    Writes into the base curves.  Only on explicit request.  The original
    curves are duplicated as tagged backups first (static channels store
    their value) so Restore Original can bring them back.

Safety
------
* Locked channels, locked nodes and channels driven by anything other than
  time-based animCurves / animation layers (constraints, expressions, driven
  keys, pairBlends ...) abort the bake before anything is written.
* A user-created layer with the requested name is never written into.
* Everything runs in one undo chunk; any failure (including failed
  validation) undoes the whole operation.
* After writing, the controller is re-evaluated at every keyed frame and at
  the guard frames; the world matrices must match the simulation.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import List

import maya.cmds as cmds

from ..utils import math_utils as mu
from ..utils.logger import get_logger
from ..utils.maya_utils import (SimflowError, add_string_attr, cm_to_ui, get_string_attr,
                                rad_to_ui, suspend_refresh, undo_chunk)
from . import rig_preview, scene_manager
from . import transform_utils as tu
from .settings import AIM_AXES, BAKE_MODES

LOG = get_logger("bake")

MODE_NAMES = {"additive": "加算レイヤー", "override": "オーバーライドレイヤー", "overwrite": "上書き"}

T_ATTRS = ("translateX", "translateY", "translateZ")
R_ATTRS = ("rotateX", "rotateY", "rotateZ")
POS_TOLERANCE = 1.0e-3       # cm
ROT_TOLERANCE = 1.0e-4       # rad (~0.006 deg)


@dataclass
class BakeReport:
    mode: str
    layer: str = ""
    entries: List[str] = field(default_factory=list)
    keyed_frames: int = 0
    max_pos_error: float = 0.0
    max_rot_error: float = 0.0          # radians
    max_guard_pos_error: float = 0.0
    max_guard_rot_error: float = 0.0
    max_mask_deviation: float = 0.0     # orientation lost through channel masks / shear (rad)
    max_aim_deviation: float = 0.0      # aim-axis direction lost (rad)
    worst: str = ""                     # where the largest validation error is
    worst_guard: str = ""               # where the largest out-of-range change is
    warnings: List[str] = field(default_factory=list)

    def summary(self):
        lines = ["出力先: %s%s" % (MODE_NAMES.get(self.mode, self.mode),
                                     ("（レイヤー %s）" % self.layer) if self.layer else ""),
                 "コントローラー: %s" % "、".join(self.entries),
                 "チャンネルあたりのキー数: %d" % self.keyed_frames,
                 "検証の最大誤差: 位置 %.6f / 回転 %.5f 度" % (
                     self.max_pos_error, math.degrees(self.max_rot_error)),
                 "範囲外の最大変化: 位置 %.6f / 回転 %.5f 度" % (
                     self.max_guard_pos_error, math.degrees(self.max_guard_rot_error))]
        if self.worst and (self.max_pos_error > POS_TOLERANCE or self.max_rot_error > ROT_TOLERANCE):
            lines.append("最大誤差の場所: %s" % self.worst)
        if self.worst_guard and (self.max_guard_pos_error > POS_TOLERANCE or
                                 self.max_guard_rot_error > ROT_TOLERANCE):
            lines.append("範囲外の最大変化の場所: %s" % self.worst_guard)
        if self.max_mask_deviation > 1.0e-3:
            lines.append("注意: ベイクした姿勢は、制約なしのシミュレーション結果と最大 %.2f 度異なります"
                         "（先端の軸の向きの差: %.2f 度）。原因は、無効にした移動・回転の軸か、"
                         "非一様スケールの親です（その場合ワールドでの剛体回転にはシアーが必要で、"
                         "回転チャンネルでは表せません。SIMFLOW は先端の軸の向きを正確に保ち、"
                         "軸まわりのねじれを近似します）。" % (
                             math.degrees(self.max_mask_deviation),
                             math.degrees(self.max_aim_deviation)))
        lines.extend("警告: %s" % w for w in self.warnings)
        return "\n".join(lines)


def enabled_attrs(settings):
    attrs = [a for a, on in zip(T_ATTRS, settings.translate) if on]
    attrs += [a for a, on in zip(R_ATTRS, settings.rotate) if on]
    return attrs


# ---------------------------------------------------------------------------
# Channel checks
# ---------------------------------------------------------------------------

def check_channels(node, attrs, mode):
    """Return a list of problems that make baking unsafe."""
    problems = []
    label = node.split("|")[-1]
    if cmds.lockNode(node, query=True, lock=True)[0]:
        problems.append("%s: ノードがロックされています" % label)
    for attr in attrs:
        plug = "%s.%s" % (node, attr)
        if cmds.getAttr(plug, lock=True):
            problems.append("%s: チャンネルがロックされています" % plug)
            continue
        sources = cmds.listConnections(plug, source=True, destination=False,
                                       skipConversionNodes=True) or []
        if not sources:
            continue
        src = sources[0]
        src_type = cmds.nodeType(src)
        if src_type.startswith("animCurveT"):
            if cmds.listConnections(src + ".input", source=True, destination=False):
                problems.append("%s: アニメーションカーブの入力が別のノードで制御されています（%s）" % (plug, src))
            elif cmds.referenceQuery(src, isNodeReferenced=True) and mode == "overwrite":
                problems.append("%s: アニメーションカーブがリファレンスの中にあります。レイヤーに出力してください"
                                % plug)
        elif src_type.startswith("animBlendNode"):
            if mode == "overwrite":
                problems.append("%s: チャンネルがアニメーションレイヤーに入っています。上書きはベースレイヤーしか"
                                "変えられないため、レイヤーに出力してください" % plug)
        else:
            problems.append("%s: %s（%s）で制御されています。競合するためベイクしません"
                            % (plug, src, src_type))
    return problems


# ---------------------------------------------------------------------------
# Manager
# ---------------------------------------------------------------------------

class BakeManager(object):

    def __init__(self, controller_manager, simulation_manager):
        self.controllers = controller_manager
        self.sim = simulation_manager

    # ------------------------------------------------------------------
    def bake(self, batches, options, progress=None, save_callback=None):
        """Bake swing groups (``batches``: simulation_manager.Batch list)."""
        entries = [e for b in batches for e in b.entries]
        if options.mode not in BAKE_MODES:
            raise SimflowError("不明な出力先です: %r" % options.mode)
        if int(options.sample_step) != options.sample_step or options.sample_step < 1:
            raise SimflowError("キーの間隔は 1 以上の整数にしてください")
        if not entries:
            raise SimflowError("有効なコントローラーが登録されていません。")
        attrs = {}
        for b in batches:
            for e in b.entries:
                attrs[e.id] = enabled_attrs(b.settings)
        for e in entries:
            if not attrs[e.id]:
                raise SimflowError("%s: 移動・回転のチャンネルが 1 つも有効になっていません。" % e.label)
        self._check_descendants(entries)
        self._check_previewed_ancestors(entries)
        key_frames = options.frames()
        report = BakeReport(mode=options.mode)
        snapshot = {e.id: copy.deepcopy(e.bake) for e in entries}
        try:
            self._bake(batches, entries, options, progress, save_callback, attrs, key_frames, report)
        except BaseException:
            for e in entries:                     # scene was undone -> undo python state too
                e.bake = snapshot[e.id]
            raise
        LOG.info("Bake finished:\n%s", report.summary())
        return report

    def _check_descendants(self, entries):
        """A baked controller below one that is re-baked would keep channels
        solved against the OLD parent result -> refuse."""
        ids = {e.id for e in entries}
        paths = {}
        for e in self.controllers.entries:
            try:
                paths[e.id] = self.controllers.resolve(e)
            except SimflowError:
                continue
        stale = []
        for other in self.controllers.entries:
            if other.id in ids or not other.bake or other.id not in paths:
                continue
            for e in entries:
                if e.id in paths and paths[other.id].startswith(paths[e.id] + "|"):
                    stale.append("%s（%s の子）" % (other.label, e.label))
                    break
        if stale:
            raise SimflowError("次のコントローラーはベイク済みで、今回ベイクする親の結果に依存しています。"
                               "有効にして一緒にベイクするか、先に「元に戻す」を実行してください:\n"
                               + "\n".join(stale))

    def _check_previewed_ancestors(self, entries):
        """A controller of another swing group ABOVE the baked ones that is
        only shown by 「リグで見る」 (not baked, not baked now): the bake is
        calculated on its original animation, but the preview keeps moving
        it, so the result would not match what is seen (and baking that
        group later would be refused: its children are baked) -> refuse."""
        lay = rig_preview.layer()
        if not lay:
            return
        previewed = set()
        for plug in cmds.animLayer(lay, query=True, attribute=True) or []:
            previewed.update(cmds.ls(plug.rsplit(".", 1)[0], long=True) or [])
        if not previewed:
            return
        ids = {e.id for e in entries}
        paths = {}
        for e in entries:
            try:
                paths[e.id] = self.controllers.resolve(e)
            except SimflowError:
                pass
        groups = {g.id: g.name for g in self.controllers.groups}
        found = {}
        for other in self.controllers.entries:
            if other.id in ids:
                continue
            try:
                path = self.controllers.resolve(other)
            except SimflowError:
                continue
            if path not in previewed:
                continue
            below = [e for e in entries if e.id in paths and paths[e.id].startswith(path + "|")]
            if below:
                found.setdefault(groups.get(other.group, "?"), set()).update(
                    groups.get(e.group, "?") for e in below)
        if found:
            lines = ["「%s」（%s の上）" % (g, "、".join("「%s」" % c for c in sorted(kids)))
                     for g, kids in sorted(found.items())]
            raise SimflowError(
                "ベイクを中止しました。シーンは変更していません。\n"
                "ベイクするコントローラーの親が、ほかの揺れものの「リグで見る」の表示で動いています（まだベイクしていません）:\n"
                + "\n".join(lines) + "\n\n"
                "子だけをベイクすると、親の元のアニメーションに対して計算するため、見えている動きと合いません。\n"
                "・親も揺らす：親の揺れものにもチェックを入れて、一緒にベイクしてください。\n"
                "・親は揺らさない：「リグで見る」をオフにしてからベイクしてください。")

    def _bake(self, batches, entries, options, progress, save_callback, attrs, key_frames, report):
        with undo_chunk("SIMFLOW Bake", rollback=True), suspend_refresh():
            # 1. remove previous bakes of these controllers so sampling and
            #    the new keys start from the original animation.
            self.restore(entries, keep_layers=True)
            # the bake shows the same result: these controllers leave the
            # preview layer (other groups stay previewed)
            paths = []
            for e in entries:
                try:
                    paths.append(self.controllers.resolve(e))
                except SimflowError:
                    pass
            rig_preview.remove_nodes(paths)

            # 2. safety checks (inside the chunk: failure rolls back step 1)
            problems = []
            nodes = {}
            for entry in entries:
                node = self.controllers.resolve(entry)
                nodes[entry.id] = node
                problems += check_channels(node, attrs[entry.id], options.mode)
            if problems:
                raise SimflowError("ベイクを中止しました。シーンは変更していません:\n" + "\n".join(problems))

            layer = None
            if options.mode in ("additive", "override"):
                layer = self._ensure_layer(options, entries)
                report.layer = layer
                if scene_manager.raise_layer(layer):
                    LOG.info("レイヤー %s をいちばん上に移しました（上にほかのアニメーションレイヤーがあると、"
                             "ベイクが計算の結果と合わなくなるため）", layer)

            # 3. (re)calculate from the original animation
            from .simulation_manager import _SubRange
            from ..solvers.base_solver import Progress
            progress = progress or Progress()
            results = self.sim.calculate(batches, options, _SubRange(progress, 0.0, 0.7))

            # 4. reference evaluation outside the range
            guard_frames = [options.start - 1.0, options.end + 1.0]
            guard_before = {}
            for entry in entries:
                sampler = tu.ControllerSampler(nodes[entry.id])
                guard_before[entry.id] = [sampler.sample(f).world for f in guard_frames]

            # 5. write keys
            for k, entry in enumerate(entries):
                progress.update(0.7 + 0.2 * k / max(1, len(entries)),
                                "キーを打っています：%s（%d / %d）" % (entry.label, k + 1, len(entries)))
                result = results[entry.id]
                by_frame = result.by_frame()
                frs = [by_frame[f] for f in key_frames]
                node = nodes[entry.id]
                if options.mode == "overwrite":
                    self._write_overwrite(entry, node, attrs[entry.id], frs, options)
                else:
                    self._write_layer(entry, node, attrs[entry.id], frs, options, layer)
                report.entries.append(entry.label)
            if options.mode == "override":
                self._key_layer_weight(layer, options)
            report.keyed_frames = len(key_frames)
            # remove SIMFLOW layers emptied by step 1
            for other in scene_manager.tool_layers():
                if other != layer and not cmds.animLayer(other, query=True, attribute=True):
                    scene_manager.delete_nodes([other])

            if layer:
                scene_manager.dirty_layers([layer])
            # 6. validate
            for k, entry in enumerate(entries):
                progress.update(0.9 + 0.1 * k / max(1, len(entries)),
                                "検証中：%s（%d / %d）" % (entry.label, k + 1, len(entries)))
                self._validate(entry, nodes[entry.id], results[entry.id], key_frames,
                               guard_frames, guard_before[entry.id], report)
            if (report.max_pos_error > POS_TOLERANCE or report.max_rot_error > ROT_TOLERANCE or
                    report.max_guard_pos_error > POS_TOLERANCE or
                    report.max_guard_rot_error > ROT_TOLERANCE):
                raise SimflowError("ベイクの検証に失敗したため、シーンを元に戻しました。\n" +
                                   report.summary())
            if save_callback:
                save_callback()
            progress.update(1.0, "完了")

    # ------------------------------------------------------------------
    # layers
    # ------------------------------------------------------------------
    def _ensure_layer(self, options, entries):
        name = options.layer_name.strip() or "SIMFLOW_Bake"
        override = options.mode == "override"
        if cmds.objExists(name):
            if cmds.nodeType(name) != "animLayer":
                raise SimflowError("「%s」はアニメーションレイヤーではないノードとして存在します。" % name)
            if not scene_manager.is_tool_node(name):
                raise SimflowError("アニメーションレイヤー「%s」は SIMFLOW が作成したものではないため、書き込みません。"
                                   "別のレイヤー名を指定してください。" % name)
            if bool(cmds.animLayer(name, query=True, override=True)) != override:
                raise SimflowError("SIMFLOW のレイヤー「%s」は%sです。%sに出力するには別のレイヤー名を指定してください。"
                                   % (name, "オーバーライドレイヤー" if not override else "加算レイヤー",
                                      MODE_NAMES.get(options.mode, options.mode)))
            if override:
                stored = get_string_attr(name, "simflowRange")
                wanted = "%g:%g" % (options.start, options.end)
                others = set(cmds.animLayer(name, query=True, attribute=True) or [])
                if stored and stored != wanted and others:
                    raise SimflowError("オーバーライドレイヤー「%s」は範囲 %s でキーが打たれており、ほかのコントローラーも"
                                       "入っています。同じ範囲にするか、別のレイヤー名を指定してください。" % (name, stored))
            return name
        layer = cmds.animLayer(name, override=override)
        scene_manager.tag(layer, "layer")
        if not override:
            cmds.setAttr(layer + ".rotationAccumulationMode", 0)    # By Component (Euler add)
            cmds.setAttr(layer + ".scaleAccumulationMode", 1)       # Additive
        return layer

    def _write_layer(self, entry, node, attrs, frs, options, layer):
        key_layer_curves(node, attrs, frs, options, layer, additive=options.mode == "additive")
        entry.bake = {"mode": options.mode, "layer": layer, "attrs": list(attrs),
                      "start": options.start, "end": options.end, "step": options.sample_step}

    def _key_layer_weight(self, layer, options):
        plug = layer + ".weight"
        cmds.cutKey(plug, clear=True)
        for f, v in ((options.start - 1.0, 0.0), (options.start, 1.0),
                     (options.end, 1.0), (options.end + 1.0, 0.0)):
            cmds.setKeyframe(plug, time=f, value=v, outTangentType="step")
        add_string_attr(layer, "simflowRange", "%g:%g" % (options.start, options.end))

    # ------------------------------------------------------------------
    # overwrite
    # ------------------------------------------------------------------
    def _write_overwrite(self, entry, node, attrs, frs, options):
        backups = {}
        for attr in attrs:
            plug = "%s.%s" % (node, attr)
            sources = cmds.listConnections(plug, source=True, destination=False,
                                           skipConversionNodes=True) or []
            guards = [(f, cmds.getAttr(plug, time=f)) for f in (options.start - 1.0, options.end + 1.0)]
            if sources:
                dup = cmds.duplicate(sources[0], name="SIMFLOW_backup_%s_%s" % (entry.id, attr))[0]
                scene_manager.tag(dup, "backup", entry.id)
                add_string_attr(dup, "simflowBackupFor", plug)
                backups[attr] = {"curve": dup}
                cmds.cutKey(plug, time=(options.start, options.end), clear=True)
            else:
                backups[attr] = {"static": cmds.getAttr(plug)}
            values = _channel_values(attr, frs, delta=False)
            for fr, v in zip(frs, values):
                cmds.setKeyframe(plug, time=fr.frame, value=v,
                                 inTangentType="auto", outTangentType="auto")
            # guard keys keep the original values just outside the range
            for f, v in guards:
                if not cmds.keyframe(plug, query=True, time=(f, f)):
                    cmds.setKeyframe(plug, time=f, value=v)
        entry.bake = {"mode": "overwrite", "layer": "", "attrs": list(attrs),
                      "start": options.start, "end": options.end, "step": options.sample_step,
                      "backups": backups}

    # ------------------------------------------------------------------
    # validation
    # ------------------------------------------------------------------
    def _validate(self, entry, node, result, key_frames, guard_frames, guard_before, report):
        sampler = tu.ControllerSampler(node)
        by_frame = result.by_frame()
        for f in key_frames:
            fr = by_frame[f]
            world = sampler.sample(f).world
            d, a = tu.matrix_error(world, fr.final_world)
            if (d > POS_TOLERANCE or a > ROT_TOLERANCE) and (d > report.max_pos_error or
                                                              a > report.max_rot_error):
                report.worst = "%s の %g フレーム" % (entry.label, f)
            report.max_pos_error = max(report.max_pos_error, d)
            report.max_rot_error = max(report.max_rot_error, a)
            _, dev = tu.matrix_error(fr.final_world, fr.desired_world)
            report.max_mask_deviation = max(report.max_mask_deviation, dev)
            aim = AIM_AXES[result.settings.get("aim_axis", "+X")]
            a = mu.v_normalize(mu.m_transform_vector(fr.final_world, aim))
            b = mu.v_normalize(mu.m_transform_vector(fr.desired_world, aim))
            report.max_aim_deviation = max(report.max_aim_deviation,
                                           math.acos(max(-1.0, min(1.0, mu.v_dot(a, b)))))
        for f, before in zip(guard_frames, guard_before):
            d, a = tu.matrix_error(sampler.sample(f).world, before)
            if (d > POS_TOLERANCE or a > ROT_TOLERANCE) and (d > report.max_guard_pos_error or
                                                              a > report.max_guard_rot_error):
                report.worst_guard = "%s の %g フレーム" % (entry.label, f)
            report.max_guard_pos_error = max(report.max_guard_pos_error, d)
            report.max_guard_rot_error = max(report.max_guard_rot_error, a)

    # ------------------------------------------------------------------
    # restore
    # ------------------------------------------------------------------
    def restore_original(self, entries, save_callback=None):
        """Restore Original button: one undoable, rolled-back-on-error step."""
        snapshot = {e.id: copy.deepcopy(e.bake) for e in entries}
        try:
            with undo_chunk("SIMFLOW Restore Original", rollback=True):
                restored = self.restore(entries)
                if save_callback:
                    save_callback()
        except BaseException:
            for e in entries:
                e.bake = snapshot[e.id]
            raise
        return restored

    def restore(self, entries, keep_layers=False):
        """Undo previous bakes of ``entries`` (spec 8.3)."""
        restored = []
        for entry in entries:
            state = entry.bake
            if not state:
                continue
            node = self.controllers.resolve(entry)
            if state["mode"] == "overwrite":
                for attr, rec in (state.get("backups") or {}).items():
                    plug = "%s.%s" % (node, attr)
                    if "curve" in rec:
                        curve = rec["curve"]
                        if not cmds.objExists(curve):
                            raise SimflowError("バックアップのカーブ %s が見つからないため、%s を元に戻せません"
                                               % (curve, plug))
                        cmds.cutKey(plug, clear=True)
                        cmds.copyKey(curve)
                        cmds.pasteKey(plug, option="replaceCompletely")
                        scene_manager.delete_nodes([curve])
                    else:
                        cmds.cutKey(plug, clear=True)
                        cmds.setAttr(plug, rec["static"])
            else:
                layer = state.get("layer")
                if layer and cmds.objExists(layer):
                    current = cmds.animLayer(layer, query=True, attribute=True) or []
                    for attr in state.get("attrs", []):
                        plug = "%s.%s" % (node, attr)
                        if plug in current or _short_plug(plug) in current:
                            cmds.animLayer(layer, edit=True, removeAttribute=plug)
                    left = cmds.animLayer(layer, query=True, attribute=True) or []
                    if not left and not keep_layers:
                        scene_manager.delete_nodes([layer])
            entry.bake = None
            restored.append(entry.label)
        return restored


def _layer_curve(layer, plug):
    curves = cmds.animLayer(layer, query=True, findCurveForPlug=plug) or []
    return curves[0] if curves else None


def key_layer_curves(node, attrs, frs, options, layer, additive):
    """Key ``attrs`` of ``node`` in ``layer`` (shared by the bake and the rig preview)."""
    plugs = ["%s.%s" % (node, a) for a in attrs]
    existing = set(cmds.animLayer(layer, query=True, attribute=True) or [])
    to_add = [p for p in plugs if p not in existing and _short_plug(p) not in existing]
    if to_add:
        cmds.animLayer(layer, edit=True, attribute=to_add)
    for attr, plug in zip(attrs, plugs):
        # NOTE: setKeyframe(plug, value=v, animLayer=L) treats ``v`` as the
        # desired *final* value and converts it itself.  To control the raw
        # layer values exactly, the layer's own curve node is keyed.
        curve = _layer_curve(layer, plug)
        if not curve:
            cmds.setKeyframe(plug, time=frs[0].frame, animLayer=layer)
            curve = _layer_curve(layer, plug)
        if not curve:
            raise SimflowError("%s のレイヤー用カーブを作成できませんでした" % plug)
        values = _channel_values(attr, frs, delta=additive)
        for fr, v in zip(frs, values):
            cmds.setKeyframe(curve, time=fr.frame, value=v,
                             inTangentType="auto", outTangentType="auto")
        keep = [fr.frame for fr in frs]
        if additive:
            # zero delta outside the range (step tangents: integer frames exact)
            cmds.setKeyframe(curve, time=options.start - 1.0, value=0.0, outTangentType="step")
            cmds.setKeyframe(curve, time=options.end + 1.0, value=0.0)
            cmds.keyTangent(curve, time=(options.end, options.end), outTangentType="step")
            keep += [options.start - 1.0, options.end + 1.0]
        # drop keys of a previous bake that are not part of this one
        # (cutting the whole curve would delete the layer curve node)
        for t in cmds.keyframe(curve, query=True, timeChange=True) or []:
            if all(abs(t - k) > 1.0e-6 for k in keep):
                cmds.cutKey(curve, time=(t, t), clear=True)


def _short_plug(plug):
    node, attr = plug.rsplit(".", 1)
    return "%s.%s" % (node.split("|")[-1], attr)


def _channel_values(attr, frs, delta):
    """Values in UI units for one channel."""
    out = []
    if attr in T_ATTRS:
        i = T_ATTRS.index(attr)
        for fr in frs:
            v = fr.t[i] - fr.base_t[i] if delta else fr.t[i]
            out.append(cm_to_ui(v))
    else:
        i = R_ATTRS.index(attr)
        for fr in frs:
            v = fr.r[i] - fr.base_r[i] if delta else fr.r[i]
            out.append(rad_to_ui(v))
    return out
