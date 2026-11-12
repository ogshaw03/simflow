# -*- coding: utf-8 -*-
"""UI-independent facade that ties the managers together.

The PySide UI and the integration tests both drive SIMFLOW exclusively
through this class, keeping Maya / solver logic out of the UI.

Controllers are organised in *swing groups* ("揺れもの"): each group has its
own settings and chain / per-controller mode.  For scripts and tests that
only need one group, ``settings`` / ``chain_mode`` / ``chain_auto`` and
``add_selected()`` without a group act on the first group (created on
demand).
"""
from __future__ import annotations

import maya.cmds as cmds

from ..utils.logger import get_logger
from ..utils.maya_utils import SimflowError, playback_range, suspend_refresh, undo_chunk
from . import collider_manager
from . import collision_view
from . import rig_preview
from . import scene_manager
from .bake_manager import BakeManager
from .controller_manager import ControllerManager, SwingGroup
from .preset_manager import PresetManager, default_user_path
from .preview_manager import PreviewManager
from .settings import BakeOptions, SimSettings
from .simulation_manager import Batch, SimulationManager, _SubRange as _Part
from ..solvers.base_solver import Progress

LOG = get_logger("session")

DATA_VERSION = 3


class SimflowSession(object):

    def __init__(self):
        self.controllers = ControllerManager()
        self.sim = SimulationManager(self.controllers)
        self.preview = PreviewManager()
        self.baker = BakeManager(self.controllers, self.sim)
        self.bake_options = BakeOptions()
        self.rig_preview = False          # also drive the real controllers (override layer)
        self.rig_preview_skipped = []     # channels the last rig preview could not key
        # 「リグで見る」 checked, the rig not driven yet (the next 計算 does it)
        self.rig_preview_waiting = False
        start, end = playback_range()
        self.bake_options.start, self.bake_options.end = start, end
        try:
            self.presets = PresetManager(default_user_path())
        except Exception:                # pragma: no cover - Maya not fully up
            self.presets = PresetManager(None)

    # ------------------------------------------------------------------
    # single-group convenience (scripts / tests)
    # ------------------------------------------------------------------
    def _first_group(self):
        if not self.controllers.groups:
            self.controllers.add_group(chain=False)
        return self.controllers.groups[0]

    @property
    def settings(self):
        return self._first_group().settings

    @settings.setter
    def settings(self, value):
        self._first_group().settings = value

    @property
    def chain_mode(self):
        return self._first_group().chain

    @chain_mode.setter
    def chain_mode(self, value):
        self._first_group().chain = bool(value)
        self.sim.invalidate()

    @property
    def chain_auto(self):
        return self._first_group().chain_auto

    @chain_auto.setter
    def chain_auto(self, value):
        self._first_group().chain_auto = bool(value)
        self.sim.invalidate()

    # ------------------------------------------------------------------
    # persistence (scene metadata)
    # ------------------------------------------------------------------
    def save(self):
        scene_manager.save_data({
            "version": DATA_VERSION,
            "groups": self.controllers.groups_to_list(),
            "controllers": self.controllers.to_list(),
            "bake": self.bake_options.to_dict(),
        })

    def load(self):
        """Load state from the scene.  Returns True when metadata existed."""
        data = scene_manager.load_data()
        self.sim.invalidate()
        self.rig_preview = rig_preview.layer() is not None
        self.rig_preview_waiting = False
        if not data:
            self.controllers.clear()
            self.bake_options = BakeOptions()
            self.bake_options.start, self.bake_options.end = playback_range()
            return False
        self.bake_options = BakeOptions.from_dict(data.get("bake"))
        if "groups" in data:
            self.controllers.from_data(data.get("groups"), data.get("controllers"))
        else:
            self._migrate_v2(data)
        return True

    def _migrate_v2(self, data):
        """Phase 2 scenes: one global setting + optional per-controller
        overrides -> one group (overrides are dropped and reported)."""
        g = SwingGroup(name="揺れもの 1", chain=bool(data.get("chain", False)),
                       chain_auto=bool(data.get("chain_auto", True)),
                       settings=SimSettings.from_dict(data.get("settings")))
        ctls = []
        dropped = []
        for d in data.get("controllers") or []:
            d = dict(d)
            if d.get("overrides"):
                dropped.append(d.get("path", "?").split("|")[-1])
            d["group"] = g.id
            ctls.append(d)
        self.controllers.from_data([g.to_dict()], ctls)
        if dropped:
            LOG.warning("以前のコントローラーごとの個別設定は使わなくなりました（%s）。値を変えたい場合は、"
                        "別の揺れものに分けてください。", "、".join(dropped))

    def reload_after_undo(self):
        """Undo / redo changed the metadata: follow the controller / bake
        state and the list of groups, keep unsaved settings of groups that
        still exist."""
        data = scene_manager.load_data() or {}
        current = {g.id: g for g in self.controllers.groups}
        groups = []
        for d in data.get("groups") or []:
            g = current.get(d.get("id")) or SwingGroup.from_dict(d)
            groups.append(g)
        self.controllers.groups = groups
        self.controllers.entries_from_list(data.get("controllers"))
        self.sim.invalidate()
        # checked but not calculated yet: the undo of something else keeps it on
        self.rig_preview = rig_preview.layer() is not None or (self.rig_preview and self.rig_preview_waiting)

    # ------------------------------------------------------------------
    # groups
    # ------------------------------------------------------------------
    def add_group(self, name=None, from_selection=True):
        """New group (copy of nothing: default settings).  With
        ``from_selection`` the selected controllers are registered in it."""
        g = self.controllers.add_group(name)
        warnings = []
        if from_selection and (cmds.ls(selection=True) or []):
            try:
                _added, warnings = self.controllers.add_selected(g.id)
            except SimflowError:
                self.controllers.remove_group(g.id)
                raise
        with undo_chunk("SIMFLOW Add Group"):
            self.save()
        return g, warnings

    def remove_group(self, group_id):
        busy = [e.label for e in self.controllers.group_entries(group_id) if e.bake]
        if busy:
            raise SimflowError("先に「元に戻す」を実行してください。この揺れものにはベイクが残っています: %s"
                               % "、".join(busy))
        ids = [e.id for e in self.controllers.group_entries(group_id)]
        with undo_chunk("SIMFLOW Remove Group"):
            self.preview.clear(ids)
            self._unpreview_rig(ids)
            self.controllers.remove_group(group_id)
            # it no longer pushes any group out
            for g in self.controllers.groups:
                if group_id in g.settings.collide_with:
                    g.settings.collide_with = tuple(x for x in g.settings.collide_with if x != group_id)
            self.sim.invalidate(ids)
            self.save()

    def rename_group(self, group_id, name):
        name = (name or "").strip()
        if not name:
            raise SimflowError("名前を入力してください。")
        g = self.controllers.get_group(group_id)
        if g:
            g.name = name
            with undo_chunk("SIMFLOW Rename Group"):
                self.save()

    def set_group_enabled(self, group_id, enabled):
        g = self.controllers.get_group(group_id)
        if g:
            g.enabled = bool(enabled)

    def set_chain(self, group_id, chain):
        g = self.controllers.get_group(group_id)
        if g:
            g.chain = bool(chain)
            self.sim.invalidate()

    def set_rope(self, group_id, rope):
        """"縄・ひも（両端を持つ）" mode on / off (off = back to the chain /
        per-controller mode it had)."""
        g = self.controllers.get_group(group_id)
        if g:
            g.rope = bool(rope)
            self.sim.invalidate()
            entries = [e for e in self.controllers.group_entries(g.id) if e.enabled]
            if g.rope and len(entries) >= 3:
                try:
                    self._order_rope(g, entries)        # the list shows the grips at the ends right away
                except SimflowError:
                    pass                                # reported when calculating

    def measure_rope_length(self, node=None):
        """(length in the scene's linear unit, how) of the selected spline IK
        curve / root joint / ikHandle (縄の長さ)."""
        from .rope_length import measure
        if node is None:
            sel = cmds.ls(selection=True, long=True) or []
            if not sel:
                raise SimflowError("スプライン IK のカーブ、ルートのジョイント、または IK ハンドルを選んでください。")
            node = sel[0]
        return measure(node)

    def set_chain_auto(self, group_id, auto):
        g = self.controllers.get_group(group_id)
        if g:
            g.chain_auto = bool(auto)
            self.sim.invalidate()

    def reset_values(self, group_id):
        """All settings of the group back to the defaults (controllers and
        chain / per-controller mode are kept)."""
        g = self.controllers.get_group(group_id)
        if g:
            g.settings = SimSettings()
            self.sim.invalidate([e.id for e in self.controllers.group_entries(g.id)])

    def group_settings(self, group_id):
        g = self.controllers.get_group(group_id)
        return g.settings if g else None

    def set_value(self, name, value, group_id=None):
        g = self.controllers.get_group(group_id) if group_id else self._first_group()
        if g:
            setattr(g.settings, name, tuple(value) if isinstance(value, list) else value)

    # ------------------------------------------------------------------
    # controllers
    # ------------------------------------------------------------------
    def add_selected(self, group_id=None):
        gid = group_id or self._first_group().id
        added, warnings = self.controllers.add_selected(gid)
        if added:
            with undo_chunk("SIMFLOW Register"):
                self.save()
        return added, warnings

    def remove(self, ids):
        busy = [e.label for e in self.controllers.entries if e.id in ids and e.bake]
        if busy:
            raise SimflowError("先に「元に戻す」を実行してください。次のコントローラーに SIMFLOW のベイクがあります: %s"
                               % "、".join(busy))
        with undo_chunk("SIMFLOW Remove"):
            for i in ids:
                self.preview.clear([i])
            self._unpreview_rig(ids)
            self.controllers.remove(ids)
            self.sim.invalidate(ids)
            self.save()

    def clear(self):
        self.remove([e.id for e in self.controllers.entries])

    def move(self, ids, offset):
        """Change the order inside a group (= chain order root -> tip)."""
        self.controllers.move(ids, offset)
        self.sim.invalidate()
        with undo_chunk("SIMFLOW Reorder"):
            self.save()

    # ------------------------------------------------------------------
    # presets
    # ------------------------------------------------------------------
    def apply_preset(self, key, group_id=None):
        g = self.controllers.get_group(group_id) if group_id else self._first_group()
        if g is None:
            raise SimflowError("揺れものが選ばれていません。")
        self.presets.apply(key, g.settings)
        self.sim.invalidate([e.id for e in self.controllers.group_entries(g.id)])

    def save_preset(self, name, group_id=None):
        g = self.controllers.get_group(group_id) if group_id else self._first_group()
        if g is None:
            raise SimflowError("揺れものが選ばれていません。")
        return self.presets.save(name, g.settings)

    def capture_rest(self, ids, frame=None):
        frame = cmds.currentTime(query=True) if frame is None else frame
        with undo_chunk("SIMFLOW Capture Rest Pose"):
            for e in self.controllers.entries:
                if e.id in ids:
                    self.sim.capture_rest(e, frame)
            self.save()

    # ------------------------------------------------------------------
    # colliders
    # ------------------------------------------------------------------
    def add_collider(self, kind, attach=None):
        """New sphere / capsule.  ``attach`` defaults to the selected node."""
        if attach is None:
            sel = cmds.ls(selection=True, long=True, type="transform") or []
            attach = sel[0] if sel else None
        with undo_chunk("SIMFLOW Add Collider"):
            col = collider_manager.create_collider(kind, attach)
        self.sim.invalidate()
        return col

    def remove_collider(self, col):
        with undo_chunk("SIMFLOW Remove Collider"):
            collider_manager.delete_collider(col)
        self.sim.invalidate()

    def colliders(self):
        return sorted((collider_manager.info(c) for c in collider_manager.colliders()), key=lambda i: i["name"])

    # ------------------------------------------------------------------
    # simulation
    # ------------------------------------------------------------------
    def _order_rope(self, g, entries):
        """The rope order (grip, controllers along the rope, grip) does not
        depend on how the controllers were selected: it is found from the
        rig and the start frame, and the list is put in that order."""
        from .rope_order import rope_order
        nodes = [self.controllers.resolve(e) for e in entries]
        start, end = int(round(self.bake_options.start)), int(round(self.bake_options.end))
        order, how = rope_order(nodes, [float(f) for f in range(start, max(start + 1, end) + 1)])
        ordered = [entries[i] for i in order]
        if [e.id for e in ordered] != [e.id for e in entries]:
            ids = set(e.id for e in entries)
            rest = [e for e in self.controllers.group_entries(g.id) if e.id not in ids]
            others = [e for e in self.controllers.entries if e.group != g.id]
            self.controllers.entries = others + ordered + rest
            self.controllers._normalise_order()
            LOG.info("縄「%s」: 縄に沿った順に並べ替えました（%s）: 持ち手 %s → %s → 持ち手 %s", g.name, how,
                     ordered[0].label, "→".join(e.label for e in ordered[1:-1]), ordered[-1].label)
        return ordered

    def batches(self, group_ids=None):
        """Enabled groups (or ``group_ids``) with their enabled controllers."""
        out = []
        for g in self.controllers.groups:
            if group_ids is not None:
                if g.id not in group_ids:
                    continue
            elif not g.enabled:
                continue
            entries = [e for e in self.controllers.group_entries(g.id) if e.enabled]
            if not entries:
                continue
            if g.settings.solver != "spring":
                raise SimflowError("nHair モードは Phase 3 で実装予定のため、まだ使えません。")
            if g.rope:
                if len(entries) < 3:
                    raise SimflowError("「%s」は縄・ひもです。持ち手 2 つ（リストのいちばん上と下）と、その間の"
                                       "コントローラーを 1 つ以上登録してください（今は %d 個）。" % (g.name, len(entries)))
                entries = self._order_rope(g, entries)
                # a rope moves its controllers (translate keys); the rotation stays as animated
                rs = g.settings.copy()
                rs.translate, rs.rotate = (True, True, True), (False, False, False)
                out.append(Batch(name=g.name, settings=rs, chain=False, chain_auto=False,
                                 entries=entries[1:-1], gid=g.id, ends=(entries[0], entries[-1])))
                continue
            # a chain of one controller is simply that controller on its own
            chain = g.chain and len(entries) > 1
            out.append(Batch(name=g.name, settings=g.settings, chain=chain, chain_auto=g.chain_auto,
                             entries=entries, gid=g.id))
        if not out:
            raise SimflowError("計算する揺れものがありません（揺れものを作ってコントローラーを登録し、チェックを入れてください）。")
        return out

    def calculate(self, progress=None, preview=True, group_ids=None):
        batches = self.batches(group_ids)
        progress = progress or Progress()
        # the calculation, then the preview proxies and the rig preview (both
        # take a while on long ranges: they get their own part of the bar)
        sim_hi = 1.0 if not preview else (0.8 if self.rig_preview else 0.9)
        results = self.sim.calculate(batches, self.bake_options, _Part(progress, 0.0, sim_hi))
        with undo_chunk("SIMFLOW Save"):
            self.save()
        if preview:
            self.preview.update([e for b in batches for e in b.entries], results,
                                _Part(progress, sim_hi, 0.9 if self.rig_preview else 1.0, "プレビューを作成中"))
            if self.rig_preview:
                progress.update(0.9, "リグに反映中（アニメーションレイヤーにキーを打っています）")
                self._preview_rig(batches, results, _Part(progress, 0.9, 1.0, "リグに反映中"))
        if preview and self.collision_view_gid:
            # keep the shown collision shapes in step with the new result
            self.set_collision_view(self.collision_view_gid)
        progress.update(1.0, "完了")
        return results

    # ------------------------------------------------------------------
    # collision between swing groups: what the last calculation found / used
    # ------------------------------------------------------------------
    collision_view_gid = None

    def push_report(self, group_id):
        """collision_check report of the last calculation (None = not calculated
        with 押し出す揺れもの)."""
        return self.sim.push_reports.get(group_id)

    def set_collision_view(self, group_id):
        """Show the bones / panels ``group_id`` was pushed by (None = hide).
        Returns how many shapes are shown."""
        if not group_id:
            self.collision_view_gid = None
            collision_view.remove()
            return 0
        view = self.sim.group_view.get(group_id)
        if view is None:
            self.collision_view_gid = None
            collision_view.remove()
            raise SimflowError("この揺れものは、まだ「押し出す揺れもの」を使って計算していません。"
                               "押し出す揺れものを選んで、計算してから表示してください。")
        self.collision_view_gid = group_id
        return collision_view.build(view)

    # ------------------------------------------------------------------
    # preview on the real rig
    # ------------------------------------------------------------------
    def want_rig_preview(self):
        """「リグで見る」 checked: the next calculation drives the rig."""
        self.rig_preview = True
        self.rig_preview_waiting = not self.rig_preview_active()

    def set_rig_preview(self, on, progress=None):
        """Show the last results on the real controllers (or take them off)."""
        self.rig_preview = bool(on)
        self.rig_preview_waiting = False
        if not on:
            with undo_chunk("SIMFLOW Rig Preview Off"):
                rig_preview.remove()
            cmds.currentTime(cmds.currentTime(query=True), update=True)
            return None
        # cached results: no recalculation unless something changed
        return self.calculate(progress, preview=True)

    def rig_preview_active(self):
        return rig_preview.layer() is not None

    def _preview_rig(self, batches, results, progress=None):
        nodes = {}
        for b in batches:
            for e in b.entries:
                try:
                    nodes[e.id] = self.controllers.resolve(e)
                except SimflowError:
                    pass
        with undo_chunk("SIMFLOW Rig Preview", rollback=True), suspend_refresh():
            self.rig_preview_skipped = rig_preview.update(batches, results, nodes, self.bake_options, progress)
        self.rig_preview_waiting = False
        cmds.currentTime(cmds.currentTime(query=True), update=True)

    def _unpreview_rig(self, ids):
        paths = []
        for e in self.controllers.entries:
            if e.id in ids:
                try:
                    paths.append(self.controllers.resolve(e))
                except SimflowError:
                    pass
        rig_preview.remove_nodes(paths)

    def bake(self, progress=None, group_ids=None):
        batches = self.batches(group_ids)
        report = self.baker.bake(batches, self.bake_options, progress, save_callback=self.save)
        self.rig_preview = False          # the bake removed the preview layer (it shows the same result)
        self.rig_preview_waiting = False
        return report

    def restore_original(self, ids=None):
        """Restore the given controllers.  Baked controllers BELOW them are
        restored too (their channels were solved against the baked parent)."""
        entries = [e for e in self.controllers.entries if ids is None or e.id in ids]
        paths = {}
        for e in self.controllers.entries:
            try:
                paths[e.id] = self.controllers.resolve(e)
            except SimflowError:
                pass
        extra = []
        for other in self.controllers.entries:
            if other in entries or not other.bake or other.id not in paths:
                continue
            if any(e.id in paths and paths[other.id].startswith(paths[e.id] + "|") for e in entries):
                extra.append(other)
        if extra:
            LOG.info("親と一緒に子孫も元に戻します: %s", ", ".join(e.label for e in extra))
        return self.baker.restore_original(entries + extra, save_callback=self.save)

    def reset_simulation(self):
        """Drop cached results and preview proxies (settings are kept)."""
        self.sim.invalidate()
        self.preview.clear()
        with undo_chunk("SIMFLOW Rig Preview Off"):
            rig_preview.remove()

    def cleanup(self, include_layers=False):
        if include_layers and any(e.bake for e in self.controllers.entries):
            raise SimflowError("ベイクレイヤーやバックアップを削除する前に「元に戻す」を実行してください。"
                               "そうしないと元のアニメーションに戻せなくなります。")
        with undo_chunk("SIMFLOW Clean Up"):
            count = scene_manager.cleanup(include_layers=include_layers,
                                          include_backups=include_layers)
        self.sim.invalidate()
        if scene_manager.get_metadata_node(create=False) is None:
            self.controllers.clear()
        return count
