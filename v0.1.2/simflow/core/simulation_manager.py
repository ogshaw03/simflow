# -*- coding: utf-8 -*-
"""Gather input from Maya, run the solver, convert results back to channels
(spec 5.1).

Pipeline per controller::

    sample frames (SIMFLOW layers muted)          -> FrameSample list
    build SolverInput (world pivot + orientation) -> solver.calculate()
    world delta about pivot -> local matrix -> rotate / translate channels
    apply per-axis channel masks -> recomposed final world matrix

The *final world matrix* (recomposed from the masked channels) is what the
Output Proxy shows and what the bake is validated against, so preview and
bake always agree.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import Dict, List

import maya.cmds as cmds

from ..solvers.base_solver import Progress, SolverInput
from ..solvers.chain_solver import ChainSolver, delay_rotations, find_links, follow_rotations, follow_segments
from ..solvers import collision as col_mod
from ..solvers.spring_solver import SpringSolver
from ..solvers.nhair_solver import NHairSolver
from ..solvers.rope_solver import RopeSolver
from ..solvers import rope_curve
from . import rope_curve_probe
from ..utils import math_utils as mu
from ..utils.logger import get_logger
from ..utils.maya_utils import SimflowError, fps as scene_fps
from . import camera_view
from . import collider_manager
from . import collision_check
from . import scene_manager
from . import transform_utils as tu
from . import warp as warp_mod
from .settings import AIM_AXES

LOG = get_logger("simulation")


@dataclass
class FrameResult:
    frame: float
    base_t: tuple          # original channel values (internal units)
    base_r: tuple
    t: tuple               # final (masked) channel values
    r: tuple
    final_world: list      # world matrix produced by t / r
    orig_world: list
    desired_world: list    # unmasked solver result
    pivot: tuple           # final world pivot
    quat: tuple            # final world orientation
    orig_pivot: tuple
    orig_quat: tuple
    scene_world: list = None   # world as evaluated in the scene (before chain / parent carry)


@dataclass
class Batch:
    """One swing group to calculate."""
    name: str
    settings: object          # SimSettings
    chain: bool
    chain_auto: bool
    entries: list
    gid: str = ""             # SwingGroup.id (other groups refer to it: collide_with)
    # rope ("縄・ひも"): the two grips (entries, not simulated / baked);
    # ``entries`` are then the controllers between them, in rope order
    ends: tuple = ()
    # "チェーンごとの値": chain root entry id -> {field: value} over ``settings``
    chain_overrides: dict = None


class _Unit(object):
    """Solved together: a whole chain, or one controller."""

    def __init__(self, batch, entries, chain=False, chains=None, rope=False):
        self.batch = batch
        self.entries = entries
        self.chain = chain and len(entries) > 1
        self.chains = chains            # several chains solved together ("横のつながり")
        self.rope = rope                # the controllers of a rope, between its two grips (batch.ends)


@dataclass
class SimResult:
    entry_id: str
    node: str
    frames: List[float]
    frame_results: List[FrameResult]
    input_hash: str
    settings: dict
    tip_local: tuple
    pendulum_length: float
    info: dict = field(default_factory=dict)

    def by_frame(self):
        return {fr.frame: fr for fr in self.frame_results}


def _serialisable_ch(ch):
    return {k: (list(v) if isinstance(v, tuple) else v) for k, v in ch.items()}


def _ch_from_json(ch):
    return {k: (tuple(v) if isinstance(v, list) else v) for k, v in ch.items()}


def make_solver(settings):
    if settings.solver == "spring":
        return SpringSolver()
    if settings.solver == "nhair":
        return NHairSolver()
    raise SimflowError("不明なソルバーです: %r" % settings.solver)


class SimulationManager(object):

    def __init__(self, controller_manager):
        self.controllers = controller_manager
        self.cache: Dict[str, SimResult] = {}
        self._colliders = None          # per frame collider list of the current calculation
        self._collider_sig = ""
        self._cameras = {}              # camera name -> per frame (inverse world, tan_h) (warp check)
        self._swing_of, self._swings = {}, {}   # solved chains' swings (横のつながり on hanging chains)
        self._ride = {}                          # samples carried by baked / previewed parents of other groups
        # "押し出す揺れもの": per pushed group id, the bones / panels it was pushed
        # by in the last calculation ({"frames", "cols"}) and how far its bones
        # went into them (collision_check report)
        self.group_view = {}
        self.push_reports = {}

    # ------------------------------------------------------------------
    @staticmethod
    def sim_frames(start, end):
        start, end = int(round(start)), int(round(end))
        if end <= start:
            raise SimflowError("終了フレームは開始フレームより後にしてください")
        return [float(f) for f in range(start, end + 1)]

    def sample(self, node, frames, progress=None, label=""):
        progress = progress or Progress()
        sampler = tu.ControllerSampler(node)
        samples = []
        with scene_manager.tool_layers_muted():
            for i, f in enumerate(frames):
                if i % 10 == 0:
                    progress.check(0.5 * i / len(frames), "Sampling %s" % label)
                s = sampler.sample(f)
                tu.verify_sample(s)
                samples.append(s)
        return samples

    # ------------------------------------------------------------------
    def capture_rest(self, entry, frame):
        node = self.controllers.resolve(entry)
        samples = self.sample(node, [float(frame)])
        entry.rest = {"frame": float(frame), "ch": _serialisable_ch(samples[0].ch)}
        LOG.info("Rest Pose captured for %s at frame %s", entry.label, frame)
        return entry.rest

    # ------------------------------------------------------------------
    @staticmethod
    def input_hash(samples, settings, entry, extra=()):
        h = hashlib.sha1()
        for s in samples:
            h.update(("%.6g" % s.frame).encode())
            h.update(",".join("%.6g" % v for v in s.world).encode())
            h.update(",".join("%.6g" % v for v in s.local).encode())
            h.update(",".join("%.6g" % v for v in s.solve_parent).encode())
        h.update(json.dumps(settings.to_dict(), sort_keys=True).encode())
        h.update(json.dumps(entry.rest, sort_keys=True).encode())
        for x in extra:
            h.update(repr(x).encode())
        return h.hexdigest()

    # ------------------------------------------------------------------
    @staticmethod
    def _is_ancestor(nodes, a, b):
        return nodes[b.id].startswith(nodes[a.id] + "|")

    @staticmethod
    def check_chain_order(entries, nodes):
        """A chain keeps the list order; reject an order that contradicts the
        hierarchy (a parent listed after its child)."""
        for i, a in enumerate(entries):
            for b in entries[i + 1:]:
                if nodes[a.id].startswith(nodes[b.id] + "|"):
                    raise SimflowError(
                        "チェーンの順序が正しくありません: %s は %s の親なので、先（根元側）に並べてください。"
                        % (b.label, a.label))

    @staticmethod
    def variation_key(nodes, entry):
        """Name the "ばらつき" pattern is derived from (stable when the list is
        reordered, the scene is reopened or the rig is referenced with another
        namespace)."""
        return nodes[entry.id].split("|")[-1].split(":")[-1]

    @classmethod
    def split_chains(cls, entries, nodes):
        """Chains of one swing group, root -> tip.

        * No registered controller is below another (e.g. world-space
          controllers): ONE chain in list order (the list defines it).
        * Otherwise the hierarchy defines the chains: every controller hangs
          from its nearest registered ancestor, so several bone chains in one
          group (a sleeve / skirt with 3 rows of bones) become separate chains.
          At a branch the child with the most registered controllers below it
          continues the chain when it has at least twice as many as the first
          child (the thick bundle of a ponytail whose thin bundles hang from
          it), otherwise the first child (list order); the other children start
          their own chains (moved by the solved branch point like any other
          parent).  A controller with no registered parent or child is solved
          on its own."""
        parent = {}
        for e in entries:
            best = None
            for a in entries:
                if a is not e and cls._is_ancestor(nodes, a, e):
                    if best is None or len(nodes[a.id]) > len(nodes[best.id]):
                        best = a
            parent[e.id] = best
        if all(p is None for p in parent.values()):
            cls.check_chain_order(entries, nodes)
            return [list(entries)]
        children = {e.id: [] for e in entries}
        for e in entries:
            if parent[e.id] is not None:
                children[parent[e.id].id].append(e)
        size = {}

        def subtree(e):
            if e.id not in size:
                size[e.id] = 1 + sum(subtree(k) for k in children[e.id])
            return size[e.id]
        chains = []
        starts = [e for e in entries if parent[e.id] is None]
        while starts:
            cur = starts.pop(0)
            chain = [cur]
            while children[cur.id]:
                kids = children[cur.id]
                main = max(kids, key=lambda k: (subtree(k), -kids.index(k)))
                if subtree(main) < 2 * subtree(kids[0]):
                    main = kids[0]          # about the same size (sleeve rows): the list order
                starts.extend(k for k in kids if k is not main)
                cur = main
                chain.append(cur)
            chains.append(chain)
        return chains

    @classmethod
    def chain_tiers(cls, chains, entries, nodes):
        """Chains grouped by how many other chains of the group they hang
        from (0 = from outside the group, 1 = from a branch point of a tier-0
        chain, ...).  Chains of one tier never hang from each other, so they
        can be solved together ("横のつながり") once the tier above is solved."""
        owner = {}
        for k, c in enumerate(chains):
            for e in c:
                owner[e.id] = k
        tier = {}

        def level(k):
            if k not in tier:
                root = chains[k][0]
                above = [a for a in entries if a.id in owner and owner[a.id] != k and cls._is_ancestor(nodes, a, root)]
                if not above:
                    tier[k] = 0
                else:
                    near = max(above, key=lambda a: len(nodes[a.id]))
                    tier[k] = level(owner[near.id]) + 1
            return tier[k]
        out = {}
        for k in range(len(chains)):
            out.setdefault(level(k), []).append(chains[k])
        return [out[t] for t in sorted(out)]

    def _order_units(self, units, nodes):
        """Units (a chain, or one controller) in an order where every
        ancestor is solved before its descendants (their bake changes the
        descendants' parent space)."""
        def depth(u):
            return min(nodes[e.id].count("|") for e in u.entries)

        pending = sorted(units, key=depth)
        ordered = []
        while pending:
            for u in pending:
                pushers = set(u.batch.settings.collide_with) if u.batch.settings.collision else set()
                blocked = any((o.batch is not u.batch and o.batch.gid and o.batch.gid in pushers) or
                              any(self._is_ancestor(nodes, a, b) for a in o.entries for b in u.entries)
                              for o in pending if o is not u)
                if not blocked:
                    ordered.append(u)
                    pending.remove(u)
                    break
            else:
                names = "、".join("「%s」" % u.batch.name for u in pending)
                raise SimflowError("揺れもの %s の階層が入り組んでいるため、計算の順番を決められません。"
                                   "あるチェーンの途中に、別の揺れもののコントローラーが挟まっていないか確認してください。"
                                   "（「押し出す揺れもの」で選んだ揺れものは先に計算するため、その揺れものの階層の下に"
                                   "押し出される側の揺れものがあると、順番を決められません）"
                                   % names)
        return ordered

    @staticmethod
    def _carry(samples, dep, structural):
        """Move ``samples`` with the simulated motion of an already solved
        controller ``dep`` (SimResult)."""
        by = dep.by_frame()
        out = []
        for smp in samples:
            fr = by[smp.frame]
            base = fr.scene_world if structural else fr.orig_world
            delta = mu.m_mul(mu.m_inverse(base), fr.final_world)
            out.append(smp.carried(delta, structural))
        return out

    def _nearest(self, nodes, done, e):
        ancestors = [d for d in done if self._is_ancestor(nodes, d, e)]
        return max(ancestors, key=lambda d: len(nodes[d.id])) if ancestors else None

    def calculate(self, batches, options, progress=None):
        """Calculate several swing groups.

        ``batches``: list of :class:`Batch` (settings, chain flags, entries).
        ``options`` is a BakeOptions (start / end / preroll / preroll_mode).
        Returns {entry_id: SimResult}."""
        progress = progress or Progress()
        batches = [b for b in batches if b.entries]
        if not batches:
            raise SimflowError("有効なコントローラーが登録されていません。")
        eff = {}
        nodes = {}
        owner = {}
        for b in batches:
            problems = b.settings.validate()
            if problems:
                raise SimflowError("「%s」の設定に問題があります:\n" % b.name + "\n".join(problems))
            for e in b.entries:
                eff[e.id] = b.settings
                nodes[e.id] = self.controllers.resolve(e)
                if e.bake and e.bake.get("mode") == "overwrite":
                    raise SimflowError(
                        "%s は元のアニメーションに上書きでベイクされています。再計算の前に「元に戻す」を実行してください"
                        "（カーブにもう元のアニメーションが残っていないため）。" % e.label)
                if nodes[e.id] in owner:
                    raise SimflowError("%s が「%s」と「%s」の両方に登録されています。" % (e.label, owner[nodes[e.id]], b.name))
                owner[nodes[e.id]] = b.name
            for e in b.ends:
                nodes[e.id] = self.controllers.resolve(e)
        self._check_push_cycles(batches)
        units = []
        for b in batches:
            if b.ends:
                units.append(_Unit(b, list(b.entries), rope=True))
            elif b.chain:
                chains = self.split_chains(b.entries, nodes)
                # "チェーンごとの値": a chain's own values over the group's
                for c in chains:
                    own = (b.chain_overrides or {}).get(c[0].id)
                    if own:
                        cs = b.settings.with_overrides(own)
                        problems = cs.validate()
                        if problems:
                            raise SimflowError("「%s」のチェーン %s〜 の値に問題があります:\n" % (b.name, c[0].label)
                                               + "\n".join(problems))
                        for e in c:
                            eff[e.id] = cs
                if max(eff[c[0].id].chain_link for c in chains) > 0.0 and len(chains) > 1:
                    # "横のつながり": the chains of one tier are solved together and
                    # linked (chains that branch off another chain of the group
                    # - thin bundles hanging from a thick one - ride on its
                    # result, so they are a tier of their own)
                    for tier_chains in self.chain_tiers(chains, b.entries, nodes):
                        if len(tier_chains) > 1:
                            units.append(_Unit(b, [e for c in tier_chains for e in c], chain=True, chains=tier_chains))
                        else:
                            units.append(_Unit(b, tier_chains[0], chain=True))
                else:
                    units.extend(_Unit(b, chain, chain=True) for chain in chains)
            else:
                units.extend(_Unit(b, [e]) for e in b.entries)
        units = self._order_units(units, nodes)
        order = [e for u in units for e in u.entries]
        # rope grips are sampled too (they follow the hands)
        for u in units:
            if u.rope:
                order.extend(e for e in u.batch.ends if all(e.id != o.id for o in order))

        frames = self.sim_frames(options.start, options.end)
        preroll = int(options.preroll or 0)
        if preroll < 0:
            raise SimflowError("プリロールは 0 以上にしてください")
        pre_frames = [frames[0] - k for k in range(preroll, 0, -1)]
        hold = options.preroll_mode == "hold"
        sample_frames = frames if hold else pre_frames + frames

        samplers = [tu.ControllerSampler(nodes[e.id]) for e in order]
        col_samplers = []
        if any(b.settings.collision for b in batches):
            col_samplers = [collider_manager.ColliderSampler(c) for c in collider_manager.colliders()]
        cam_names = sorted(set(b.settings.warp_camera for b in batches
                               if b.settings.warp_enabled and b.settings.warp_camera))
        cam_samplers = [camera_view.CameraSampler(c) for c in cam_names]
        self._rope_curves = {}
        curve_samplers, curve_keys = [], []
        with scene_manager.tool_layers_muted():
            if col_samplers:
                # 縄: the curve that draws the rope collides, not the controller points
                for u in units:
                    if u.rope and u.batch.settings.collision:
                        found = self._find_rope_curve(u.batch, nodes)
                        if found is not None:
                            curve_samplers.append(rope_curve_probe.CurveSampler(found[0]))
                            curve_keys.append(id(u.batch))
                            self._rope_curves[id(u.batch)] = found
            sampled = tu.sample_many(samplers + col_samplers + cam_samplers + curve_samplers, sample_frames,
                                     progress, 0.0, 0.3)
        if curve_samplers:
            for key, rows in zip(curve_keys, sampled[-len(curve_samplers):]):
                if hold and pre_frames:
                    rows = [rows[0] for _ in pre_frames] + list(rows)
                self._rope_curves[key] = self._rope_curves[key] + (rows,)
            sampled = sampled[:-len(curve_samplers)]
        cam_sampled = sampled[len(samplers) + len(col_samplers):]
        col_sampled = sampled[len(samplers):len(samplers) + len(col_samplers)]
        sampled = sampled[:len(samplers)]
        self._cameras = {}
        for name, rows in zip(cam_names, cam_sampled):
            if hold and pre_frames:
                rows = [rows[0] for _ in pre_frames] + list(rows)
            self._cameras[name] = (rows, camera_view.signature(rows))
        self._colliders = None
        if col_samplers:
            per_frame = [list(row) for row in zip(*col_sampled)]
            if hold and pre_frames:
                per_frame = [list(per_frame[0]) for _ in pre_frames] + per_frame
            self._colliders = per_frame
        self._collider_sig = collider_manager.signature(self._colliders) if self._colliders else ""
        scene_colliders = (self._colliders, self._collider_sig)
        self._frames = pre_frames + frames
        scene = {}
        for e, smps in zip(order, sampled):
            for smp in smps:
                tu.verify_sample(smp)
            if hold and pre_frames:
                # プリロール区間は入力アニメーションの代わりに開始フレームの姿勢で静止させる
                smps = [smps[0].copy_at(f) for f in pre_frames] + smps
            scene[e.id] = smps

        self._ride_on_simflow_ancestors(order, nodes, scene, sample_frames, hold, pre_frames)

        results = {}
        done = []
        # 横のつながり: the swing of every solved chain, for the chains that
        # hang from it (entry id -> (chain key, index); chain key -> swings)
        self._swing_of, self._swings = {}, {}
        total = float(len(order))
        k = 0
        group_cols = {}
        for n_unit, u in enumerate(units):
            if u.rope:
                what = "縄（%d 個）" % len(u.entries)
            elif u.chains:
                what = "チェーン %d 本" % len(u.chains)
            elif u.chain:
                what = "チェーン %s〜" % u.entries[0].label
            else:
                what = u.entries[0].label
            sub = _SubRange(progress, 0.3 + 0.7 * k / total, 0.3 + 0.7 * (k + len(u.entries)) / total,
                            "「%s」%s（%d / %d）" % (u.batch.name, what, n_unit + 1, len(units)))
            sub.update(0.0, "準備中")
            k += len(u.entries)
            # colliders of this group: the scene's + the groups that push it out
            key = id(u.batch)
            if key not in group_cols:
                group_cols[key] = self._group_colliders(u.batch, batches, results, scene_colliders)
            self._colliders, self._collider_sig = group_cols[key]
            if u.rope:
                results.update(self._calculate_rope(u.batch, eff, nodes, scene, sub, results, done))
                done.extend(u.entries)
                continue
            if u.chain:
                results.update(self._calculate_chain(u.chains or u.entries, eff, nodes, scene, u.batch.chain_auto,
                                                     sub, results, done))
                done.extend(u.entries)
                continue
            e = u.entries[0]
            samples = scene[e.id]
            deps = []
            anc = self._nearest(nodes, done, e)
            if anc is not None:
                samples = self._carry(samples, results[anc.id], structural=True)
            elif e.id in self._ride:
                samples = self._ride[e.id]           # on a baked / previewed controller of another group
                deps.append(results[anc.id].input_hash)
            es = eff[e.id].varied(self.variation_key(nodes, e))
            if es.collision and self._colliders:
                deps.append(("colliders", self._collider_sig))
            if es.warp_enabled and es.warp_camera in self._cameras:
                deps.append(("camera", self._cameras[es.warp_camera][1]))
            results[e.id] = self.calculate_entry(e, es, samples, nodes[e.id], deps, None, sub)
            done.append(e)
        self._colliders, self._collider_sig = scene_colliders
        self.cache.update(results)
        self._check_pushed(batches, results)
        return results

    def _ride_on_simflow_ancestors(self, order, nodes, scene, sample_frames, hold, pre_frames):
        """Controllers of another swing group (not calculated now) above the
        calculated ones that SIMFLOW moves in the scene: baked into a SIMFLOW
        layer, or shown by 「リグで見る」.  The sampling muted those layers (it
        saw the original animation), but they stay in the scene, so the
        controllers below ride on them (the thin bundles baked after the thick
        one).  Their samples are carried by how that ancestor moves compared
        with its original animation.  (The bake refuses a parent that is only
        previewed, see BakeManager._check_previewed_ancestors.)"""
        from . import rig_preview
        self._ride = {}
        calc = set(nodes[e.id] for e in order)
        previewed = set()
        lay = rig_preview.layer()
        if lay:
            for plug in cmds.animLayer(lay, query=True, attribute=True) or []:
                previewed.update(cmds.ls(plug.rsplit(".", 1)[0], long=True) or [])
        baked = []
        for other in self.controllers.entries:
            try:
                path = self.controllers.resolve(other)
            except SimflowError:
                continue
            moved_by_simflow = (other.bake and other.bake.get("mode") in ("additive", "override")) or path in previewed
            if not moved_by_simflow or path in calc:
                continue
            if any(n.startswith(path + "|") for n in calc):
                baked.append(path)
        if not baked:
            return
        samplers = [tu.ControllerSampler(p) for p in baked]
        with scene_manager.tool_layers_muted():
            orig = tu.sample_many(samplers, sample_frames)
        now = tu.sample_many(samplers, sample_frames)
        delta = {}
        for p, a, b in zip(baked, orig, now):
            rows = [mu.m_mul(mu.m_inverse(x.world), y.world) for x, y in zip(a, b)]
            if hold and pre_frames:
                rows = [rows[0] for _ in pre_frames] + rows
            delta[p] = rows
        moved = []
        for e in order:
            # (a controller below a calculated one is carried by that result
            # when there is one; these samples are its input otherwise)
            above = [p for p in baked if nodes[e.id].startswith(p + "|")]
            if not above:
                continue
            p = max(above, key=len)
            self._ride[e.id] = [smp.carried(d, True) for smp, d in zip(scene[e.id], delta[p])]
            moved.append(e.label)
        if moved:
            LOG.info("ほかの揺れものの親（ベイク済み・リグで見る）の動きに乗せて計算します: %s", "、".join(moved))

    def _check_pushed(self, batches, results):
        """Measure how far the bones of every pushed group went into the bones
        / panels of the groups that push it (final result, per frame)."""
        for b in batches:
            view = self.group_view.get(b.gid) if b.gid else None
            if view is None or not b.settings.collision or not b.settings.collide_with:
                self.push_reports.pop(b.gid, None)
                continue
            entries = [(e.label, results[e.id]) for e in b.entries if e.id in results]
            report = collision_check.check(entries, view["cols"], b.settings.collision_thickness)
            report["strength"] = b.settings.strength
            report["rotate"] = tuple(b.settings.rotate)
            self.push_reports[b.gid] = report
            text = collision_check.describe(report, strength=b.settings.strength, rotate=tuple(b.settings.rotate))
            if report["hits"]:
                LOG.warning("「%s」: %s", b.name, text)
                for f, label, depth, through, kind, root in report["hits"][:60]:
                    LOG.info("   フレーム %g  %s  %.3f cm（%s%s%s）", f, label, depth, kind, "を貫通" if through else "",
                             "、根元が当たりの中" if root else "")
            else:
                LOG.info("「%s」: %s", b.name, text)

    # ------------------------------------------------------------------
    # one-way collision between swing groups ("押し出す揺れもの")
    # ------------------------------------------------------------------
    def _check_push_cycles(self, batches):
        """「押し出す揺れもの」 must not go round in a circle (one way)."""
        settings, names = {}, {}
        for g in getattr(self.controllers, "groups", []):
            settings[g.id], names[g.id] = g.settings, g.name
        for b in batches:
            if b.gid:
                settings[b.gid], names[b.gid] = b.settings, b.name

        def pushers(gid):
            st = settings.get(gid)
            if st is None or not st.collision:
                return []
            return [x for x in st.collide_with if x != gid and x in settings]

        state, stack = {}, []

        def visit(gid):
            state[gid] = 1
            stack.append(gid)
            for p in pushers(gid):
                if state.get(p) == 1:
                    loop = stack[stack.index(p):]
                    raise SimflowError(
                        "揺れもの %s が、お互いを押し出す設定になっています。揺れもの同士の当たりは片方向なので、"
                        "どれか 1 つの「押し出す揺れもの」から外してください。"
                        % "と".join("「%s」" % names[x] for x in loop))
                if p not in state:
                    visit(p)
            stack.pop()
            state[gid] = 2

        for b in batches:
            if b.gid and b.gid not in state:
                visit(b.gid)

    def _group_colliders(self, batch, batches, results, scene_colliders):
        """Per frame colliders of ``batch``: the scene's colliders and, for
        every group in its "押し出す揺れもの", every segment of that group's
        result as a capsule (pivot -> tip, radius = that group's 太さ)."""
        scene_cols, scene_sig = scene_colliders
        st = batch.settings
        gids = [x for x in st.collide_with if x and x != batch.gid] if st.collision else []
        if not gids:
            return scene_colliders
        n = len(self._frames)
        extra = [[] for _ in range(n)]
        sigs = []
        for gid in gids:
            pusher = next((b for b in batches if b.gid == gid), None)
            group = self.controllers.get_group(gid) if hasattr(self.controllers, "get_group") else None
            name = pusher.name if pusher is not None else (group.name if group is not None else gid)
            if pusher is not None:
                entries = pusher.entries
                radius = pusher.settings.collision_thickness
                pool = results
            elif group is not None:
                entries = [e for e in self.controllers.group_entries(gid) if e.enabled]
                radius = group.settings.collision_thickness
                pool = self.cache
            else:
                LOG.warning("「%s」: 押し出す揺れものの 1 つが見つかりません（削除されています）。無視します。", batch.name)
                continue
            got = [pool.get(e.id) for e in entries]
            if not got or any(r is None for r in got):
                LOG.warning("「%s」: 押し出す揺れもの「%s」の計算結果がないため、当たりに使いません"
                            "（「%s」にもチェックを入れて、一緒に計算してください）。", batch.name, name, name)
                continue
            if any([float(f) for f in r.frames] != [float(f) for f in self._frames] for r in got):
                LOG.warning("「%s」: 押し出す揺れもの「%s」の結果のフレームが今回の計算と違うため、当たりに使いません"
                            "（「%s」にもチェックを入れて、一緒に計算してください）。", batch.name, name, name)
                continue
            for r in got:
                sigs.append(r.input_hash)
                for k, fr in enumerate(r.frame_results):
                    if r.pendulum_length <= 1.0e-6:
                        extra[k].append((fr.pivot, (1.0, 0.0, 0.0), 0.0, radius, True))
                        continue
                    d = mu.q_rotate(fr.quat, mu.v_normalize(r.tip_local, (1.0, 0.0, 0.0)))
                    half = 0.5 * r.pendulum_length
                    extra[k].append((mu.v_add(fr.pivot, mu.v_scale(d, half)), d, half, radius, True))
            chain_mode = pusher.chain if pusher is not None else bool(group and group.chain)
            panels = self._pusher_panels(entries, dict(zip([e.id for e in entries], got)), radius, n) \
                if chain_mode and len(entries) > 1 else None
            n_panels = 0
            if panels:
                n_panels = len(panels[0])
                for k in range(n):
                    extra[k].extend(panels[k])
            sigs.append("%s %r %d" % (gid, radius, n_panels))
            LOG.info("「%s」: 「%s」の骨 %d 本%sを当たりにします（太さ %s）", batch.name, name, len(got),
                     "と、隣のチェーンの間の面 %d 枚" % n_panels if n_panels else "", radius)
        if not sigs:
            return scene_colliders
        self.group_view[batch.gid] = {"frames": list(self._frames), "cols": extra}
        cols = [list(scene_cols[k]) if scene_cols else [] for k in range(n)]
        for k in range(n):
            cols[k].extend(extra[k])
        sig = hashlib.sha1(("%s|%s" % (scene_sig, "|".join(sigs))).encode()).hexdigest()
        return cols, sig

    def _pusher_panels(self, entries, by_id, radius, n):
        """Cloth panels between neighbouring chains of a pushing group: for
        neighbouring chains A / B (same rule as 横のつながり, roots at most
        2 x the first segment length apart) the quad pivot / tip of joint j of
        A and B, as two triangles.  Per frame list, or None."""
        try:
            nodes = {e.id: self.controllers.resolve(e) for e in entries}
            chains = self.split_chains(entries, nodes)
        except Exception:          # resolved again (and reported) when calculated
            return None
        if len(chains) < 2:
            return None

        def ends(r, k):
            fr = r.frame_results[k]
            d = mu.q_rotate(fr.quat, mu.v_normalize(r.tip_local, (1.0, 0.0, 0.0)))
            return fr.pivot, mu.v_add(fr.pivot, mu.v_scale(d, r.pendulum_length))

        roots = [by_id[c[0].id].frame_results[0].pivot for c in chains]
        pairs = []
        for a, b in find_links(roots):
            reach = 2.0 * max(by_id[chains[a][0].id].pendulum_length, by_id[chains[b][0].id].pendulum_length)
            if mu.v_length(mu.v_sub(roots[a], roots[b])) <= reach:
                pairs.append((a, b))
        if not pairs:
            return None
        out = []
        for k in range(n):
            row = []
            for a, b in pairs:
                for ea, eb in zip(chains[a], chains[b]):
                    ra, rb = by_id[ea.id], by_id[eb.id]
                    if ra.pendulum_length <= 1.0e-6 or rb.pendulum_length <= 1.0e-6:
                        continue
                    a0, a1 = ends(ra, k)
                    b0, b1 = ends(rb, k)
                    row.append(col_mod.panel(a0, a1, b1, radius))
                    row.append(col_mod.panel(a0, b1, b0, radius))
            out.append(row)
        return out

    def _calculate_chain(self, chains, eff, nodes, scene, chain_auto, progress, solved, done):
        """Chains of one swing group: all controllers of a chain are solved
        together by the coupled :class:`ChainSolver` (several chains in the
        same sub steps; ``link`` > 0 keeps neighbouring chains together); the
        result of each controller is then converted to channels against its
        (already solved) parent.  ``solved`` / ``done`` are the results of
        other groups solved before (ancestors outside the chain carry the
        chain's input).  ``chains``: one chain (list of entries) or a list of
        chains.  横のつながり / タイミングのずれ come from each chain's root's
        settings in ``eff`` (a chain may have its own values)."""
        if chains and not isinstance(chains[0], (list, tuple)):
            chains = [chains]
        links = [eff[order[0].id].chain_link for order in chains]
        link = max(links)
        inside = set(e.id for order in chains for e in order)
        base = {}
        ext_deps = []
        for order in chains:
            for e in order:
                anc = self._nearest(nodes, done, e)
                if anc is not None and anc.id not in inside:
                    base[e.id] = self._carry(scene[e.id], solved[anc.id], structural=True)
                    ext_deps.append(solved[anc.id].input_hash)
                elif anc is None and e.id in self._ride:   # (inside the chain: its input; the result is carried by the parent)
                    base[e.id] = self._ride[e.id]      # on a baked / previewed controller of another group
                else:
                    base[e.id] = scene[e.id]
        all_settings, all_inputs, entry_hashes = [], [], []
        for order in chains:
            settings_list, inputs = [], []
            key = self.variation_key(nodes, order[0])
            for idx, e in enumerate(order):
                es = eff[e.id].varied(key)
                if any(es.translate):
                    raise SimflowError("チェーンでは移動のシミュレーションは使えません（%s の「移動」の軸をすべてオフにしてください）。"
                                       "移動も揺らしたい場合は、揺れ方を「1 つずつ」にしてください。" % e.label)
                tip = None
                es = es.copy()
                # root -> tip stiffness profile (each joint gets its own value)
                es.stiffness = es.chain_stiffness(idx, len(order))
                if chain_auto:
                    tip, length = self._auto_segment(order, idx, base)
                    es.pendulum_length = length
                elif es.pendulum_length <= 1.0e-6:
                    raise SimflowError("チェーンでは振り子の長さを 0 より大きくしてください（%s）。" % e.label)
                settings_list.append(es)
                inputs.append(self.build_input(e, base[e.id], es, tip))
                if es.collision and self._colliders:
                    inputs[-1].colliders = self._colliders
                if idx == 0 and base[e.id] is not scene[e.id]:
                    inputs[-1].hangs = True          # carried by a simulated parent
                if idx == 0 and es.side_damp > 0.0 and base[e.id] is not scene[e.id]:
                    inputs[-1].scene_root = ([tu.pivot_world(smp.ch, smp.parent) for smp in scene[e.id]],
                                             [tu.world_rotation(smp.world) for smp in scene[e.id]])
                entry_hashes.append(self.input_hash(base[e.id], es, e, ("chain", idx, tip,
                                                                         self._collider_sig if es.collision else "")))
            all_settings.append(settings_list)
            all_inputs.append(inputs)
        if len(chains) > 1:
            entry_hashes.append("link %r %d" % (links, len(chains)))
        follow = self._follow_plan(chains, nodes, done, base, scene, links) if link > 0.0 else {}
        if follow:
            entry_hashes.append("follow %r %r" % (links, sorted(follow)))
        first = all_settings[0][0]
        if first.warp_enabled and first.warp_camera in self._cameras:
            ext_deps.append(self._cameras[first.warp_camera][1])
        chain_hash = hashlib.sha1("|".join(entry_hashes + ext_deps).encode()).hexdigest()

        every = [e for order in chains for e in order]
        cached = [self.cache.get(e.id) for e in every]
        if all(c is not None and c.input_hash == chain_hash and c.node == nodes[e.id]
               for c, e in zip(cached, every)):
            LOG.info("チェーン: 入力が変わっていないため、前回の結果を使います")
            results = {e.id: c for e, c in zip(every, cached)}
            self._record_swings(chains, scene, results)
            return results

        # warps: found on the (first) chain root, the same compensation for
        # the whole chain (and for every chain solved with it)
        plan = self._warp_plan(first, all_inputs[0][0], all_inputs[0][-1], all_settings[0][-1].pendulum_length)
        if plan and plan.active:
            all_inputs_sim = [[warp_mod.sim_input(i, plan.comp) for i in inputs] for inputs in all_inputs]
        else:
            all_inputs_sim = all_inputs
        solver = ChainSolver()
        solver.initialize()
        solver.assign_chains(all_inputs_sim)
        solver.configure_chains(all_settings, links if len(chains) > 1 else 0.0)
        all_raws = solver.calculate(_SubRange(progress, 0.0, 0.9))
        progress.update(0.92, "結果をチャンネルに変換中")
        solver.cleanup()
        if plan is not None:
            for k, raws in enumerate(all_raws):
                if plan.active:
                    raws = all_raws[k] = [warp_mod.world_result(r, plan.comp) for r in raws]
                for r in raws:
                    r.info["warps"] = plan.warps
            LOG.info("チェーン %s: %s", chains[0][0].label, warp_mod.describe(plan.warps))
        for k, swings in follow.items():
            follow_rotations(all_raws[k], all_inputs[k], swings, min(1.0, links[k] / 100.0))
        # タイミングのずれ: after the follow (the chains hanging from this one
        # ride on its shifted result)
        for k, settings_list in enumerate(all_settings):
            if settings_list[0].delay:
                delay_rotations(all_raws[k], all_inputs[k], settings_list[0].delay)
        progress.check(0.95, "変換中")

        results = {}
        for order, settings_list, inputs, raws in zip(chains, all_settings, all_inputs, all_raws):
            inner_done = []
            for idx, e in enumerate(order):
                anc = self._nearest(nodes, inner_done, e)
                if anc is not None:
                    # the parent inside the chain already includes outside motion
                    samples = self._carry(scene[e.id], results[anc.id], structural=True)
                else:
                    samples = base[e.id]
                if idx > 0 and not self._is_ancestor(nodes, order[idx - 1], e):
                    samples = self._carry(samples, results[order[idx - 1].id], structural=False)
                es = settings_list[idx]
                frame_results = self.to_channels(samples, raws[idx], es)
                results[e.id] = SimResult(entry_id=e.id, node=nodes[e.id], frames=[smp.frame for smp in samples],
                                          frame_results=frame_results, input_hash=chain_hash,
                                          settings=es.to_dict(), tip_local=inputs[idx].tip_local,
                                          pendulum_length=es.pendulum_length, info=raws[idx].info)
                inner_done.append(e)
        if all_raws[0][0].info.get("settled") is False:
            LOG.warning("チェーン: " + "初期状態が落ち着ききりませんでした（最初のポーズから重力で垂れるまで 30 秒ぶん揺らしても止まりませんでした。当たりに触れている、ほぼ真上を向いているなどが考えられます）。最初の数フレームに揺れが残ることがあります")
        if len(chains) > 1:
            LOG.info("チェーン %d 本を一緒に計算しました（横のつながり %s、隣のチェーンとのつながり %d か所）",
                     len(chains), link, sum(r[0].info.get("links", 0) for r in all_raws) // 2)
        if follow:
            LOG.info("横のつながり %s: ぶら下がるチェーン %d 本を、ぶら下がり先のチェーンの揺れにそろえました", link, len(follow))
        self._record_swings(chains, scene, results)
        return results

    def _record_swings(self, chains, scene, results):
        """Keep, for every joint of these solved chains, its result and
        animated world rotations per frame and its length along the hair (for
        chains that hang from them, see :meth:`_follow_plan`)."""
        for order in chains:
            key = tuple(e.id for e in order)
            turns, arcs, prev, s = [], [], None, 0.0
            for e in order:
                smps = scene[e.id]
                turns.append(([fr.quat for fr in results[e.id].frame_results],
                          [tu.world_rotation(smp.world) for smp in smps]))
                p = tu.pivot_world(smps[0].ch, smps[0].parent)
                if prev is not None:
                    s += mu.v_length(mu.v_sub(p, prev))
                arcs.append(s)
                prev = p
            self._swings[key] = (turns, arcs, prev)
            for idx, e in enumerate(order):
                self._swing_of[e.id] = (key, idx)

    def _follow_plan(self, chains, nodes, done, base, scene, links):
        """横のつながり on chains hanging from an already solved chain (thin
        bundles of a ponytail): chain index -> per joint the (result,
        animation) rotations of the segment of the solved chain beside it
        (same length along the hair from the root) and its own animation."""
        plan = {}
        for k, order in enumerate(chains):
            root = order[0]
            if base[root.id] is scene[root.id] or links[k] <= 0.0:
                continue
            anc = self._nearest(nodes, done, root)
            rec = self._swing_of.get(anc.id) if anc is not None else None
            if rec is None:
                continue
            key, idx = rec
            turns, arcs, _end = self._swings[key]
            smp0 = scene[anc.id][0]
            prev = tu.pivot_world(smp0.ch, smp0.parent)
            pivots = [tu.pivot_world(scene[e.id][0].ch, scene[e.id][0].parent) for e in order]
            start = arcs[idx] + mu.v_length(mu.v_sub(pivots[0], prev))
            lengths = [mu.v_length(mu.v_sub(pivots[j + 1], pivots[j])) for j in range(len(order) - 1)]
            lengths.append(lengths[-1] if lengths else 0.0)
            beside = follow_segments(arcs, lengths, start)
            plan[k] = [(turns[i][0], turns[i][1], [tu.world_rotation(smp.world) for smp in scene[e.id]])
                       for e, i in zip(order, beside)]
        return plan

    def _calculate_rope(self, batch, eff, nodes, scene, progress, solved, done):
        """A rope ("縄・ひも"): the controllers between the two grips are
        simulated as points (translation keys); the grips follow the hands."""
        grip_a, grip_b = batch.ends
        mids = list(batch.entries)
        es = eff[mids[0].id].copy()
        # a rope moves its controllers: translation only, the rotation stays animated
        es.translate = (True, True, True)
        es.rotate = (False, False, False)
        grip_settings = es.copy()
        grip_settings.target_mode = "animation"
        if es.warp_enabled:
            LOG.warning("「%s」: 縄・ひもでは「カメラ合わせ（ワープ吸収）」は使えません（無視して計算します）。", batch.name)
        samples, hashes, ext = {}, [], []
        for e in mids:
            smp = scene[e.id]
            anc = self._nearest(nodes, done, e)
            if anc is not None:
                smp = self._carry(smp, solved[anc.id], structural=True)
                ext.append(solved[anc.id].input_hash)
            samples[e.id] = smp
        inputs = [self.build_input(grip_a, scene[grip_a.id], grip_settings)]
        inputs += [self.build_input(e, samples[e.id], es) for e in mids]
        inputs.append(self.build_input(grip_b, scene[grip_b.id], grip_settings))
        if es.collision and self._colliders:
            inputs[0].colliders = self._colliders
            found = getattr(self, "_rope_curves", {}).get(id(batch))
            if found is not None and len(found) == 5:
                shape, degree, knots, jac, rows = found
                inputs[0].rope_curve = rope_curve.RopeCurve(degree, knots, rows, jac, shape.split("|")[-1])
        undragged = self._rope_targets(batch, nodes, scene, mids, inputs)
        for e in [grip_a] + mids + [grip_b]:
            hashes.append(self.input_hash(samples.get(e.id) or scene[e.id], es, e,
                                          ("rope", e is grip_a, e is grip_b)))
        hashes.append(self._collider_sig if es.collision else "")
        rc = getattr(inputs[0], "rope_curve", None)
        hashes.append(hashlib.sha1(rc.signature().encode()).hexdigest() if rc is not None else "")
        hashes.append("undrag %d" % undragged)
        rope_hash = hashlib.sha1("|".join(hashes + ext).encode()).hexdigest()
        cached = [self.cache.get(e.id) for e in mids]
        if all(c is not None and c.input_hash == rope_hash and c.node == nodes[e.id] for c, e in zip(cached, mids)):
            LOG.info("縄「%s」: 入力が変わっていないため、前回の結果を使います", batch.name)
            return {e.id: c for e, c in zip(mids, cached)}
        self._check_rope_order(batch, [grip_a] + mids + [grip_b], inputs)

        solver = RopeSolver()
        solver.initialize()
        solver.assign(inputs)
        solver.configure(es)
        raws = solver.calculate(_SubRange(progress, 0.0, 0.9))
        solver.cleanup()
        progress.check(0.95, "変換中")
        results = {}
        for e, raw, inp in zip(mids, raws, inputs[1:-1]):
            frs = self.to_channels(samples[e.id], raw, es)
            results[e.id] = SimResult(entry_id=e.id, node=nodes[e.id], frames=[smp.frame for smp in samples[e.id]],
                                      frame_results=frs, input_hash=rope_hash, settings=es.to_dict(),
                                      tip_local=inp.tip_local, pendulum_length=0.0, info=raw.info)
        info = raws[0].info
        if info.get("settled") is False:
            LOG.warning("縄「%s」: 初期状態が落ち着ききりませんでした（30 秒ぶん揺らしても止まりませんでした）。"
                        "最初の数フレームに揺れが残ることがあります", batch.name)
        st = info.get("stretch") or [1.0]
        LOG.info("縄「%s」: 持ち手 %s〜%s、間のコントローラー %d 個、縄の長さ %.4g（%s）。伸びは最大 %d%%%s",
                 batch.name, grip_a.label, grip_b.label, len(mids), info.get("rest_length", 0.0),
                 "開始フレームの形から" if es.rope_length <= 1.0e-6 else
                 "指定の長さ。開始フレームの形は %.4g" % info.get("start_length", 0.0),
                 int(round((max(st) - 1.0) * 100.0)),
                 "、床に当たった回数 %d（内部ステップ）" % info["floor_hits"] if info.get("floor_hits") else "")
        return results

    def _find_rope_curve(self, batch, nodes):
        """The rig's curve of a rope and how the controllers move it
        (core/rope_curve_probe.py), or None (the controller points collide)."""
        grip_a, grip_b = batch.ends
        rope = [grip_a] + list(batch.entries) + [grip_b]
        try:
            found, msg = rope_curve_probe.find([nodes[e.id] for e in rope], range(1, len(rope) - 1))
        except Exception as exc:
            found, msg = None, "カーブを調べられませんでした（%s）。コントローラーの点で当たりを見ます" % exc
        LOG.info("縄「%s」: %s", batch.name, msg)
        return found

    def _rope_targets(self, batch, nodes, scene, mids, inputs):
        """Rope controllers that are carried by a grip (its children, or
        constrained / rigged to it) are swung around when the grip rotates
        (the hand turns the rope): that is the rig, not the motion of the
        rope.  Their target (硬さ / 減衰 / 追従性 / initial velocity) follows
        the grip's POSITION but keeps the grip's orientation of the first
        frame.  The animated position itself (適用率 0) is not changed.

        Carried = a child of the grip, or (any rig) its position seen from
        the grip hardly changes while it moves a lot in the world.  Returns
        how many controllers were changed."""
        grips = list(batch.ends)
        gws = {g.id: [smp.world for smp in scene[g.id]] for g in grips}
        changed = []
        for k, e in enumerate(mids):
            inp = inputs[k + 1]
            pts = inp.target_pos
            path = nodes[e.id]
            under = [g for g in grips if path.startswith(nodes[g.id] + "|")]
            g = max(under, key=lambda x: len(nodes[x.id])) if under else None
            if g is None:
                world_move = max(mu.v_length(mu.v_sub(p, pts[0])) for p in pts)
                best = None
                for cand in grips:
                    gw = gws[cand.id]
                    rel = [mu.m_transform_point(mu.m_inverse(gw[f]), p) for f, p in enumerate(pts)]
                    rel_move = max(mu.v_length(mu.v_sub(r, rel[0])) for r in rel)
                    if best is None or rel_move < best[0]:
                        best = (rel_move, cand)
                if best is not None and world_move > 1.0e-3 and best[0] < 0.05 * world_move:
                    g = best[1]
            if g is None:
                continue
            gw = gws[g.id]
            frozen = list(gw[0])
            out = []
            for f, p in enumerate(pts):
                m = list(frozen)
                m[12:15] = gw[f][12:15]
                out.append(mu.m_transform_point(mu.m_mul(mu.m_inverse(gw[f]), m), p))
            inp.target_pos = out
            changed.append("%s（%s%s）" % (e.label, g.label, "の子" if under else "に付いて動く"))
        if changed:
            LOG.info("縄「%s」: 持ち手に付いて動くコントローラー %d 個は、持ち手の回転に振り回される動きを除いて揺らします"
                     "（持ち手の位置には付いていき、向きは開始フレームのまま）: %s", batch.name, len(changed), "、".join(changed))
        return len(changed)

    @staticmethod
    def _check_rope_order(batch, entries, inputs):
        """The list must go along the rope: warn when another order of the
        middle controllers makes a clearly shorter rope (first frame)."""
        pts = [inp.orig_pos[0] for inp in inputs]

        def length(seq):
            return sum(mu.v_length(mu.v_sub(pts[seq[k + 1]], pts[seq[k]])) for k in range(len(seq) - 1))
        n = len(pts)
        listed = list(range(n))
        greedy, left = [0], set(range(1, n - 1))
        while left:
            nxt = min(left, key=lambda j: mu.v_length(mu.v_sub(pts[j], pts[greedy[-1]])))
            greedy.append(nxt)
            left.discard(nxt)
        greedy.append(n - 1)
        if length(greedy) < 0.8 * length(listed):
            LOG.warning("縄「%s」: リストの順が縄に沿っていないようです（持ち手 %s から近い順なら %s）。"
                        "▲▼で、持ち手から持ち手まで縄に沿った順に並べてください。", batch.name, entries[0].label,
                        "→".join(entries[j].label for j in greedy))

    @staticmethod
    def _auto_segment(order, idx, scene):
        """Pendulum length / tip direction (in the controller's scene
        orientation frame) from its pivot to the next controller's pivot at the
        first frame.  The last segment continues the previous direction with
        the previous length."""
        def pivot(i):
            smp = scene[order[i].id][0]
            return tu.pivot_world(smp.ch, smp.parent)
        if idx + 1 < len(order):
            v = mu.v_sub(pivot(idx + 1), pivot(idx))
        else:
            v = mu.v_sub(pivot(idx), pivot(idx - 1))
        length = mu.v_length(v)
        if length < 1.0e-6:
            raise SimflowError("チェーンの %s と隣のコントローラーの支点が同じ位置にあるため、"
                               "長さを自動で決められません。" % order[idx].label)
        smp = scene[order[idx].id][0]
        q0 = tu.world_rotation(smp.world)
        tip_local = mu.q_rotate(mu.q_conj(q0), mu.v_scale(v, 1.0 / length))
        return tip_local, length

    def calculate_entry(self, entry, settings, samples, node, deps=(), tip=None, progress=None):
        progress = progress or Progress()
        input_hash = self.input_hash(samples, settings, entry, deps)
        cached = self.cache.get(entry.id)
        if cached and cached.input_hash == input_hash and cached.node == node:
            LOG.info("%s: input unchanged - using cached result", entry.label)
            return cached
        solver_input = self.build_input(entry, samples, settings, tip)
        if settings.collision and getattr(self, "_colliders", None):
            solver_input.colliders = self._colliders
        plan = self._warp_plan(settings, solver_input, solver_input, settings.pendulum_length)
        solver = make_solver(settings)
        solver.initialize()
        solver.assign(warp_mod.sim_input(solver_input, plan.comp) if plan and plan.active else solver_input)
        solver.configure(settings)
        solver.calculate(_SubRange(progress, 0.0, 0.9))
        raw = solver.get_result()
        if plan is not None:
            if plan.active:
                raw = warp_mod.world_result(raw, plan.comp)
            raw.info["warps"] = plan.warps
            LOG.info("%s: %s", entry.label, warp_mod.describe(plan.warps))
        progress.check(0.95, "変換中 %s" % entry.label)
        frame_results = self.to_channels(samples, raw, settings)
        result = SimResult(entry_id=entry.id, node=node, frames=[smp.frame for smp in samples],
                           frame_results=frame_results, input_hash=input_hash,
                           settings=settings.to_dict(), tip_local=solver_input.tip_local,
                           pendulum_length=settings.pendulum_length, info=raw.info)
        if raw.info.get("settled") is False:
            LOG.warning("%s: " + "初期状態が落ち着ききりませんでした（最初のポーズから重力で垂れるまで 30 秒ぶん揺らしても止まりませんでした。当たりに触れている、ほぼ真上を向いているなどが考えられます）。最初の数フレームに揺れが残ることがあります", entry.label)
        solver.cleanup()
        return result

    def _warp_plan(self, settings, root, tip_inp, tip_len):
        """Warp plan of a unit (None when the absorption is off).  ``root``:
        SolverInput of the root controller (detection), ``tip_inp``: the last
        controller (its tip is a screen check point)."""
        if not settings.warp_enabled:
            return None
        screen = None
        cam = self._cameras.get(settings.warp_camera) if settings.warp_camera else None
        if cam is not None:
            rows = cam[0]
            screen = []
            for k in range(len(root.frames)):
                pts = [root.orig_pos[k]]
                if tip_len > 1.0e-6:
                    pts.append(mu.v_add(tip_inp.orig_pos[k],
                                        mu.v_scale(mu.q_rotate(tip_inp.orig_rot[k], tip_inp.tip_local), tip_len)))
                elif tip_inp is not root:
                    pts.append(tip_inp.orig_pos[k])
                screen.append([warp_mod.project(rows[k], p) for p in pts])
        return warp_mod.plan(root.frames, root.orig_pos, root.orig_rot, settings, screen)

    # ------------------------------------------------------------------
    def build_input(self, entry, samples, settings, tip=None):
        orig_pos, orig_rot = [], []
        for s in samples:
            orig_pos.append(tu.pivot_world(s.ch, s.parent))
            orig_rot.append(tu.world_rotation(s.world))
        if settings.target_mode == "rest":
            if not entry.rest:
                raise SimflowError("%s: 戻る姿勢が「レストポーズ」ですが、レストポーズがキャプチャされていません。"
                                   % entry.label)
            rest_ch = _ch_from_json(entry.rest["ch"])
            rest_local = list(tu.compose_local(rest_ch))
            tgt_pos, tgt_rot = [], []
            for s in samples:
                w = mu.m_mul(rest_local, s.parent)
                tgt_pos.append(tu.pivot_world(rest_ch, s.parent))
                tgt_rot.append(tu.world_rotation(w))
        else:
            tgt_pos, tgt_rot = list(orig_pos), list(orig_rot)

        # pendulum tip direction, expressed in the orientation frame
        aim = AIM_AXES[settings.aim_axis]
        ref_world = samples[0].world
        if settings.target_mode == "rest":
            ref_world = mu.m_mul(list(tu.compose_local(_ch_from_json(entry.rest["ch"]))), samples[0].parent)
        tip_world = mu.v_normalize(mu.m_transform_vector(ref_world, aim), aim)
        if tip is not None:
            # chain auto: ``tip`` is given in the orientation frame of the first
            # (possibly carried) sample
            tip_world = mu.q_rotate(tu.world_rotation(samples[0].world), tip)
        tip_local = mu.q_rotate(mu.q_conj(tgt_rot[0]), tip_world)
        return SolverInput(fps=scene_fps(), frames=[s.frame for s in samples],
                           target_pos=tgt_pos, target_rot=tgt_rot,
                           orig_pos=orig_pos, orig_rot=orig_rot,
                           tip_local=tip_local, name=entry.label)

    def to_channels(self, samples, raw, settings):
        out = []
        aim = AIM_AXES[settings.aim_axis]
        tmask = tuple(bool(x) for x in settings.translate)
        rmask = tuple(bool(x) for x in settings.rotate)
        for s, pos, q in zip(samples, raw.pos, raw.rot):
            q_orig = tu.world_rotation(s.world)
            p_orig = tu.pivot_world(s.ch, s.parent)
            dq = mu.q_mul(q, mu.q_conj(q_orig))
            desired_world = mu.rotate_about_pivot(s.world, p_orig, dq, pos)
            desired_local = mu.m_mul(desired_world, mu.m_inverse(s.solve_parent))
            t_new, r_new = tu.solve_channels(s.ch, desired_local, s.ch["r"], aim)
            t = tuple(t_new[i] if tmask[i] else s.ch["t"][i] for i in range(3))
            r = tuple(r_new[i] if rmask[i] else s.ch["r"][i] for i in range(3))
            final_local = list(tu.compose_local(s.ch, r=r, t=t))
            final_world = mu.m_mul(final_local, s.solve_parent)
            fch = dict(s.ch)
            fch["t"], fch["r"] = t, r
            out.append(FrameResult(
                frame=s.frame, base_t=s.ch["t"], base_r=s.ch["r"], t=t, r=r,
                final_world=final_world, orig_world=list(s.world), desired_world=desired_world,
                pivot=tu.pivot_world(fch, s.solve_parent), quat=tu.world_rotation(final_world),
                orig_pivot=p_orig, orig_quat=q_orig, scene_world=list(s.scene_world)))
        return out

    # ------------------------------------------------------------------
    def invalidate(self, entry_ids=None):
        if entry_ids is None:
            self.cache.clear()
        else:
            for i in entry_ids:
                self.cache.pop(i, None)


class _SubProgress(Progress):
    """Maps an entry's 0..1 progress into the overall progress."""

    def __init__(self, parent, index, count):
        self.parent, self.index, self.count = parent, index, count

    def update(self, fraction, message=""):
        self.parent.update((self.index + fraction) / float(self.count), message)

    def cancelled(self):
        return self.parent.cancelled()


class _SubRange(Progress):
    """Maps 0..1 into lo..hi of ``parent``; ``prefix`` is put before every
    message (which group / chain is being calculated)."""

    def __init__(self, parent, lo, hi, prefix=""):
        self.parent, self.lo, self.hi, self.prefix = parent, lo, hi, prefix

    def update(self, fraction, message=""):
        if self.prefix:
            message = self.prefix + ("：" + message if message else "")
        self.parent.update(self.lo + (self.hi - self.lo) * fraction, message)

    def cancelled(self):
        return self.parent.cancelled()
