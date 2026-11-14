# -*- coding: utf-8 -*-
"""SIMFLOW main window (PySide6, dockable through Maya's workspaceControl).

Layout (top to bottom)::

    揺れもの          list of swing groups (check = calculate / bake it)
    「<name>」の設定   the selected group: how it swings, its controllers,
                      preset, the main values; the rest folded in 詳細
    範囲              frame range / pre-roll (bake details folded)
    計算 / ベイク     act on every checked group

The window only edits :class:`SimflowSession` state and calls its methods;
no Maya / solver logic lives here.
"""
from __future__ import annotations

import json
import os
import traceback

import maya.cmds as cmds
from maya.app.general.mayaMixin import MayaQWidgetDockableMixin
import shiboken6
from PySide6 import QtCore, QtWidgets

from .. import __version__
from ..core import collider_manager
from ..core.preset_manager import PresetError
from ..core.session import SimflowSession
from ..core.settings import AIM_AXES, CHAIN_FIELDS, LABELS
from ..solvers.base_solver import Progress, SimulationCancelled, SolverError
from ..utils.logger import get_logger
from ..utils.maya_utils import SimflowError, cm_to_ui, linear_unit_name, playback_range, ui_to_cm
from . import widgets as W
from .controller_list import ControllerListWidget, GroupListWidget
from .curve_edit import CurveEdit
from . import help_text as HELP

LOG = get_logger("ui")
DELAY_UI = 15.0          # タイミング: frames shown on the slider (the calculation accepts +-30)

MODE_NAMES = {"additive": "加算レイヤー", "override": "オーバーライドレイヤー", "overwrite": "上書き"}

CALC_TEXT = "計算 / プレビュー"
CALC_PENDING_TEXT = "計算 / プレビュー（未反映の変更あり）"

BAKE_MODE_LABELS = [("additive", "加算レイヤー（推奨）"),
                    ("override", "オーバーライドレイヤー"),
                    ("overwrite", "元のアニメーションに上書き")]


def installer_path():
    """install_simflow.py next to the simflow folder this SIMFLOW runs from."""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        "install_simflow.py")


def load_installer():
    import importlib.util
    path = installer_path()
    if not os.path.isfile(path):
        raise SimflowError("インストーラーが見つかりません: %s" % path)
    spec = importlib.util.spec_from_file_location("simflow_installer", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def open_versions_dialog():
    """「バージョン」: the installer's dialog."""
    try:
        return load_installer().show_dialog()
    except Exception as exc:
        LOG.error(traceback.format_exc())
        QtWidgets.QMessageBox.warning(None, "SIMFLOW", "バージョンの切り替えの画面を開けませんでした:\n%s" % exc)
        return None


def _kill_jobs(jobs):
    """Kill scriptJobs in-place (the list object is shared with callbacks)."""
    while jobs:
        job = jobs.pop()
        try:
            if cmds.scriptJob(exists=job):
                cmds.scriptJob(kill=job, force=True)
        except Exception:  # Maya shutting down
            pass


class QtProgress(Progress):
    def __init__(self, parent, title):
        self.dialog = QtWidgets.QProgressDialog(title, "キャンセル", 0, 1000, parent)
        self.dialog.setWindowTitle("SIMFLOW")
        self.dialog.setWindowModality(QtCore.Qt.WindowModal)
        self.dialog.setMinimumDuration(300)
        self.dialog.setValue(0)

    def update(self, fraction, message=""):
        self.dialog.setValue(int(max(0.0, min(1.0, fraction)) * 1000))
        if message:
            self.dialog.setLabelText(message)
        QtWidgets.QApplication.processEvents()

    def cancelled(self):
        return self.dialog.wasCanceled()

    def close(self):
        self.dialog.setValue(1000)
        self.dialog.close()


class SimflowWindow(MayaQWidgetDockableMixin, QtWidgets.QWidget):
    OBJECT_NAME = "SimflowMainWindow"
    TITLE = "SIMFLOW"

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.setObjectName(self.OBJECT_NAME)
        self.setWindowTitle(self.TITLE)
        self.setMinimumWidth(330)
        self.session = SimflowSession()
        self._script_jobs = []
        self._group_id = None
        self._chain_id = None            # chosen chain (its root entry id): the sliders are its own values
        self._calculated_sig = None      # state at the last calculate / bake
        self._build()
        self.reload_from_scene()
        self._install_script_jobs()
        # every edit path (sliders, 詳細, range, controllers ...) is covered by
        # comparing the state, so a light poll is simpler than wiring each one
        self._pending_timer = QtCore.QTimer(self)
        self._pending_timer.timeout.connect(self._update_pending)
        self._pending_timer.start(400)

    # ==================================================================
    # construction
    # ==================================================================
    def _build(self):
        root = QtWidgets.QWidget()
        root.setObjectName("SimflowRoot")
        root.setStyleSheet(W.STYLE)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(root)
        main = QtWidgets.QVBoxLayout(root)
        main.setContentsMargins(6, 6, 6, 6)
        main.setSpacing(6)

        main.addLayout(self._build_header())

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        content = QtWidgets.QWidget()
        content.setObjectName("SimflowRoot")
        self.sections = QtWidgets.QVBoxLayout(content)
        self.sections.setContentsMargins(0, 0, 4, 0)
        self.sections.setSpacing(6)
        scroll.setWidget(content)
        main.addWidget(scroll, 1)

        self._build_groups()
        self._build_group_settings()
        self._build_chain_settings()
        self._build_colliders()
        self._build_range()
        self.sections.addStretch(1)

        main.addWidget(self._build_actions())
        # combo boxes must not dictate the panel width (docked ~330 px)
        for combo in root.findChildren(QtWidgets.QComboBox):
            combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
            combo.setMinimumContentsLength(6)
        self.status = QtWidgets.QLabel("準備完了")
        self.status.setObjectName("Status")
        self.status.setWordWrap(True)
        main.addWidget(self.status)

    def _build_header(self):
        lay = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("SIMFLOW")
        title.setObjectName("Title")
        ver = W.muted_label("v%s" % __version__)
        menu_btn = QtWidgets.QToolButton(text="⋮")
        menu_btn.setObjectName("MainMenuButton")
        menu_btn.setToolTip("メニュー（元に戻す・クリーンアップ・バージョンの切り替えなど）")
        menu_btn.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        menu_btn.setStyleSheet("QToolButton { font-size: 15px; font-weight: bold; padding: 0px 6px; }"
                               "QToolButton::menu-indicator { image: none; width: 0px; }")
        menu = QtWidgets.QMenu(menu_btn)
        menu.addAction("すべての揺れものを元に戻す", self._restore_all)
        menu.addAction("クリーンアップ...", self._cleanup)
        menu.addSeparator()
        menu.addAction("内部ステップレート（選択中の揺れもの）...", self._edit_step_rate)
        menu.addAction("シーンから再読み込み", self.reload_from_scene)
        menu.addSeparator()
        menu.addAction("バージョン...", self._versions)
        menu.addAction("SIMFLOW について", self._about)
        menu_btn.setMenu(menu)
        lay.addWidget(title)
        lay.addWidget(ver)
        lay.addStretch(1)
        self.b_help = QtWidgets.QToolButton(text="？ 値の説明")
        self.b_help.setToolTip("硬さ・減衰・重さ・追従性・適用率・重力の説明と、調整のこつ")
        self.b_help.clicked.connect(self._show_help)
        lay.addWidget(self.b_help)
        lay.addWidget(menu_btn)
        return lay

    # ---- 揺れもの ---------------------------------------------------------
    def _build_groups(self):
        sec = W.Section("揺れもの")
        self.group_list = GroupListWidget()
        self.group_list.itemSelectionChanged.connect(self._on_group_selected)
        self.group_list.enabledChanged.connect(self._on_group_enabled)
        sec.add(self.group_list)
        row = QtWidgets.QHBoxLayout()
        self.b_new = W.button("＋ 新規", "Maya で選択しているコントローラーで、新しい揺れものを作ります"
                                         "（選択順 = 根元から先端）")
        self.b_delgroup = W.button("削除", "選択中の揺れものを一覧から削除します（シーンのノードは消えません）")
        self.b_new.clicked.connect(self._new_group)
        self.group_list.itemDoubleClicked.connect(lambda it: it.parent() is None and self._rename_group())
        self.b_delgroup.clicked.connect(self._delete_group)
        for b in (self.b_new, self.b_delgroup):
            row.addWidget(b)
        sec.add_layout(row)
        self.lbl_empty = W.muted_label("揺れるコントローラーを根元から順に選択して「＋ 新規」を押してください。"
                                       "髪・尻尾・アクセサリーなど、揺れものごとに作ります。")
        sec.add(self.lbl_empty)
        self.sections.addWidget(sec)

    # ---- 選択中の揺れものの設定 ----------------------------------------------
    def _build_group_settings(self):
        self.sec_group = W.Section("設定")
        sec = self.sec_group

        mode = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel("揺れ方")
        lbl.setFixedWidth(80)
        self.rb_chain = QtWidgets.QRadioButton("つながって揺れる")
        self.rb_single = QtWidgets.QRadioButton("1 つずつ")
        self.rb_chain.setToolTip("リストの順（上が根元）に、1 本につながったものとして揺らします。"
                                 "髪・尻尾など")
        self.rb_single.setToolTip("コントローラーをそれぞれ独立に揺らします。左右の耳飾りなど")
        self.rb_chain.toggled.connect(self._on_chain_toggled)
        self.rb_rope = QtWidgets.QRadioButton("縄・ひも（両端を持つ）")
        self.rb_rope.setToolTip(HELP.TIPS["rope"])
        self.rb_rope.toggled.connect(self._on_rope_toggled)
        mode.addWidget(lbl)
        mode.addWidget(self.rb_chain)
        mode.addWidget(self.rb_single)
        mode.addStretch(1)
        sec.add_layout(mode)
        mode2 = QtWidgets.QHBoxLayout()
        mode2.addSpacing(80 + mode.spacing())
        mode2.addWidget(self.rb_rope)
        mode2.addStretch(1)
        sec.add_layout(mode2)

        self.lbl_ctl = W.muted_label("コントローラー（上が根元）")
        self.lbl_ctl.setWordWrap(True)
        sec.add(self.lbl_ctl)
        self.ctl_list = ControllerListWidget()
        self.ctl_list.selectInMaya.connect(self._select_in_maya)
        sec.add(self.ctl_list)
        row = QtWidgets.QHBoxLayout()
        b_add = W.button("選択を追加", "Maya で選択しているコントローラーを、この揺れものに追加します")
        b_rem = W.button("外す", "リストで選択したコントローラーをこの揺れものから外します（シーンのノードは消えません）")
        b_up = W.button("▲", "リストで選択したコントローラーを根元側へ")
        b_down = W.button("▼", "リストで選択したコントローラーを先端側へ")
        for b in (b_up, b_down):
            b.setFixedWidth(32)
        b_add.clicked.connect(self._add_selected)
        b_rem.clicked.connect(self._remove_selected)
        b_up.clicked.connect(lambda: self._move(-1))
        b_down.clicked.connect(lambda: self._move(+1))
        for b in (b_add, b_rem, b_up, b_down):
            row.addWidget(b)
        sec.add_layout(row)

        prow = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel("プリセット")
        lbl.setFixedWidth(80)
        self.cb_preset = QtWidgets.QComboBox()
        self.cb_preset.setToolTip("選ぶと、そのプリセットの値をこの揺れものに入れます")
        self.cb_preset.activated.connect(lambda _i: self._apply_preset())
        b_more = QtWidgets.QToolButton(text="…")
        b_more.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        pmenu = QtWidgets.QMenu(b_more)
        pmenu.addAction("初期値に戻す", self._reset_values)
        pmenu.addSeparator()
        pmenu.addAction("今の値をプリセットとして保存...", self._save_preset)
        pmenu.addAction("選んだプリセットを削除", self._delete_preset)
        pmenu.addSeparator()
        pmenu.addAction("プリセットを書き出し...", lambda: self._export_presets())
        pmenu.addAction("プリセットを読み込み...", lambda: self._import_presets())
        b_more.setMenu(pmenu)
        prow.addWidget(lbl)
        prow.addWidget(self.cb_preset, 1)
        prow.addWidget(b_more)
        sec.add_layout(prow)

        self.sl_stiffness = W.ParamSlider("硬さ", 50.0, tooltip=HELP.TIPS["stiffness"])
        self.sl_damping = W.ParamSlider("減衰", 30.0, tooltip=HELP.TIPS["damping"])
        self.sl_mass = W.ParamSlider("重さ", 50.0, tooltip=HELP.TIPS["mass"])
        self.sl_follow = W.ParamSlider("追従性", 0.0, tooltip=HELP.TIPS["follow"])
        self.sl_air = W.ParamSlider("空気抵抗", 0.0, tooltip=HELP.TIPS["air_drag"])
        self.sl_variation = W.ParamSlider("ばらつき", 0.0, tooltip=HELP.TIPS["variation"])
        self.sl_link = W.ParamSlider("横のつながり", 0.0, tooltip=HELP.TIPS["chain_link"])
        self.sl_side = W.ParamSlider("横揺れ抑え", 0.0, tooltip=HELP.TIPS["side_damp"])
        self.sl_shape = W.ParamSlider("形を保つ", 0.0, tooltip=HELP.TIPS["shape_keep"])
        self.sl_delay = W.ParamSlider("タイミング", 0.0, minimum=-DELAY_UI, maximum=DELAY_UI,
                                      tooltip=HELP.TIPS["delay"])
        self.sl_stretch = W.ParamSlider("伸び", 0.0, tooltip=HELP.TIPS["rope_stretch"])
        self.sl_strength = W.ParamSlider("適用率", 100.0, tooltip=HELP.TIPS["strength"])
        for w, name in ((self.sl_stiffness, "stiffness"), (self.sl_damping, "damping"), (self.sl_mass, "mass"),
                        (self.sl_follow, "follow"), (self.sl_air, "air_drag"), (self.sl_variation, "variation"),
                        (self.sl_link, "chain_link"), (self.sl_side, "side_damp"), (self.sl_shape, "shape_keep"),
                        (self.sl_delay, "delay"), (self.sl_stretch, "rope_stretch"),
                        (self.sl_strength, "strength")):
            w.valueChanged.connect(lambda v, n=name: self._set(n, v))
            sec.add(w)
            if w is self.sl_stiffness:
                sec.add(self._build_stiffness_curve())
            if w is self.sl_stretch:
                sec.add(self._build_rope_length())
            if w is self.sl_side:
                self.chk_side_down = QtWidgets.QCheckBox("真下を基準にする（袖など）")
                self.chk_side_down.setToolTip(HELP.TIPS["side_down"])
                self.chk_side_down.toggled.connect(lambda on: self._set("side_down", bool(on)))
                sec.add(self.chk_side_down)

        grow = QtWidgets.QHBoxLayout()
        self.chk_gravity = QtWidgets.QCheckBox("重力")
        self.chk_gravity.setFixedWidth(80)
        self.chk_gravity.toggled.connect(lambda v: (self._set("gravity_enabled", v), self._sync_enabled()))
        self.sl_gravity = W.ParamSlider("", 100.0, maximum=1000.0, slider_max=300.0, label_width=0,
                                        tooltip=HELP.TIPS["gravity"])
        self.chk_gravity.setToolTip(HELP.TIPS["gravity"])
        self.sl_gravity.label.hide()
        self.sl_gravity.valueChanged.connect(lambda v: self._set("gravity", v))
        grow.addWidget(self.chk_gravity)
        grow.addWidget(self.sl_gravity, 1)
        sec.add_layout(grow)
        crow = QtWidgets.QHBoxLayout()
        self.chk_collision = QtWidgets.QCheckBox("当たり判定")
        self.chk_collision.setToolTip(HELP.TIPS["collision"])
        self.chk_collision.toggled.connect(lambda v: (self._set("collision", bool(v)), self._sync_enabled()))
        self.sp_thick = QtWidgets.QDoubleSpinBox()
        self.sp_thick.setRange(0.0, 1000.0)
        self.sp_thick.setDecimals(2)
        self.sp_thick.setKeyboardTracking(False)
        self.sp_thick.setToolTip("当たりの表面から、この距離の手前で止めます（髪の束の太さ）")
        self.sp_thick.valueChanged.connect(lambda v: self._set("collision_thickness", ui_to_cm(v)))
        self.lbl_thick_unit = W.muted_label("")
        crow.addWidget(self.chk_collision)
        crow.addWidget(QtWidgets.QLabel("太さ"))
        crow.addWidget(self.sp_thick, 1)
        crow.addWidget(self.lbl_thick_unit)
        sec.add_layout(crow)
        # one-way collision with other swing groups
        prow = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel("押し出す揺れもの")
        lbl.setToolTip(HELP.TIPS["collide_with"])
        self.b_push = QtWidgets.QToolButton()
        self.b_push.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self.b_push.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.b_push.setToolTip(HELP.TIPS["collide_with"])
        self.push_menu = QtWidgets.QMenu(self.b_push)
        self.push_menu.aboutToShow.connect(self._fill_push_menu)
        self.b_push.setMenu(self.push_menu)
        prow.addWidget(lbl)
        prow.addWidget(self.b_push, 1)
        sec.add_layout(prow)
        self.lbl_push_result = W.muted_label("")
        self.lbl_push_result.setWordWrap(True)
        self.lbl_push_result.setToolTip(
            "直前の計算で、この揺れものの骨が、押し出す揺れものの骨や布の面にどれだけ入ったか（適用率を混ぜた最終の結果で測ります）。\n"
            "「食い込みなし」なのにメッシュが貫通して見えるときは、骨は当たっていません。メッシュの太さに合わせて「太さ」を上げるか、\n"
            "布の面がない所（端のチェーンの外側、最後の段より下）でないか確認してください。\n"
            "フレームごとの値はスクリプトエディタに出ます。")
        sec.add(self.lbl_push_result)
        self.chk_push_view = QtWidgets.QCheckBox("当たりを表示（押し出す揺れものの骨と布の面）")
        self.chk_push_view.setToolTip("直前の計算で使った当たりを、ビューポートに半透明の赤で表示します（タイムラインに合わせて動きます）。\n"
                                      "骨は押し出す揺れものの「太さ」の円柱、布の面は隣り合うチェーンの間の三角形です。\n"
                                      "当たる距離は、これにこの揺れものの「太さ」を足した所です。")
        self.chk_push_view.toggled.connect(self._toggle_push_view)
        sec.add(self.chk_push_view)

        sec.add(self._build_warp())

        # ---- 詳細（折りたたみ） ----
        self.sec_detail = W.Section("詳細", expanded=False)
        d = self.sec_detail
        self.cb_start = QtWidgets.QComboBox()
        self.cb_start.addItem("アニメーションの姿勢", "animated")
        self.cb_start.addItem("落ち着かせてから開始", "settled")
        self.cb_start.setToolTip("「落ち着かせてから開始」は、重力で垂れた状態から始めます（開始直後に落下しません）")
        self.cb_start.currentIndexChanged.connect(lambda _: self._set("start_state", self.cb_start.currentData()))
        d.add(W.labeled("初期状態", self.cb_start))
        self.cb_initvel = QtWidgets.QComboBox()
        self.cb_initvel.addItem("アニメーションから", "animation")
        self.cb_initvel.addItem("0（静止）", "zero")
        self.cb_initvel.currentIndexChanged.connect(
            lambda _: self._set("initial_velocity", self.cb_initvel.currentData()))
        d.add(W.labeled("初速度", self.cb_initvel))
        self.sp_seed = QtWidgets.QSpinBox()
        self.sp_seed.setRange(1, 9999)
        self.sp_seed.setKeyboardTracking(False)
        self.sp_seed.setToolTip("「ばらつき」の出方を変えます（番号を変えると、どの骨が硬めになるかが入れ替わります）")
        self.sp_seed.valueChanged.connect(lambda v: self._set("variation_seed", int(v)))
        d.add(W.labeled("ばらつきの型", self.sp_seed))
        self.chk_chain_auto = QtWidgets.QCheckBox("長さと向きはコントローラーの位置から自動で決める")
        self.chk_chain_auto.setToolTip("つながって揺れるとき、各段の長さと向きを次のコントローラーまでの距離と方向から決めます")
        self.chk_chain_auto.toggled.connect(self._on_chain_auto_toggled)
        d.add(self.chk_chain_auto)
        self.sp_length = QtWidgets.QDoubleSpinBox()
        self.sp_length.setRange(0.0, 100000.0)
        self.sp_length.setDecimals(3)
        self.sp_length.setKeyboardTracking(False)
        self.sp_length.valueChanged.connect(lambda v: self._set("pendulum_length", ui_to_cm(v)))
        self.length_row = QtWidgets.QWidget()
        lrow = QtWidgets.QHBoxLayout(self.length_row)
        lrow.setContentsMargins(0, 0, 0, 0)
        lbl = QtWidgets.QLabel("振り子の長さ")
        lbl.setFixedWidth(80)
        lrow.addWidget(lbl)
        lrow.addWidget(self.sp_length, 1)
        self.lbl_unit = W.muted_label("")
        lrow.addWidget(self.lbl_unit)
        d.add(self.length_row)
        self.cb_aim = QtWidgets.QComboBox()
        for k in AIM_AXES:
            self.cb_aim.addItem(k, k)
        self.cb_aim.setToolTip("支点から先端へ向かう、コントローラーのローカル軸")
        self.cb_aim.currentIndexChanged.connect(lambda _: self._set("aim_axis", self.cb_aim.currentData()))
        self.aim_row = W.labeled("先端の軸", self.cb_aim)
        d.add(self.aim_row)
        # which channels are written (all three axes together: a part of
        # the axes cannot hold the physical result)
        self.ch_row = QtWidgets.QWidget()
        crow = QtWidgets.QHBoxLayout(self.ch_row)
        crow.setContentsMargins(0, 0, 0, 0)
        lbl = QtWidgets.QLabel("動かす値")
        lbl.setFixedWidth(80)
        self.chk_rot = QtWidgets.QCheckBox("回転")
        self.chk_tr = QtWidgets.QCheckBox("移動")
        self.chk_rot.setToolTip("コントローラーの回転（X・Y・Z）に結果を入れます")
        self.chk_tr.setToolTip("コントローラーの移動（X・Y・Z）にも結果を入れます（支点が動くリグなど）")
        self.chk_rot.toggled.connect(lambda v: self._set("rotate", (bool(v),) * 3))
        self.chk_tr.toggled.connect(lambda v: self._set("translate", (bool(v),) * 3))
        crow.addWidget(lbl)
        crow.addWidget(self.chk_rot)
        crow.addWidget(self.chk_tr)
        crow.addStretch(1)
        d.add(self.ch_row)
        sec.add(d)
        self.sections.addWidget(sec)

    def _build_warp(self):
        """カメラ合わせ（ワープ吸収）の折りたたみ欄。"""
        self.sec_warp = W.Section("カメラ合わせ（ワープ吸収）", expanded=False)
        w = self.sec_warp
        self.chk_warp = QtWidgets.QCheckBox("1 フレームで大きく動かしたところ（ワープ）を吸収する")
        self.chk_warp.setToolTip(HELP.TIPS["warp"])
        self.chk_warp.toggled.connect(lambda v: (self._set("warp_enabled", bool(v)), self._sync_enabled(),
                                                 self.refresh_warp_result()))
        w.add(self.chk_warp)

        def dspin(lo, hi, dec, tip):
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(dec)
            sp.setKeyboardTracking(False)
            sp.setToolTip(tip)
            return sp
        # (one value per row: the panel is docked ~330 px wide)
        def unit_row(label, spin, unit):
            row = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel(label)
            lbl.setFixedWidth(80)
            row.addWidget(lbl)
            row.addWidget(spin, 1)
            row.addWidget(unit)
            w.add_layout(row)
        self.sp_warp_dist = dspin(0.001, 1.0e6, 2, "前後のフレームの動きから、これ以上飛び出している移動をワープとみなします")
        self.sp_warp_dist.valueChanged.connect(lambda v: self._set("warp_distance", ui_to_cm(v)))
        self.lbl_warp_unit = W.muted_label("")
        unit_row("ワープの移動", self.sp_warp_dist, self.lbl_warp_unit)
        self.sp_warp_angle = dspin(0.1, 180.0, 1, "前後のフレームの動きから、これ以上飛び出している向きの変化をワープとみなします"
                                   "（吸収するのは重力の軸まわりの回転＝向きの変更だけです）")
        self.sp_warp_angle.valueChanged.connect(lambda v: self._set("warp_angle", float(v)))
        unit_row("ワープの向き", self.sp_warp_angle, W.muted_label("°"))
        self.sp_warp_frames = QtWidgets.QSpinBox()
        self.sp_warp_frames.setRange(1, 3)
        self.sp_warp_frames.setKeyboardTracking(False)
        self.sp_warp_frames.setToolTip("数フレームかけて位置を直している場合は 2〜3 にします")
        self.sp_warp_frames.valueChanged.connect(lambda v: self._set("warp_frames", int(v)))
        w.add(W.labeled("フレーム数", self.sp_warp_frames))
        self.sl_warp_strength = W.ParamSlider("吸収の強さ", 100.0,
                                              tooltip="100 = ワープの分の揺れを全部なくす、50 = 半分残す")
        self.sl_warp_strength.valueChanged.connect(lambda v: self._set("warp_strength", v))
        w.add(self.sl_warp_strength)

        crow = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel("カメラ")
        lbl.setFixedWidth(80)
        self.cb_warp_cam = QtWidgets.QComboBox()
        self.cb_warp_cam.setToolTip("指定すると、画面ではっきり見えている速い動き（殴る、飛びのくなど）は"
                                    "ワープとみなさず、ふつうに揺らします")
        self.cb_warp_cam.currentIndexChanged.connect(self._on_warp_camera)
        b_cam = QtWidgets.QToolButton(text="↻")
        b_cam.setToolTip("カメラの一覧を更新します")
        b_cam.clicked.connect(self._fill_warp_cameras)
        crow.addWidget(lbl)
        crow.addWidget(self.cb_warp_cam, 1)
        crow.addWidget(b_cam)
        w.add_layout(crow)
        self.sp_warp_screen = dspin(0.0, 100.0, 1, "画面の横幅に対して、これより大きく見える飛びは"
                                    "「見えている動き」として吸収しません")
        self.sp_warp_screen.valueChanged.connect(lambda v: self._set("warp_screen", float(v)))
        unit_row("画面の境目", self.sp_warp_screen, W.muted_label("%"))
        self.le_warp_add = QtWidgets.QLineEdit()
        self.le_warp_add.setPlaceholderText("例: 37, 52, 60-62")
        self.le_warp_add.setToolTip("自動で見つからないワープのフレーム（キャラが新しい位置に来た最初のフレーム）")
        self.le_warp_add.editingFinished.connect(lambda: self._set("warp_add", self.le_warp_add.text().strip()))
        w.add(W.labeled("ワープにする", self.le_warp_add))
        self.le_warp_ignore = QtWidgets.QLineEdit()
        self.le_warp_ignore.setPlaceholderText("例: 80")
        self.le_warp_ignore.setToolTip("自動で見つかっても、吸収しないフレーム（演技として揺らしたい動き）")
        self.le_warp_ignore.editingFinished.connect(
            lambda: self._set("warp_ignore", self.le_warp_ignore.text().strip()))
        w.add(W.labeled("ワープにしない", self.le_warp_ignore))
        self.lbl_warp_result = W.muted_label("")
        w.add(self.lbl_warp_result)
        self._warp_widgets = (self.sp_warp_dist, self.sp_warp_angle, self.sp_warp_frames, self.sl_warp_strength,
                              self.cb_warp_cam, self.sp_warp_screen, self.le_warp_add, self.le_warp_ignore)
        return w

    def _fill_warp_cameras(self):
        from ..core import camera_view
        g = self.current_group()
        current = g.settings.warp_camera if g else ""
        self.cb_warp_cam.blockSignals(True)
        self.cb_warp_cam.clear()
        self.cb_warp_cam.addItem("使わない", "")
        try:
            names = camera_view.cameras()
        except Exception:
            names = []
        for n in names:
            self.cb_warp_cam.addItem(n, n)
        if current and self.cb_warp_cam.findData(current) < 0:
            self.cb_warp_cam.addItem("%s（見つかりません）" % current, current)
        self.cb_warp_cam.setCurrentIndex(max(0, self.cb_warp_cam.findData(current)))
        self.cb_warp_cam.blockSignals(False)
        self._sync_enabled()

    def _on_warp_camera(self, _):
        self._set("warp_camera", self.cb_warp_cam.currentData() or "")
        self._sync_enabled()

    def refresh_warp_result(self):
        """検出したワープ（直前の計算結果）を表示する。"""
        from ..core import warp
        g = self.current_group()
        if g is None or not g.settings.warp_enabled:
            self.lbl_warp_result.setText("")
            return
        entries = self.session.controllers.group_entries(g.id)
        found = {}
        calculated = False
        for e in entries:
            r = self.session.sim.cache.get(e.id)
            if r is None or "warps" not in r.info:      # not calculated with the absorption on
                continue
            calculated = True
            for wp in r.info.get("warps") or []:
                found.setdefault((wp["frame"], wp["status"]), wp)
        if not calculated:
            self.lbl_warp_result.setText("計算すると、見つけたワープのフレームをここに表示します")
            return
        self.lbl_warp_result.setText("直前の計算：" + warp.describe(sorted(found.values(), key=lambda x: x["frame"])))

    def _build_stiffness_curve(self):
        """Chain: how the stiffness is spread from the root to the tip.  Rope:
        from the grips to the middle of the rope (the same on both halves)."""
        self.curve_box = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(self.curve_box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(3)
        self.lbl_curve = W.muted_label("硬さのカーブ（根元 → 先端、上 = 硬さの 100%）")
        lay.addWidget(self.lbl_curve)
        self.curve_edit = CurveEdit()
        self.curve_edit.curveChanged.connect(lambda pts: self._set("stiffness_curve", pts))
        self.curve_edit.curveChanged.connect(lambda _pts: self._sync_curve_spin())
        self.curve_edit.selectionChanged.connect(lambda _i: self._sync_curve_spin())
        lay.addWidget(self.curve_edit)
        row = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel("選んだ点")
        lbl.setFixedWidth(80)
        self.sp_curve = QtWidgets.QSpinBox()
        self.sp_curve.setRange(0, 100)
        self.sp_curve.setSuffix(" %")
        self.sp_curve.setKeyboardTracking(False)
        self.sp_curve.valueChanged.connect(lambda v: self.curve_edit.setSelectedValue(v / 100.0))
        b_flat = QtWidgets.QToolButton(text="平らにする")
        b_flat.setToolTip("カーブを平ら（根元から先端まで同じ硬さ）に戻します")
        b_flat.clicked.connect(self._flatten_curve)
        row.addWidget(lbl)
        row.addWidget(self.sp_curve, 1)
        row.addWidget(b_flat)
        lay.addLayout(row)
        return self.curve_box

    def _sync_curve_spin(self):
        self.sp_curve.blockSignals(True)
        self.sp_curve.setValue(int(round(self.curve_edit.selectedValue() * 100)))
        self.sp_curve.blockSignals(False)

    def _flatten_curve(self):
        self.curve_edit.setPoints(((0.0, 1.0), (1.0, 1.0)))
        self._set("stiffness_curve", self.curve_edit.points())
        self._sync_curve_spin()

    # ---- 範囲 ----------------------------------------------------------------
    def _build_range(self):
        sec = W.Section("範囲")
        row = QtWidgets.QHBoxLayout()
        self.sp_start = QtWidgets.QSpinBox()
        self.sp_end = QtWidgets.QSpinBox()
        for sp in (self.sp_start, self.sp_end):
            sp.setRange(-100000, 100000)
            sp.setKeyboardTracking(False)
            # the 6-digit range must not dictate the panel width (docked ~330 px)
            sp.setMinimumWidth(48)
            sp.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.sp_start.valueChanged.connect(lambda v: setattr(self.session.bake_options, "start", float(v)))
        self.sp_end.valueChanged.connect(lambda v: setattr(self.session.bake_options, "end", float(v)))
        lbl = QtWidgets.QLabel("フレーム")
        row.addWidget(lbl)
        row.addWidget(self.sp_start, 1)
        row.addWidget(QtWidgets.QLabel("〜"))
        row.addWidget(self.sp_end, 1)
        b_range = QtWidgets.QToolButton(text="再生範囲")
        b_range.setToolTip("タイムラインの再生範囲を使います")
        b_range.clicked.connect(self._use_playback_range)
        row.addWidget(b_range)
        sec.add_layout(row)
        self.sp_preroll = QtWidgets.QSpinBox()
        self.sp_preroll.setRange(0, 10000)
        self.sp_preroll.setSuffix(" フレーム")
        self.sp_preroll.setToolTip("開始フレームより前から計算を始めるフレーム数（ベイクするのは範囲の中だけ）")
        self.sp_preroll.valueChanged.connect(lambda v: setattr(self.session.bake_options, "preroll", int(v)))
        sec.add(W.labeled("プリロール", self.sp_preroll))

        self.sec_bake = W.Section("ベイクの詳細", expanded=False)
        b = self.sec_bake
        self.cb_preroll = QtWidgets.QComboBox()
        self.cb_preroll.addItem("アニメーションをそのまま使う", "animation")
        self.cb_preroll.addItem("開始フレームの姿勢で静止", "hold")
        self.cb_preroll.setToolTip("プリロール区間の入力")
        self.cb_preroll.currentIndexChanged.connect(
            lambda _: setattr(self.session.bake_options, "preroll_mode", self.cb_preroll.currentData()))
        b.add(W.labeled("プリロール区間", self.cb_preroll))
        self.cb_mode = QtWidgets.QComboBox()
        for key, text in BAKE_MODE_LABELS:
            self.cb_mode.addItem(text, key)
        self.cb_mode.currentIndexChanged.connect(self._on_mode_changed)
        b.add(W.labeled("出力先", self.cb_mode))
        self.le_layer = QtWidgets.QLineEdit()
        self.le_layer.editingFinished.connect(
            lambda: setattr(self.session.bake_options, "layer_name", self.le_layer.text().strip()))
        b.add(W.labeled("レイヤー名", self.le_layer))
        sec.add(b)
        self.sections.addWidget(sec)

    def _build_actions(self):
        box = QtWidgets.QWidget()
        lay = QtWidgets.QGridLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        self.lbl_targets = W.muted_label("")
        self.b_calc = W.accent_button(CALC_TEXT, "チェックを入れた揺れものを計算し、プレビュー用のプロキシを更新します")
        self.b_bake = W.accent_button("ベイク", "チェックを入れた揺れものを、元のアニメーションから計算し直してベイクします"
                                              "（検証付き、Undo できます）")
        self.b_restore = W.button("元に戻す", "選択中の揺れものから SIMFLOW のベイクを取り除きます")
        self.b_reset = W.button("プレビューを消す", "計算結果とプレビュー用のプロキシを削除し、リグへの反映も外します")
        self.chk_orig = QtWidgets.QCheckBox("元の動き（灰色）も表示")
        self.chk_orig.setToolTip("計算結果（黄色）と一緒に、元のアニメーション（灰色）のプロキシも表示します")
        self.chk_orig.setChecked(True)
        self.chk_rig = QtWidgets.QCheckBox("リグで見る（ベイク前に実際に動かす）")
        self.chk_rig.setToolTip("オンにして「計算 / プレビュー」を押すと、計算結果を SIMFLOW 専用の一時レイヤー（SIMFLOW_Preview）で\n"
                                "実際のコントローラーに当てます（チェックを入れただけでは計算しません）。\n"
                                "メッシュも一緒に動くので、ベイク前に見た目を確認できます。元のカーブやリグの接続は変えません。\n"
                                "オフにするか「プレビューを消す」で外れます。ベイクすると自動で外れます。")
        lay.addWidget(self.lbl_targets, 0, 0, 1, 2)
        lay.addWidget(self.b_calc, 1, 0, 1, 2)
        lay.addWidget(self.b_bake, 2, 0, 1, 2)
        lay.addWidget(self.b_restore, 3, 0)
        lay.addWidget(self.b_reset, 3, 1)
        tog = QtWidgets.QHBoxLayout()
        tog.addWidget(self.chk_orig)
        tog.addStretch(1)
        lay.addWidget(self.chk_rig, 4, 0, 1, 2)
        lay.addLayout(tog, 5, 0, 1, 2)
        self.chk_rig.toggled.connect(self._toggle_rig_preview)
        self.b_calc.clicked.connect(self._calculate)
        self.b_bake.clicked.connect(self._bake)
        self.b_restore.clicked.connect(self._restore)
        self.b_reset.clicked.connect(self._reset)
        self.chk_orig.toggled.connect(lambda v: self._guard(lambda: self.session.preview.show_original(v)))
        return box

    # ==================================================================
    # state <-> widgets
    # ==================================================================
    def current_group(self):
        return self.session.controllers.get_group(self._group_id) if self._group_id else None

    def _set(self, name, value):
        g = self.current_group()
        if g is not None:
            self.session.set_value(name, value, g.id)

    # ---- チェーンごとの値 ----------------------------------------------------------
    def _build_chain_settings(self):
        self.sec_chain = W.Section("チェーンの値")
        sec = self.sec_chain
        note = W.muted_label("このチェーンだけの値です。揺れもの全体の値の上に重ねて使います。"
                             "全体と違う値は名前の前に ● が付きます（名前を右クリックで、その値だけ全体に戻す）。")
        note.setToolTip(HELP.TIPS["chain_own"])
        sec.add(note)
        self.chain_sliders = {}
        specs = (("stiffness", "硬さ", {}), ("damping", "減衰", {}), ("mass", "重さ", {}),
                 ("follow", "追従性", {}), ("air_drag", "空気抵抗", {}), ("chain_link", "横のつながり", {}),
                 ("side_damp", "横揺れ抑え", {}), ("shape_keep", "形を保つ", {}),
                 ("delay", "タイミング", dict(minimum=-DELAY_UI, maximum=DELAY_UI)),
                 ("strength", "適用率", {}), ("gravity", "重力", dict(maximum=1000.0, slider_max=300.0)))
        for name, label, kw in specs:
            w = W.ParamSlider(label, 0.0, tooltip=HELP.TIPS.get(name, ""), **kw)
            w.valueChanged.connect(lambda v, n=name: self._set_chain(n, v))
            w.label.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
            w.label.customContextMenuRequested.connect(lambda pos, n=name, x=w: self._chain_value_menu(n, x, pos))
            self.chain_sliders[name] = w
            sec.add(w)
        row = QtWidgets.QHBoxLayout()
        self.b_chain_reset = W.button("全体の値に戻す", "このチェーンの値を、すべて揺れもの全体の値に戻します")
        self.b_chain_reset.clicked.connect(self._reset_chain)
        b_back = W.button("揺れもの全体の設定へ", "一覧で揺れもの（親の行）を選び、全体の設定に戻ります")
        b_back.clicked.connect(self._back_to_group)
        row.addWidget(self.b_chain_reset)
        row.addWidget(b_back)
        sec.add_layout(row)
        sec.setVisible(False)
        self.sections.addWidget(sec)

    def current_chain(self):
        """The chosen chain (list of entries) of the current group, or None."""
        if not self._chain_id or not self._group_id:
            return None
        for c in self.session.chains(self._group_id):
            if c[0].id == self._chain_id:
                return c
        return None

    def _set_chain(self, name, value):
        g = self.current_group()
        c = self.current_chain()
        if g is None or c is None:
            return
        self._guard(lambda: self.session.set_chain_value(g.id, c[0].id, name, value))
        self._mark_chain_values()
        self.refresh_groups()

    def _chain_value_menu(self, name, widget, pos):
        g = self.current_group()
        c = self.current_chain()
        if g is None or c is None:
            return
        menu = QtWidgets.QMenu(self)
        act = menu.addAction("全体の値（%g）に戻す" % getattr(g.settings, name))
        act.setEnabled(name in (g.chain_overrides.get(c[0].id) or {}))
        if menu.exec_(widget.label.mapToGlobal(pos)) is act:
            self.session.clear_chain_values(g.id, c[0].id, name)
            self.refresh_group_panel()
            self.refresh_groups()

    def _reset_chain(self):
        g = self.current_group()
        c = self.current_chain()
        if g is None or c is None:
            return
        self.session.clear_chain_values(g.id, c[0].id)
        self.refresh_group_panel()
        self.refresh_groups()
        self.set_status("チェーン %s〜 の値を、揺れもの全体の値に戻しました。" % c[0].label)

    def _back_to_group(self):
        self._chain_id = None
        self.refresh_groups()
        self.refresh_group_panel()

    def _mark_chain_values(self):
        g = self.current_group()
        c = self.current_chain()
        own = (g.chain_overrides.get(c[0].id) or {}) if g is not None and c is not None else {}
        for name, w in self.chain_sliders.items():
            base = LABELS.get(name, name)
            if name == "delay":
                base = "タイミング"
            w.label.setText(("● " + base) if name in own else base)
            w.label.setStyleSheet(("color: %s; font-weight: bold;" % W.ACCENT) if name in own else "")

    def refresh_chain_panel(self, g, chain):
        chains = self.session.chains(g.id)
        k = [x[0].id for x in chains].index(chain[0].id) + 1
        self.sec_chain.header.setText("「%s」チェーン %d：%s〜%s の値" % (g.name, k, chain[0].label, chain[-1].label))
        s = self.session.chain_settings(g.id, chain[0].id)
        for name, w in self.chain_sliders.items():
            w.setValue(getattr(s, name))
        self._mark_chain_values()
        link = self.chain_sliders["chain_link"]
        link.setEnabled(len(chains) > 1)
        link.setToolTip(HELP.TIPS["chain_link"] if len(chains) > 1 else
                        "チェーンが 2 本以上あるときに使えます。\n\n" + HELP.TIPS["chain_link"])
        grav = self.chain_sliders["gravity"]
        grav.setEnabled(bool(g.settings.gravity_enabled))
        grav.setToolTip(HELP.TIPS["gravity"] if g.settings.gravity_enabled else
                        "揺れもの全体の設定で重力をオンにすると使えます。\n\n" + HELP.TIPS["gravity"])

    # ---- 押し出す揺れもの -------------------------------------------------------
    def _push_ids(self):
        g = self.current_group()
        if g is None:
            return []
        known = {x.id for x in self.session.controllers.groups}
        return [x for x in g.settings.collide_with if x in known and x != g.id]

    def _fill_push_menu(self):
        self.push_menu.clear()
        g = self.current_group()
        others = [x for x in self.session.controllers.groups if g is not None and x.id != g.id]
        if not others:
            a = self.push_menu.addAction("（ほかの揺れものがありません）")
            a.setEnabled(False)
            return
        chosen = set(self._push_ids())
        for other in others:
            a = self.push_menu.addAction(other.name)
            a.setCheckable(True)
            a.setChecked(other.id in chosen)
            a.toggled.connect(lambda on, gid=other.id: self._toggle_push(gid, on))

    def _toggle_push(self, gid, on):
        ids = [x for x in self._push_ids() if x != gid]
        if on:
            ids.append(gid)
        self._set("collide_with", tuple(ids))
        self._refresh_push_label()
        self.refresh_push_result()
        self._update_pending()

    def refresh_push_result(self):
        g = self.current_group()
        report = self.session.push_report(g.id) if g else None
        if g is None or not self._push_ids() or not g.settings.collision:
            self.lbl_push_result.setText("")
            self.lbl_push_result.setVisible(False)
        else:
            from ..core import collision_check
            self.lbl_push_result.setVisible(True)
            self.lbl_push_result.setText("直前の計算：" + collision_check.describe(
                report, strength=report.get("strength", 100.0), rotate=report.get("rotate", (True, True, True)))
                if report is not None else
                                         "計算すると、骨が当たりに食い込んだフレームをここに表示します")
        self.chk_push_view.blockSignals(True)
        self.chk_push_view.setChecked(bool(g and self.session.collision_view_gid == g.id))
        self.chk_push_view.blockSignals(False)
        self.chk_push_view.setEnabled(bool(g and self._push_ids() and g.settings.collision))

    def _toggle_push_view(self, on):
        g = self.current_group()
        if g is None:
            return
        n = self._guard(lambda: self.session.set_collision_view(g.id if on else None))
        if on and not n:
            self.chk_push_view.blockSignals(True)
            self.chk_push_view.setChecked(False)
            self.chk_push_view.blockSignals(False)
        elif on:
            self.set_status("「%s」の当たり（骨と布の面 %d 個）を表示しました。タイムラインをスクラブして確認してください。" % (g.name, n))

    def _refresh_push_label(self):
        names = {x.id: x.name for x in self.session.controllers.groups}
        chosen = [names[x] for x in self._push_ids()]
        text = "、".join(chosen) if chosen else "なし"
        fm = self.b_push.fontMetrics()
        self.b_push.setText(fm.elidedText(text, QtCore.Qt.ElideRight, max(60, self.b_push.width() - 24)))
        tip = HELP.TIPS["collide_with"]
        if chosen:
            tip = "選んでいる揺れもの：" + text + "\n\n" + tip
        self.b_push.setToolTip(tip)

    def _sync_enabled(self):
        self.sl_gravity.setEnabled(self.chk_gravity.isChecked())
        if hasattr(self, "b_push"):
            self.b_push.setEnabled(self.chk_collision.isChecked())
        if hasattr(self, "lbl_push_result"):
            self.refresh_push_result()
        if hasattr(self, "_warp_widgets"):
            for x in self._warp_widgets:
                x.setEnabled(self.chk_warp.isChecked())
            self.sp_warp_screen.setEnabled(self.chk_warp.isChecked() and bool(self.cb_warp_cam.currentData()))
        g = self.current_group()
        rope = bool(g and g.rope)
        chain = bool(g and g.chain and not rope)
        # the stiffness curve: a chain (root -> tip) or a rope (grips -> middle)
        self.curve_box.setVisible(chain or rope)
        self.lbl_curve.setText("硬さのカーブ（持ち手 → 縄の真ん中、左右同じ。上 = 硬さの 100%）" if rope
                               else "硬さのカーブ（根元 → 先端、上 = 硬さの 100%）")
        self.curve_edit.ends = ("持ち手", "真ん中") if rope else ("根元", "先端")
        self.curve_edit.update()
        auto = bool(chain and g.chain_auto)
        # shown only for chains that had it turned off (older scenes): there
        # it can be turned back on; new chains always decide it automatically
        self.chk_chain_auto.setVisible(chain and not g.chain_auto)
        self.sl_stretch.setVisible(rope)
        self.rope_len_row.setVisible(rope)
        # values that do nothing for this kind of group are hidden (not greyed
        # out: the docked panel stays short)
        self.sl_variation.setVisible(not rope)
        self.sl_side.setVisible(not rope)
        self.chk_side_down.setVisible(not rope)
        # 形を保つ / タイミング: the joints between the controllers of a chain
        self.sl_delay.setVisible(chain)
        self.sl_shape.setVisible(chain)
        self.lbl_ctl.setText("コントローラー（縄に沿って、持ち手から持ち手まで。いちばん上と下が持ち手）" if rope
                             else "コントローラー（上が根元）")
        # a rope moves the controllers' translation only (fixed)
        self.ch_row.setVisible(not rope)
        # "横のつながり" links the chains of the group: needs 2 or more chains
        n_chains = 0
        if chain:
            from .controller_list import _chain_count
            entries = self.session.controllers.group_entries(g.id)
            n_chains = _chain_count(entries) if len(entries) > 1 else 0
        self.sl_link.setVisible(n_chains > 1)
        # 振り子の長さ / 先端の軸: only where they are used (1 つずつ, or a chain
        # without the automatic length and direction)
        for w in (self.length_row, self.aim_row):
            w.setVisible(not auto and not rope)

    def refresh_all(self):
        self.chk_rig.blockSignals(True)
        self.chk_rig.setChecked(self.session.rig_preview)
        self.chk_rig.blockSignals(False)
        self.refresh_groups()
        self.refresh_group_panel()
        self.refresh_range()
        self.refresh_presets()
        self.refresh_colliders()

    # ---- 当たり判定 ------------------------------------------------------------
    def _build_colliders(self):
        sec = W.Section("当たり判定")
        self.col_list = QtWidgets.QListWidget()
        self.col_list.setMinimumHeight(52)
        self.col_list.setMaximumHeight(110)
        self.col_list.setToolTip("ダブルクリックで Maya 上で選択（位置・向き・大きさを調整できます）")
        self.col_list.itemDoubleClicked.connect(self._select_collider)
        sec.add(self.col_list)
        row = QtWidgets.QHBoxLayout()
        b_sphere = W.button("＋ 球", "Maya で選んでいるノード（頭のジョイントなど）に付いて動く球を作ります")
        b_capsule = W.button("＋ カプセル", "Maya で選んでいるノード（首・腕・胴など）に付いて動くカプセルを作ります")
        b_floor = W.button("＋ 床", "Maya で選んでいる床のノード（床のメッシュやロケーター）に付いて動く床を作ります。\n"
                                    "床のノードの位置を通り、Y 軸を上向きとする、どこまでも広がる平面として当たります。\n"
                                    "高さや傾きは、作った床を移動・回転して合わせます（表示の大きさは見た目だけです）")
        b_floor.clicked.connect(lambda: self._add_collider("floor"))
        self.b_col_del = W.button("当たりを削除", "一覧で選んだ当たりを削除します")
        b_sphere.clicked.connect(lambda: self._add_collider("sphere"))
        b_capsule.clicked.connect(lambda: self._add_collider("capsule"))
        self.b_col_del.clicked.connect(self._remove_collider)
        for b in (b_sphere, b_capsule, b_floor, self.b_col_del):
            row.addWidget(b)
        sec.add_layout(row)
        sec.add(W.muted_label("頭などのノードを選んでから追加すると、そのノードに付いて動きます。"
                              "位置と向きは移動・回転、大きさはチャンネルボックスの「半径」「長さ」で合わせます。"
                              "当たりは「クリーンアップ」では消えません。"))
        self.sections.addWidget(sec)

    def refresh_colliders(self):
        try:
            infos = self.session.colliders()
        except Exception as exc:  # scene being torn down
            LOG.warning(str(exc))
            infos = []
        self._collider_nodes = [i["node"] for i in infos]
        self.col_list.clear()
        for i in infos:
            text = "%s    %s%s" % (i["name"], collider_manager.KIND_NAMES.get(i["kind"], i["kind"]),
                                  "・%s に付く" % i["attach"] if i["attach"] else "")
            item = QtWidgets.QListWidgetItem(text)
            item.setData(QtCore.Qt.UserRole + 1, i["node"])
            self.col_list.addItem(item)
        self.b_col_del.setEnabled(bool(infos))

    def _add_collider(self, kind):
        col = self._guard(lambda: self.session.add_collider(kind))
        if col:
            self.refresh_colliders()
            self.set_status("当たり（%s）を作りました。チャンネルボックスで位置と大きさを合わせてください。"
                            % collider_manager.KIND_NAMES.get(kind, kind) if kind != "floor" else
                            "床を作りました。床の面に合うよう、移動・回転で高さと傾きを合わせてください。")

    def _selected_collider(self):
        items = self.col_list.selectedItems()
        return items[0].data(QtCore.Qt.UserRole + 1) if items else None

    def _select_collider(self, item):
        node = item.data(QtCore.Qt.UserRole + 1)
        self._guard(lambda: cmds.select(node, replace=True))

    def _remove_collider(self):
        node = self._selected_collider()
        if not node:
            self.set_status("削除する当たりを一覧で選んでください。")
            return
        self._guard(lambda: self.session.remove_collider(node), "当たりを削除しました。")
        self.refresh_colliders()

    def refresh_groups(self):
        groups = self.session.controllers.groups
        if self._group_id not in [g.id for g in groups]:
            self._group_id = groups[0].id if groups else None
        if self._chain_id and self.current_chain() is None:
            self._chain_id = None          # the chain is gone (controllers changed, undo ...)
        self.group_list.set_groups(groups, self.session.controllers.group_entries, self._group_id,
                                   chains_of=self.session.chains, current_chain=self._chain_id)
        self.lbl_empty.setVisible(not groups)
        self.b_delgroup.setEnabled(bool(groups))
        on = [g for g in groups if g.enabled and self.session.controllers.group_entries(g.id)]
        self.b_calc.setEnabled(bool(on))
        self.b_bake.setEnabled(bool(on))
        self._update_pending()

    # ---- 「まだ反映されていない変更」の表示 ------------------------------------
    def _state_signature(self):
        c = self.session.controllers
        cols = []
        for node in getattr(self, "_collider_nodes", []):
            try:
                cols.append([cmds.getAttr(node + "." + a) for a in ("t", "r", "radius")] +
                            [cmds.getAttr(node + ".length") if cmds.attributeQuery("length", node=node, exists=True)
                             else 0.0])
            except (RuntimeError, ValueError):
                cols.append(None)
        # 「リグで見る」 on but the rig not driven yet: the button shows it is pending.
        # The session's own flag, NOT a scene search: this runs every 400 ms and
        # searching the scene for the preview layer froze playback for a moment
        # in a big scene (27 ms with 1700 nodes, 56 ms with 20000 more).
        rig_waiting = bool(self.session.rig_preview and self.session.rig_preview_waiting)
        return json.dumps([[g.to_dict() for g in c.groups],
                           [(e.id, e.group, e.rest) for e in c.entries],
                           self.session.bake_options.to_dict(), cols, rig_waiting], sort_keys=True, default=str)

    def _mark_calculated(self):
        self._calculated_sig = self._state_signature()
        self._update_pending()

    def _update_pending(self):
        if cmds.play(query=True, state=True):
            return          # never take time from playback
        groups = self.session.controllers.groups
        on = [g for g in groups if g.enabled and self.session.controllers.group_entries(g.id)]
        if not on:
            self.lbl_targets.setText("対象：なし（揺れものにチェックを入れてください）")
            self.b_calc.setText(CALC_TEXT)
            return
        self.lbl_targets.setText("対象：" + "、".join(g.name for g in on))
        pending = self._calculated_sig is not None and self._calculated_sig != self._state_signature()
        self.b_calc.setText(CALC_PENDING_TEXT if pending else CALC_TEXT)
        self.b_calc.setToolTip("値が変わっています。プレビューはまだ前の結果です。押すと反映します" if pending else
                               "チェックを入れた揺れものを計算し、プレビュー用のプロキシ（「リグで見る」がオンならリグも）を更新します")

    def refresh_group_panel(self):
        g = self.current_group()
        chain = self.current_chain() if g is not None else None
        self.sec_group.setVisible(g is not None and chain is None)
        self.sec_chain.setVisible(chain is not None)
        self.b_restore.setEnabled(g is not None)
        if g is None:
            return
        if chain is not None:
            self.refresh_chain_panel(g, chain)
            return
        self.sec_group.header.setText("「%s」の設定" % g.name)
        s = g.settings
        for w, v in ((self.rb_chain, g.chain and not g.rope), (self.rb_single, not g.chain and not g.rope),
                     (self.rb_rope, g.rope), (self.chk_chain_auto, g.chain_auto),
                     (self.chk_gravity, s.gravity_enabled)):
            w.blockSignals(True)
            w.setChecked(v)
            w.blockSignals(False)
        self.sl_stiffness.setValue(s.stiffness)
        self.curve_edit.setPoints(s.stiffness_curve)
        self._sync_curve_spin()
        self.sl_damping.setValue(s.damping)
        self.sl_mass.setValue(s.mass)
        self.sl_follow.setValue(s.follow)
        self.sl_air.setValue(s.air_drag)
        self.sl_variation.setValue(s.variation)
        self.sl_link.setValue(s.chain_link)
        self.sl_side.setValue(s.side_damp)
        self.chk_side_down.blockSignals(True)
        self.chk_side_down.setChecked(bool(s.side_down))
        self.chk_side_down.blockSignals(False)
        self.sl_shape.setValue(s.shape_keep)
        self.sl_delay.setValue(s.delay)
        self.sl_stretch.setValue(s.rope_stretch)
        self.sp_rope_len.blockSignals(True)
        self.sp_rope_len.setValue(cm_to_ui(s.rope_length))
        self.sp_rope_len.blockSignals(False)
        self.chk_rope_spin.blockSignals(True)
        self.chk_rope_spin.setChecked(bool(s.rope_spin))
        self.chk_rope_spin.blockSignals(False)
        self.chk_rope_self.blockSignals(True)
        self.chk_rope_self.setChecked(bool(s.rope_self_collision))
        self.chk_rope_self.blockSignals(False)
        self.lbl_rope_len_unit.setText(linear_unit_name())
        self.sp_seed.blockSignals(True)
        self.sp_seed.setValue(int(s.variation_seed))
        self.sp_seed.blockSignals(False)
        self.sl_strength.setValue(s.strength)
        self.sl_gravity.setValue(s.gravity)
        self.sp_length.blockSignals(True)
        self.sp_length.setValue(cm_to_ui(s.pendulum_length))
        self.sp_length.blockSignals(False)
        self.lbl_unit.setText(linear_unit_name())
        for combo, value in ((self.cb_aim, s.aim_axis),
                             (self.cb_start, s.start_state), (self.cb_initvel, s.initial_velocity)):
            combo.blockSignals(True)
            combo.setCurrentIndex(max(0, combo.findData(value)))
            combo.blockSignals(False)
        for chk, v in ((self.chk_tr, s.translate), (self.chk_rot, s.rotate)):
            chk.blockSignals(True)
            chk.setChecked(any(v))
            chk.blockSignals(False)
        self.chk_collision.blockSignals(True)
        self.chk_collision.setChecked(s.collision)
        self.chk_collision.blockSignals(False)
        self.sp_thick.blockSignals(True)
        self.sp_thick.setValue(cm_to_ui(s.collision_thickness))
        self.sp_thick.blockSignals(False)
        self.lbl_thick_unit.setText(linear_unit_name())
        self._refresh_push_label()
        self.refresh_push_result()
        self.chk_warp.blockSignals(True)
        self.chk_warp.setChecked(s.warp_enabled)
        self.chk_warp.blockSignals(False)
        for sp, v in ((self.sp_warp_dist, cm_to_ui(s.warp_distance)), (self.sp_warp_angle, s.warp_angle),
                      (self.sp_warp_frames, int(s.warp_frames)), (self.sp_warp_screen, s.warp_screen)):
            sp.blockSignals(True)
            sp.setValue(v)
            sp.blockSignals(False)
        self.sl_warp_strength.setValue(s.warp_strength)
        self.lbl_warp_unit.setText(linear_unit_name())
        self.le_warp_add.setText(s.warp_add)
        self.le_warp_ignore.setText(s.warp_ignore)
        self._fill_warp_cameras()
        self.refresh_warp_result()
        self._sync_enabled()
        self.refresh_controllers()

    def refresh_controllers(self):
        g = self.current_group()
        entries = self.session.controllers.group_entries(g.id) if g else []
        self.ctl_list.set_entries(entries, self.session.controllers.status, rope=bool(g and g.rope))
        # the curve runs along each chain: markers for the longest chain
        n = len(entries)
        if entries:
            try:
                from ..core.simulation_manager import SimulationManager
                n = max(len(c) for c in SimulationManager.split_chains(entries, {e.id: e.path for e in entries}))
            except Exception:       # order problems are reported when calculating
                pass
        self.curve_edit.setMarkers(n)

    def refresh_range(self):
        b = self.session.bake_options
        for sp, v in ((self.sp_start, b.start), (self.sp_end, b.end), (self.sp_preroll, b.preroll)):
            sp.blockSignals(True)
            sp.setValue(int(round(v)))
            sp.blockSignals(False)
        for combo, value in ((self.cb_mode, b.mode), (self.cb_preroll, b.preroll_mode)):
            combo.blockSignals(True)
            combo.setCurrentIndex(max(0, combo.findData(value)))
            combo.blockSignals(False)
        self.le_layer.setText(b.layer_name)
        self.le_layer.setEnabled(b.mode != "overwrite")

    def refresh_presets(self):
        current = self.cb_preset.currentData()
        self.cb_preset.blockSignals(True)
        self.cb_preset.clear()
        try:
            names = self.session.presets.names()
        except Exception as exc:  # broken user file -> show built-ins only
            LOG.warning(str(exc))
            names = []
        for key, label, user in names:
            self.cb_preset.addItem(label + ("（ユーザー）" if user else ""), key)
        idx = self.cb_preset.findData(current)
        self.cb_preset.setCurrentIndex(idx if idx >= 0 else 0)
        self.cb_preset.blockSignals(False)

    def reload_from_scene(self):
        self._guard(self.session.load)
        self._group_id = None
        self._chain_id = None
        self._calculated_sig = None
        self.refresh_all()
        n = len(self.session.controllers.groups)
        self.set_status("シーンから SIMFLOW の設定を読み込みました（揺れもの %d 個）" % n if n else "準備完了")

    def set_status(self, text):
        self.status.setText(text)

    # ==================================================================
    # actions
    # ==================================================================
    def _guard(self, fn, success=None, progress=None):
        """Run ``fn`` and turn errors into dialogs.  Returns fn's result.

        ``progress`` is closed BEFORE any error dialog is shown, so the
        progress dialog never stays on screen behind the warning."""
        try:
            result = fn()
        except BaseException:
            if progress is not None:
                progress.close()
            return self._report_error()
        if progress is not None:
            progress.close()
        if success:
            self.set_status(success)
        return result

    def _report_error(self):
        try:
            raise
        except SimulationCancelled:
            self.set_status("キャンセルしました。シーンは変更していません。")
            return None
        except (SimflowError, SolverError, PresetError) as exc:
            LOG.warning(str(exc))
            QtWidgets.QMessageBox.warning(self, "SIMFLOW", str(exc))
            self.set_status(str(exc).splitlines()[0])
            return None
        except NotImplementedError as exc:
            QtWidgets.QMessageBox.information(self, "SIMFLOW", str(exc))
            return None
        except Exception as exc:  # unexpected -> show and log full traceback
            LOG.error(traceback.format_exc())
            QtWidgets.QMessageBox.critical(self, "SIMFLOW - 予期しないエラー",
                                           "%s\n\n詳細はスクリプトエディタを確認してください。" % exc)
            self.set_status("エラー：%s" % exc)
            return None

    # ---- groups --------------------------------------------------------------
    def _on_group_selected(self):
        gid = self.group_list.current_id()
        chain = self.group_list.current_chain()
        if not gid or (gid == self._group_id and chain == self._chain_id):
            return
        self._group_id, self._chain_id = gid, chain
        c = self.current_chain()
        if c is not None:
            # the chain's controllers are selected in Maya (rig controllers)
            nodes = []
            for e in c:
                try:
                    nodes.append(self.session.controllers.resolve(e))
                except SimflowError:
                    pass
            if nodes:
                self._guard(lambda: cmds.select(nodes, replace=True))
            self.set_status("チェーン %s〜%s（%d 個）を選択しました。スライダーはこのチェーンだけの値です。"
                            % (c[0].label, c[-1].label, len(c)))
        self.refresh_group_panel()

    def _on_group_enabled(self, gid, on):
        self.session.set_group_enabled(gid, on)
        self.refresh_groups()

    def _new_group(self):
        def run():
            g, warnings = self.session.add_group()
            self._group_id = g.id
            n = len(self.session.controllers.group_entries(g.id))
            msg = "「%s」を作りました（コントローラー %d 個）。" % (g.name, n)
            if not n:
                msg += " コントローラーを選んで「選択を追加」を押してください。"
            if warnings:
                msg += " " + "; ".join(warnings)
            self.set_status(msg)
        self._guard(run)
        self.refresh_groups()
        self.refresh_group_panel()

    def _rename_group(self):
        g = self.current_group()
        if g is None:
            return
        name, ok = QtWidgets.QInputDialog.getText(self, "SIMFLOW - 名前変更", "揺れものの名前：", text=g.name)
        if not ok:
            return
        self._guard(lambda: self.session.rename_group(g.id, name), "名前を「%s」にしました。" % name.strip())
        self.refresh_groups()
        self.refresh_group_panel()

    def _delete_group(self):
        g = self.current_group()
        if g is None:
            return
        if QtWidgets.QMessageBox.question(self, "SIMFLOW", "「%s」を削除しますか？\n（シーンのノードは消えません）"
                                          % g.name) != QtWidgets.QMessageBox.Yes:
            return
        name = g.name
        self._guard(lambda: self.session.remove_group(g.id), "「%s」を削除しました。" % name)
        self.refresh_groups()
        self.refresh_group_panel()

    def _on_chain_toggled(self, on):
        g = self.current_group()
        if g is not None:
            self.session.set_chain(g.id, on)
            self._sync_enabled()
            self.refresh_groups()

    def _build_rope_length(self):
        """縄の長さ (rope mode only): 0 = the start frame's shape."""
        self.rope_len_row = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(self.rope_len_row)
        lay.setContentsMargins(0, 0, 0, 0)
        lbl = QtWidgets.QLabel("縄の長さ")
        lbl.setFixedWidth(80)
        lbl.setToolTip(HELP.TIPS["rope_length"])
        self.sp_rope_len = QtWidgets.QDoubleSpinBox()
        self.sp_rope_len.setRange(0.0, 1000000.0)
        self.sp_rope_len.setDecimals(3)
        self.sp_rope_len.setKeyboardTracking(False)
        self.sp_rope_len.setSpecialValueText("自動")
        self.sp_rope_len.setMinimumWidth(50)
        self.sp_rope_len.setToolTip(HELP.TIPS["rope_length"])
        self.sp_rope_len.valueChanged.connect(lambda v: self._set("rope_length", ui_to_cm(v)))
        self.lbl_rope_len_unit = W.muted_label("")
        self.b_rope_len = W.button("選択から", "スプライン IK のカーブ（またはルートのジョイント、IK ハンドル）を選んでから押すと、"
                                              "その長さを縄の長さにします。\nカーブは変形前の元の形の長さ、ジョイントは骨の長さの合計を使います")
        self.b_rope_len.clicked.connect(self._rope_length_from_selection)
        lay.addWidget(lbl)
        lay.addWidget(self.sp_rope_len, 1)
        lay.addWidget(self.lbl_rope_len_unit)
        lay.addWidget(self.b_rope_len)
        outer = QtWidgets.QWidget()
        col = QtWidgets.QVBoxLayout(outer)
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self.rope_len_row)
        self.chk_rope_spin = QtWidgets.QCheckBox("手の回転に合わせて回す（縄跳び）")
        self.chk_rope_spin.setToolTip(HELP.TIPS["rope_spin"])
        self.chk_rope_spin.toggled.connect(lambda on: self._set("rope_spin", bool(on)))
        col.addWidget(self.chk_rope_spin)
        self.chk_rope_self = QtWidgets.QCheckBox("縄どうしの当たり（絡まない）")
        self.chk_rope_self.setToolTip(HELP.TIPS["rope_self_collision"])
        self.chk_rope_self.toggled.connect(lambda on: self._set("rope_self_collision", bool(on)))
        col.addWidget(self.chk_rope_self)
        self.rope_len_row = outer           # shown / hidden together (rope only)
        return outer

    def _rope_length_from_selection(self):
        got = self._guard(lambda: self.session.measure_rope_length())
        if got:
            length, how = got
            self.sp_rope_len.setValue(length)
            self.set_status("縄の長さを %.4g %s にしました（%s）。" % (length, linear_unit_name(), how))

    def _on_rope_toggled(self, on):
        g = self.current_group()
        if g is not None:
            self.session.set_rope(g.id, on)
            self._sync_enabled()
            self.refresh_groups()
            self.refresh_controllers()

    def _on_chain_auto_toggled(self, on):
        g = self.current_group()
        if g is not None:
            self.session.set_chain_auto(g.id, on)
            self._sync_enabled()

    # ---- controllers -----------------------------------------------------------
    def _add_selected(self):
        g = self.current_group()
        if g is None:
            return

        def run():
            added, warnings = self.session.add_selected(g.id)
            msg = "「%s」にコントローラーを %d 個追加しました。" % (g.name, len(added))
            if warnings:
                msg += " " + "; ".join(warnings)
            self.set_status(msg)
        self._guard(run)
        self.refresh_groups()
        self.refresh_controllers()

    def _remove_selected(self):
        ids = self.ctl_list.selected_ids()
        if not ids:
            self.set_status("外すコントローラーをリストで選択してください。")
            return
        self._guard(lambda: self.session.remove(ids), "コントローラーを %d 個外しました。" % len(ids))
        self.refresh_groups()
        self.refresh_controllers()

    def _move(self, offset):
        ids = self.ctl_list.selected_ids()
        if not ids:
            self.set_status("移動するコントローラーをリストで選択してください。")
            return
        self._guard(lambda: self.session.move(ids, offset))
        self.refresh_controllers()
        for i in range(self.ctl_list.count()):
            item = self.ctl_list.item(i)
            item.setSelected(item.data(QtCore.Qt.UserRole + 1) in ids)

    def _select_in_maya(self, entry_id):
        e = self.session.controllers.get(entry_id)
        if e:
            self._guard(lambda: cmds.select(self.session.controllers.resolve(e), replace=True))

    # ---- presets -------------------------------------------------------------------
    def _apply_preset(self):
        g = self.current_group()
        key = self.cb_preset.currentData()
        if g is None or not key:
            return
        self._guard(lambda: self.session.apply_preset(key, g.id),
                    "プリセット「%s」を「%s」に適用しました。" % (self.cb_preset.currentText(), g.name))
        self.refresh_group_panel()

    def _save_preset(self):
        g = self.current_group()
        if g is None:
            return
        name, ok = QtWidgets.QInputDialog.getText(self, "SIMFLOW - プリセットを保存", "プリセット名：")
        if not ok:
            return
        saved = self._guard(lambda: self.session.save_preset(name, g.id))
        if saved:
            self.refresh_presets()
            self.cb_preset.setCurrentIndex(self.cb_preset.findData(saved))
            self.set_status("プリセット「%s」を保存しました。" % saved)

    def _delete_preset(self):
        key = self.cb_preset.currentData()
        if not key:
            return
        if QtWidgets.QMessageBox.question(self, "SIMFLOW", "プリセット「%s」を削除しますか？"
                                          % self.cb_preset.currentText()) != QtWidgets.QMessageBox.Yes:
            return

        def run():
            self.session.presets.delete(key)
            return True
        if self._guard(run):
            self.refresh_presets()
            self.set_status("プリセットを削除しました。")

    def _export_presets(self, keys=None, path=None):
        """書き出し: chosen user presets to a file of their own, to give to
        someone (they add it with 読み込み; their own presets stay).
        ``keys`` / ``path`` skip the dialogs (scripts / tests)."""
        pm = self.session.presets
        user = [(k, lbl) for k, lbl, is_user in self._guard(pm.names) or [] if is_user]
        if not user:
            QtWidgets.QMessageBox.information(self, "SIMFLOW", "書き出せるプリセットがありません。\n"
                                              "「今の値をプリセットとして保存」で保存したものを書き出せます。")
            return None
        if keys is None:
            keys = self._choose_presets(user)
            if not keys:
                return None
        if path is None:
            first = keys[0] if len(keys) == 1 else "simflow_presets"
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "SIMFLOW - プリセットを書き出し",
                os.path.join(os.path.expanduser("~"), first + ".json"), "JSON (*.json)")
            if not path:
                return None
        done = self._guard(lambda: pm.export(keys, path))
        if done:
            self.set_status("プリセット %d 個を書き出しました: %s" % (len(done), path))
        return done

    def _choose_presets(self, user):
        """A dialog with the user presets to tick (all ticked)."""
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("SIMFLOW - 書き出すプリセット")
        lay = QtWidgets.QVBoxLayout(dlg)
        lay.addWidget(QtWidgets.QLabel("書き出すプリセットにチェックを入れてください。"))
        lst = QtWidgets.QListWidget()
        for key, label in user:
            it = QtWidgets.QListWidgetItem(label)
            it.setData(QtCore.Qt.UserRole, key)
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Checked)
            lst.addItem(it)
        lay.addWidget(lst)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return []
        return [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count())
                if lst.item(i).checkState() == QtCore.Qt.Checked]

    def _import_presets(self, path=None):
        """読み込み: add a file's presets to this user's (nothing of theirs is
        removed).  Same names: overwrite or add as 「名前 (2)」 (asked)."""
        pm = self.session.presets
        if path is None:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "SIMFLOW - プリセットを読み込み", os.path.expanduser("~"), "JSON (*.json)")
            if not path:
                return None
        entries = self._guard(lambda: pm.read_file(path))
        if not entries:
            return None
        same = self._guard(lambda: pm.conflicts(entries)) or []
        overwrite = False
        if same:
            box = QtWidgets.QMessageBox(self)
            box.setWindowTitle("SIMFLOW")
            box.setIcon(QtWidgets.QMessageBox.Question)
            box.setText("同じ名前のプリセットがあります:\n「%s」\n\n上書きしますか？" % "」「".join(same))
            b_new = box.addButton("別の名前で追加", QtWidgets.QMessageBox.AcceptRole)
            b_over = box.addButton("上書き", QtWidgets.QMessageBox.DestructiveRole)
            box.addButton("やめる", QtWidgets.QMessageBox.RejectRole)
            box.setDefaultButton(b_new)
            box.exec()
            if box.clickedButton() is b_over:
                overwrite = True
            elif box.clickedButton() is not b_new:
                return None
        done = self._guard(lambda: pm.import_entries(entries, overwrite=overwrite))
        if done:
            self.refresh_presets()
            renamed = ["「%s」→「%s」" % (a, b) for a, b in done if a != b]
            self.set_status("プリセット %d 個を読み込みました。%s" % (
                len(done), ("名前を変えたもの: " + "、".join(renamed)) if renamed else ""))
        return done

    # ---- range ---------------------------------------------------------------------
    def _use_playback_range(self):
        start, end = playback_range()
        self.session.bake_options.start, self.session.bake_options.end = start, end
        self.refresh_range()

    def _on_mode_changed(self, _):
        mode = self.cb_mode.currentData()
        self.session.bake_options.mode = mode
        self.le_layer.setEnabled(mode != "overwrite")

    def _sync_from_fields(self):
        # QLineEdit only commits on editingFinished; make sure it is applied
        self.session.bake_options.layer_name = self.le_layer.text().strip() or "SIMFLOW_Bake"

    # ---- calculate / bake / restore -----------------------------------------------
    def _calculate(self):
        self._sync_from_fields()
        progress = QtProgress(self, "シミュレーションを計算中...")
        try:
            results = self._guard(lambda: self.session.calculate(progress), progress=progress)
        finally:
            progress.close()
        self.refresh_colliders()
        if results:
            self._mark_calculated()
            if self.session.rig_preview:
                self._report_rig_preview(len(results))
            else:
                self.set_status("%d 個のコントローラーを計算しました（フレーム %g〜%g）。タイムラインをスクラブして確認してください。"
                                % (len(results), self.session.bake_options.start, self.session.bake_options.end))
        self.refresh_groups()
        self.refresh_controllers()
        self.refresh_warp_result()
        self.refresh_push_result()

    def _bake(self):
        self._sync_from_fields()
        if self.session.bake_options.mode == "overwrite":
            ans = QtWidgets.QMessageBox.question(
                self, "SIMFLOW - 上書き",
                "チェックを入れた揺れものの、元のアニメーションカーブを上書きしますか？\n\n"
                "元のカーブはバックアップされ、「元に戻す」で復元できます。")
            if ans != QtWidgets.QMessageBox.Yes:
                return
        progress = QtProgress(self, "ベイク中...")
        try:
            report = self._guard(lambda: self.session.bake(progress), progress=progress)
        finally:
            progress.close()
        self.chk_rig.blockSignals(True)
        self.chk_rig.setChecked(self.session.rig_preview)
        self.chk_rig.blockSignals(False)
        self.refresh_groups()
        self.refresh_controllers()
        if report:
            self._mark_calculated()
            self.set_status("ベイクと検証が完了しました（%s）。" % MODE_NAMES.get(report.mode, report.mode))
            QtWidgets.QMessageBox.information(self, "SIMFLOW - ベイク", report.summary())

    def _restore(self):
        g = self.current_group()
        if g is None:
            return
        ids = [e.id for e in self.session.controllers.group_entries(g.id)]
        restored = self._guard(lambda: self.session.restore_original(ids))
        if restored is not None:
            self.set_status("元のアニメーションに戻しました：%s" % ("、".join(restored) or "戻すものはありません"))
        self.refresh_groups()
        self.refresh_controllers()

    def _restore_all(self):
        restored = self._guard(lambda: self.session.restore_original(None))
        if restored is not None:
            self.set_status("元のアニメーションに戻しました：%s" % ("、".join(restored) or "戻すものはありません"))
        self.refresh_groups()
        self.refresh_controllers()

    def _toggle_rig_preview(self, on):
        if not on:
            self._guard(lambda: self.session.set_rig_preview(False), "リグでのプレビューを外しました（元のアニメーションに戻っています）。")
            self._update_pending()
            return
        # only the choice: the rig is driven by the next 「計算 / プレビュー」
        self.session.want_rig_preview()
        self.set_status("「リグで見る」をオンにしました。「計算 / プレビュー」を押すと、リグに反映します。")
        self._update_pending()

    def _report_rig_preview(self, count):
        skipped = self.session.rig_preview_skipped
        if skipped:
            self.set_status("リグに反映しました。次のチャンネルは動かせないため、プロキシだけで確認してください：%s"
                            % " / ".join(skipped[:3]) + (" ほか %d 件" % (len(skipped) - 3) if len(skipped) > 3 else ""))
        else:
            self.set_status("%d 個のコントローラーをリグで動かしています（フレーム %g〜%g、まだベイクはしていません）。"
                            % (count, self.session.bake_options.start, self.session.bake_options.end))

    def _reset(self):
        self._guard(self.session.reset_simulation, "計算結果とプレビュー（プロキシとリグへの反映）を削除しました。")
        self.chk_rig.blockSignals(True)
        self.chk_rig.setChecked(False)
        self.chk_rig.blockSignals(False)
        self.session.rig_preview = False
        self.session.rig_preview_waiting = False
        self._calculated_sig = None
        self._update_pending()

    def _reset_values(self):
        g = self.current_group()
        if g is None:
            return
        if QtWidgets.QMessageBox.question(
                self, "SIMFLOW", "「%s」の値をすべて初期値に戻しますか？\n（詳細の項目も含みます。コントローラーと揺れ方はそのままです）"
                % g.name) != QtWidgets.QMessageBox.Yes:
            return
        self._guard(lambda: self.session.reset_values(g.id), "「%s」の値を初期値に戻しました。" % g.name)
        self.refresh_group_panel()

    def _cleanup(self):
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("SIMFLOW - クリーンアップ")
        box.setText("SIMFLOW のプロキシと補助ノードを削除しますか？\n\n"
                    "削除するのは SIMFLOW が作成したノードだけです。")
        chk = QtWidgets.QCheckBox("SIMFLOW のベイクレイヤーとバックアップも削除する")
        box.setCheckBox(chk)
        box.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel)
        if box.exec() != QtWidgets.QMessageBox.Yes:
            return
        count = self._guard(lambda: self.session.cleanup(include_layers=chk.isChecked()))
        if count is not None:
            self.set_status("%d 個のノードを削除しました。" % count)
        self.refresh_all()

    def _edit_step_rate(self):
        g = self.current_group()
        if g is None:
            return
        value, ok = QtWidgets.QInputDialog.getDouble(
            self, "SIMFLOW", "「%s」の内部ステップレートの上限（Hz）：" % g.name, g.settings.substep_rate,
            24.0, 2000.0, 0)
        if ok:
            g.settings.substep_rate = value

    def _show_help(self):
        QtWidgets.QMessageBox.information(self, "SIMFLOW - 値の説明", HELP.HELP)

    def _versions(self):
        """The installer's 「SIMFLOW のインストール」 dialog (GitHub の版の一覧).
        Opened after this menu's slot returns, on Maya's main window: installing
        closes and reopens this panel."""
        import maya.utils
        maya.utils.executeDeferred(open_versions_dialog)

    def _about(self):
        QtWidgets.QMessageBox.about(
            self, "SIMFLOW",
            "SIMFLOW %s\n\n揺れもの（髪・尻尾・アクセサリーなど）ごとに、スプリング / 重力 / 振り子の"
            "シミュレーションを設定し、プロキシでプレビューして、検証付きでアニメーションレイヤーにベイクします。"
            "\n\nnHair モード：Phase 3" % __version__)

    # ==================================================================
    # scene callbacks
    # ==================================================================
    def _install_script_jobs(self):
        # Maya may delete and re-create this widget (docking re-runs the
        # workspaceControl uiScript).  Jobs of a dead widget must never touch
        # it: they are killed on ``destroyed`` and also check validity.
        jobs = self._script_jobs

        def alive():
            if shiboken6.isValid(self):
                return True
            _kill_jobs(jobs)
            return False

        def scene_changed():
            if alive():
                QtCore.QTimer.singleShot(0, lambda: alive() and self.reload_from_scene())

        def undo_redo():
            # groups / controllers / bake state follow undo; unsaved slider
            # values of groups that still exist stay
            def run():
                self.session.reload_after_undo()
                self.chk_rig.blockSignals(True)
                self.chk_rig.setChecked(self.session.rig_preview)
                self.chk_rig.blockSignals(False)
                self.refresh_groups()
                self.refresh_group_panel()
                self.refresh_colliders()
            if alive():
                QtCore.QTimer.singleShot(0, lambda: alive() and self._guard(run))

        for event, fn in (("SceneOpened", scene_changed), ("NewSceneOpened", scene_changed),
                          ("Undo", undo_redo), ("Redo", undo_redo)):
            jobs.append(cmds.scriptJob(event=[event, fn]))
        self.destroyed.connect(lambda *_: _kill_jobs(jobs))

    def _kill_script_jobs(self):
        _kill_jobs(self._script_jobs)

    def dockCloseEventTriggered(self):
        self._kill_script_jobs()

    def closeEvent(self, event):
        self._kill_script_jobs()
        super().closeEvent(event)
