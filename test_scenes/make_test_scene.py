# -*- coding: utf-8 -*-
"""SIMFLOW の検証用シーンを作るスクリプト。

Maya 2025 のスクリプトエディタ（Python タブ）に貼り付けて実行するか、次のように実行します。

    import sys; sys.path.insert(0, r"D:/OG_Tools/SIM")
    import make_test_scene; make_test_scene.build()

    # リグを Reference で読み込む場合（リグのファイルを保存する場所を指定）
    make_test_scene.build(reference_dir=r"D:/OG_Tools/SIM_scratch/ref")

注意：今開いているシーンは保存せずに閉じます（新規シーンから作ります）。

シーンの内容（cm、24fps、1〜96 フレーム）
  A. 単体     box_ctl（急発進・急停止）の子 antenna_ctl（上向き）
              → Phase 1：スプリング / 重力 / 振り子、ベイク、元に戻す
  B. 髪       head_ctl（前進・振り向き・停止）の子 hair01〜03_ctl（下向き、オフセットのグループが回転済み）
              → Phase 2：チェーン、自動の長さと向き、プリロール
  C. 耳飾り   head_ctl の子 earL_ctl / earR_ctl（親子関係のない 2 つ）
              → 個別モード、個別の設定、プリセット「アクセサリー」
  D. 尻尾     hip_ctl（上下に弾みながら前進して停止）の子 tail01〜05_ctl
              （ジョイント。jointOrient あり、tail03 だけ回転順序 zxy）
              → 長いチェーン、jointOrient・回転順序の扱い
  E. 袖       arm_ctl（腕を上げる・前後に振る・下ろして止まる）の下に、
              FK のコントローラー 3 段の骨が 3 本（sleeveA1〜3_ctl、sleeveB…、sleeveC…）
              → 1 つの揺れものに複数の骨（骨ごとに別のチェーン）、ばらつき
  F. カメラ合わせ  F_move_ctl（カメラ合わせ用の移動）の下の F_body_ctl（ゆっくり歩く）の子 pony01〜03_ctl（ポニーテール）
              と、ショットのカメラ shotCam_F。カメラ合わせの「嘘」を 1 フレームで入れてある：
                24F  キャラをカメラの方へ 80 cm ワープ（画面ではほとんど動かない）     → 吸収される
                40F  キャラの向きを 60° 変える（カメラの方へ向ける）                     → 吸収される
                56F  キャラとカメラを一緒に横へ 100 cm ワープ（カットの代わり）           → 吸収される
                72F  体を横へ 40 cm 速く動かす（画面ではっきり見える演技）               → 吸収されない（ふつうに揺れる）
              → カメラ合わせ（ワープ吸収）。オフのままだと 24F・40F・56F で大きく暴れる
  G. 揺れもの同士の当たり  G_body_ctl（後ろへ急に動いて止まる → 戻る）の子に、
              2 cm 間隔で垂れた硬いチェーン 3 本 G_curtainA1〜3_ctl / B / C（袖や布のつもり）と、
              その 1.5 cm 後ろ、チェーンとチェーンの隙間の真後ろに垂れる髪 G_hair1〜4_ctl
              → 髪の「押し出す揺れもの」にカーテンを選ぶと、髪がカーテンの手前で止まる
                （選ばないと突き抜ける。骨だけを当たりにすると隙間を抜ける）
  H. 縄跳び   H_handL_ctl / H_handR_ctl（腰の高さで 1 秒に 1.5 回、手首で小さく回す。72F から両手を広げる）に
              ポイントコンストレインした持ち手 H_gripL_ctl / H_gripR_ctl（手首と一緒に X 軸まわりに回転）と、
              縄のコントローラー H_rope1〜8_ctl（1〜4 は左の持ち手の子、5〜8 は右の持ち手の子。縄が回るキーは移動に）、
              床 H_floor（高さ 0）。持ち手が回ると子の縄が振り回される（利用者のリグと同じ構造）。
              縄の見た目は、持ち手と縄のコントローラーを通るカーブ（スプライン IK の代わり）
              → 揺れ方「縄・ひも（両端を持つ）」。H_gripL_ctl、H_rope1〜8_ctl、H_gripR_ctl の順に登録し、
                「＋ 床」で H_floor に床を付ける。キーの縄は床を 5 cm 突き抜けるが、計算すると床で止まる。
                72F 以降は両手の間が縄より長くなり、縄が伸びる
揺らすコントローラーはすべて、ローカル +X が先端を向いています（SIMFLOW の「先端の軸」の既定値 +X）。
見た目で確認しやすいように、+X 方向に伸びる円柱のジオメトリを子に付けています（ジオメトリはコントローラーではありません）。
個別モードの振り子の長さの目安：アンテナ 8、耳飾り 3（cm）。
"""
from maya import cmds

FPS = "film"  # 24fps
START, END = 1, 96

HAIR = ["hair01_ctl", "hair02_ctl", "hair03_ctl"]
EARS = ["earL_ctl", "earR_ctl"]
TAIL = ["tail01_ctl", "tail02_ctl", "tail03_ctl", "tail04_ctl", "tail05_ctl"]
# E. sleeve: 3 rows x 3 FK controllers (row = one chain)
SLEEVE_ROWS = [["sleeve%s%d_ctl" % (r, i) for i in range(1, 4)] for r in "ABC"]
SLEEVE = [c for row in SLEEVE_ROWS for c in row]
# F. camera cheats: ponytail chain + shot camera
PONY = ["pony01_ctl", "pony02_ctl", "pony03_ctl"]
CAMERA_F = "shotCam_F"
WARPS_F = {24: "absorbed", 40: "absorbed", 56: "absorbed", 72: "visible"}
# G. collision between swing groups: a stiff "curtain" of 3 chains 2 cm apart and
# a strand hanging just behind it, behind the GAP between two chains
G_CURTAIN_ROWS = [["G_curtain%s%d_ctl" % (r, i) for i in range(1, 4)] for r in "ABC"]
G_CURTAIN = [c for row in G_CURTAIN_ROWS for c in row]
G_HAIR = ["G_hair1_ctl", "G_hair2_ctl", "G_hair3_ctl", "G_hair4_ctl"]
G_CURTAIN_Z = 0.0          # the curtain plane (in G_body_ctl space); the strand hangs at z = -1.5
# H. jump rope: grips constrained to the hands, rope controllers side by side, a floor
H_GRIPS = ["H_gripL_ctl", "H_gripR_ctl"]
H_ROPE = ["H_rope%d_ctl" % i for i in range(1, 9)]
H_FLOOR = "H_floor"
H_AXIS_Y = 90.0            # the hands turn around this height
H_HAND_X = 25.0            # +-, until frame 72, then the hands spread to +-H_HAND_X_WIDE
H_HAND_X_WIDE = 130.0     # 260 cm apart: longer than the rope (~215 cm)
H_TURNS = 1.5              # per second
H_RADIUS = 95.0            # the keyed rope: 5 cm through the floor at the bottom


def h_hand(side, f):
    """Hand (= grip) position in H_jump_grp space at frame f (side -1 = L, +1 = R)."""
    import math
    a = 2.0 * math.pi * H_TURNS * (f - START) / 24.0
    spread = min(1.0, max(0.0, (f - 72) / 12.0))
    hx = H_HAND_X + (H_HAND_X_WIDE - H_HAND_X) * spread
    return (side * hx, H_AXIS_Y + 8.0 * math.cos(a), 8.0 * math.sin(a))


def h_rope(i, f):
    """The rope the animator means (H_rope<i+1>_ctl, H_jump_grp space): a loop turning
    around the hands.  It is keyed in the grip's space, so in the scene the grip's
    rotation swings it around (the keyed world position is NOT this)."""
    import math
    a = 2.0 * math.pi * H_TURNS * (f - START) / 24.0
    u = (i + 1) / float(len(H_ROPE) + 1)
    rad = 8.0 + (H_RADIUS - 8.0) * math.sin(math.pi * u)
    return (-H_HAND_X + 2 * H_HAND_X * u, H_AXIS_Y + rad * math.cos(a + math.pi), rad * math.sin(a + math.pi))


def h_grip_of(i):
    return 0 if i < len(H_ROPE) // 2 else 1


# ---------------------------------------------------------------------------
# リグ（アニメーションなし）
# ---------------------------------------------------------------------------
def _geo(name, parent, length, radius=0.3, axis="x"):
    """コントローラーのローカル軸の方向に伸びる円柱（見た目用）。"""
    axes = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}
    geo = cmds.polyCylinder(name=name, axis=axes[axis], height=length, radius=radius,
                            subdivisionsAxis=12, constructionHistory=False)[0]
    cmds.parent(geo, parent, relative=True)
    cmds.setAttr("%s.t%s" % (geo, axis), length * 0.5)
    return geo


def _ctl(name, parent, radius=1.2, normal=(1, 0, 0)):
    c = cmds.circle(name=name, normal=normal, radius=radius, constructionHistory=False)[0]
    if parent:
        cmds.parent(c, parent, relative=True)
    return c


def _box_ctl(name, size=3.0):
    s = size * 0.5
    pts = [(-s, -s, -s), (s, -s, -s), (s, -s, s), (-s, -s, s), (-s, -s, -s),
           (-s, s, -s), (s, s, -s), (s, -s, -s), (s, s, -s), (s, s, s), (s, -s, s),
           (s, s, s), (-s, s, s), (-s, -s, s), (-s, s, s), (-s, s, -s)]
    return cmds.curve(name=name, degree=1, point=pts)


def build_rig():
    """リグだけを作ります（キーなし）。最上位のグループ名を返します。"""
    top = cmds.group(empty=True, name="TEST_RIG")

    # A. 単体：箱の上にアンテナ（ローカル +Y 向き）
    grp_a = cmds.group(empty=True, name="A_single_grp", parent=top)
    cmds.setAttr(grp_a + ".t", -40, 0, 0)
    box = _box_ctl("box_ctl")
    cmds.parent(box, grp_a, relative=True)
    cmds.setAttr(box + ".ty", 1.5)
    _geo("box_geo", box, 3.0, radius=1.4, axis="y")
    cmds.setAttr("box_geo.ty", 0)
    # オフセットのグループを Z +90° 回して、コントローラーのローカル +X が上を向くようにする
    ant_off = cmds.group(empty=True, name="antenna_offset", parent=box)
    cmds.setAttr(ant_off + ".ty", 1.5)
    cmds.setAttr(ant_off + ".rz", 90)
    ant = _ctl("antenna_ctl", ant_off, radius=0.8)
    _geo("antenna_geo", ant, 8.0, radius=0.2)

    # B / C. 頭・髪・耳飾り
    grp_b = cmds.group(empty=True, name="B_head_grp", parent=top)
    head = _ctl("head_ctl", grp_b, radius=4.0, normal=(0, 1, 0))
    cmds.setAttr(head + ".ty", 20)
    head_geo = cmds.polySphere(name="head_geo", radius=3.0, constructionHistory=False)[0]
    cmds.parent(head_geo, head, relative=True)

    # 髪：オフセットのグループを Z -90° 回して、コントローラーのローカル +X が下を向くようにする
    parent = head
    for i, name in enumerate(HAIR):
        off = cmds.group(empty=True, name=name.replace("_ctl", "_offset"), parent=parent)
        if i == 0:
            cmds.setAttr(off + ".t", 0, -1.0, -3.2)
            cmds.setAttr(off + ".rz", -90)
        else:
            cmds.setAttr(off + ".tx", 5.0)
        c = _ctl(name, off, radius=1.0)
        _geo(name.replace("_ctl", "_geo"), c, 5.0, radius=0.35)
        parent = c

    # 耳飾り：左右にぶら下がる（オフセットを Z -90° 回して、ローカル +X が下向き）
    for name, x in zip(EARS, (3.3, -3.3)):
        off = cmds.group(empty=True, name=name.replace("_ctl", "_offset"), parent=head)
        cmds.setAttr(off + ".t", x, -1.0, 0)
        cmds.setAttr(off + ".rz", -90)
        c = _ctl(name, off, radius=0.6)
        _geo(name.replace("_ctl", "_geo"), c, 3.0, radius=0.15)

    # D. 尻尾：ジョイント（jointOrient あり）にカーブのシェイプを付けたコントローラー
    grp_d = cmds.group(empty=True, name="D_tail_grp", parent=top)
    cmds.setAttr(grp_d + ".t", 40, 0, 0)
    hip = _ctl("hip_ctl", grp_d, radius=3.0, normal=(0, 1, 0))
    cmds.setAttr(hip + ".ty", 12)
    hip_geo = cmds.polyCube(name="hip_geo", width=5, height=3, depth=6, constructionHistory=False)[0]
    cmds.parent(hip_geo, hip, relative=True)

    cmds.select(clear=True)
    joints = []
    for i, name in enumerate(TAIL):
        # 後ろ（-Z）へ、少しずつ下がりながら伸びる
        j = cmds.joint(name=name, position=(40, 11.5 - 0.6 * i, -3.0 - 4.0 * i))
        joints.append(j)
    cmds.joint(name="tail_end_jnt", position=(40, 11.5 - 0.6 * 5, -3.0 - 4.0 * 5))
    cmds.joint(joints[0], edit=True, orientJoint="xyz", secondaryAxisOrient="yup", children=True, zeroScaleOrient=True)
    cmds.setAttr("tail_end_jnt.jointOrient", 0, 0, 0)
    cmds.parent(joints[0], hip)
    cmds.setAttr(joints[2] + ".rotateOrder", 4)  # zxy（回転順序の扱いの確認用）
    for j in joints:
        shape_src = cmds.circle(normal=(1, 0, 0), radius=1.0, constructionHistory=False)[0]
        shape = cmds.listRelatives(shape_src, shapes=True)[0]
        cmds.parent(shape, j, relative=True, shape=True)
        cmds.rename(shape, j + "Shape")
        cmds.delete(shape_src)
        cmds.setAttr(j + ".radius", 0.5)
        _geo(j.replace("_ctl", "_geo"), j, 4.0, radius=0.6 - 0.08 * joints.index(j))

    # E. sleeve: rows of FK controllers hanging from an animated arm
    grp_e = cmds.group(empty=True, name="E_sleeve_grp", parent=top)
    cmds.setAttr(grp_e + ".t", 80, 0, 0)
    arm = _ctl("arm_ctl", grp_e, radius=3.0)
    cmds.setAttr(arm + ".ty", 22)
    arm_geo = _geo("arm_geo", arm, 16.0, radius=1.6)
    for row, (x, z) in zip(SLEEVE_ROWS, ((7.0, 0.0), (9.0, 0.0), (11.0, 0.0))):
        parent = arm
        for i, name in enumerate(row):
            off = cmds.group(empty=True, name=name.replace("_ctl", "_offset"), parent=parent)
            if i == 0:
                # rows side by side under the arm, 2 cm apart; local +X points down
                cmds.setAttr(off + ".t", x, -1.6, z)
                cmds.setAttr(off + ".rz", -90)
            else:
                cmds.setAttr(off + ".tx", 4.0)
            c = _ctl(name, off, radius=0.8)
            _geo(name.replace("_ctl", "_geo"), c, 4.0, radius=0.25)
            parent = c

    # F. camera cheats: a character with a ponytail, moved by F_move_ctl
    grp_f = cmds.group(empty=True, name="F_warp_grp", parent=top)
    cmds.setAttr(grp_f + ".t", 120, 0, 0)
    move = _ctl("F_move_ctl", grp_f, radius=6.0, normal=(0, 1, 0))
    body = _ctl("F_body_ctl", move, radius=4.0, normal=(0, 1, 0))
    cmds.setAttr(body + ".ty", 20)
    f_head = cmds.polySphere(name="F_head_geo", radius=3.0, constructionHistory=False)[0]
    cmds.parent(f_head, body, relative=True)
    nose = cmds.polyCone(name="F_nose_geo", radius=0.8, height=2.0, axis=(0, 0, 1), constructionHistory=False)[0]
    cmds.parent(nose, body, relative=True)
    cmds.setAttr(nose + ".tz", 3.5)
    parent = body
    for i, name in enumerate(PONY):
        off = cmds.group(empty=True, name=name.replace("_ctl", "_offset"), parent=parent)
        if i == 0:
            # back of the head (-Z), local +X points down
            cmds.setAttr(off + ".t", 0, 1.0, -3.2)
            cmds.setAttr(off + ".rz", -90)
        else:
            cmds.setAttr(off + ".tx", 5.0)
        c = _ctl(name, off, radius=1.0)
        _geo(name.replace("_ctl", "_geo"), c, 5.0, radius=0.45)
        parent = c
    cam = cmds.camera(focalLength=35.0)[0]
    cam = cmds.parent(cam, grp_f, relative=True)[0]
    cam = cmds.rename(cam, CAMERA_F)
    cmds.setAttr(cam + ".t", 10, 18, 250)

    # G. collision between swing groups
    grp_g = cmds.group(empty=True, name="G_push_grp", parent=top)
    cmds.setAttr(grp_g + ".t", 160, 0, 0)
    g_body = _ctl("G_body_ctl", grp_g, radius=6.0, normal=(0, 1, 0))
    # the curtain: 3 chains hanging from height 16, at x = 6 / 8 / 10, local +X down
    for row, x in zip(G_CURTAIN_ROWS, (6.0, 8.0, 10.0)):
        parent = g_body
        for i, name in enumerate(row):
            off = cmds.group(empty=True, name=name.replace("_ctl", "_offset"), parent=parent)
            if i == 0:
                cmds.setAttr(off + ".t", x, 16.0, G_CURTAIN_Z)
                cmds.setAttr(off + ".rz", -90)
            else:
                cmds.setAttr(off + ".tx", 4.0)
            c = _ctl(name, off, radius=0.6)
            _geo(name.replace("_ctl", "_geo"), c, 4.0, radius=0.4)
            parent = c
    # the strand: from above, 1.5 cm behind the bar, local +X down
    parent = g_body
    for i, name in enumerate(G_HAIR):
        off = cmds.group(empty=True, name=name.replace("_ctl", "_offset"), parent=parent)
        if i == 0:
            cmds.setAttr(off + ".t", 9.0, 22.0, -1.5)
            cmds.setAttr(off + ".rz", -90)
        else:
            cmds.setAttr(off + ".tx", 5.0)
        c = _ctl(name, off, radius=0.6)
        _geo(name.replace("_ctl", "_geo"), c, 5.0, radius=0.3)
        parent = c

    # H. jump rope
    grp_h = cmds.group(empty=True, name="H_jump_grp", parent=top)
    cmds.setAttr(grp_h + ".t", 260, 0, 0)
    h_body = _ctl("H_body_ctl", grp_h, radius=20.0, normal=(0, 1, 0))
    for side, x in (("L", -H_HAND_X), ("R", H_HAND_X)):
        hand = _ctl("H_hand%s_ctl" % side, h_body, radius=3.0)
        cmds.setAttr(hand + ".t", x, H_AXIS_Y, 0)
    rope_grp = cmds.group(empty=True, name="H_rope_grp", parent=grp_h)
    grips = []
    for side, name in zip((-1.0, 1.0), H_GRIPS):
        g = _ctl(name, rope_grp, radius=2.5)
        cmds.setAttr(g + ".t", *h_hand(side, START))
        grips.append(g)
    ctls = [grips[0]]
    # the rope controllers are children of the grips (half on each): turning a grip swings them
    for i, name in enumerate(H_ROPE):
        g = grips[h_grip_of(i)]
        c = _ctl(name, g, radius=1.5)
        gp = cmds.getAttr(g + ".t")[0]
        p = h_rope(i, START)
        cmds.setAttr(c + ".t", p[0] - gp[0], p[1] - gp[1], p[2] - gp[2])
        ctls.append(c)
    ctls.append(grips[1])
    for side, g in zip("LR", grips):
        cmds.pointConstraint("H_hand%s_ctl" % side, g)
    # the rope: a curve through the grips and the rope controllers (instead of a spline IK),
    # driven by their WORLD positions
    pts = [cmds.xform(c, query=True, worldSpace=True, rotatePivot=True) for c in ctls]
    crv = cmds.curve(name="H_rope_crv", degree=3, editPoint=pts)
    crv = cmds.parent(crv, grp_h)[0]
    cmds.setAttr(crv + ".inheritsTransform", 0)
    for a in ("t", "r", "s"):
        cmds.setAttr("%s.%s" % (crv, a), *(0, 0, 0) if a != "s" else (1, 1, 1))
    shape = cmds.listRelatives(crv, shapes=True, fullPath=True)[0]
    cmds.setAttr(shape + ".overrideEnabled", 1)
    cmds.setAttr(shape + ".overrideColor", 17)
    cmds.setAttr(shape + ".lineWidth", 3)
    # drive the CVs: CV 0 / 1 and n-2 / n-1 near the grips (edit points -> cvs: n + 2 cvs)
    n_cv = cmds.getAttr(shape + ".spans") + 3
    src = [ctls[0]] + ctls + [ctls[-1]]
    for k in range(n_cv):
        c = src[min(k, len(src) - 1)]
        dm = cmds.createNode("decomposeMatrix", name=c.split("|")[-1] + "_dm", skipSelect=True)
        cmds.connectAttr(c + ".worldMatrix[0]", dm + ".inputMatrix")
        cmds.connectAttr(dm + ".outputTranslate", "%s.controlPoints[%d]" % (shape, k), force=True)
    floor = cmds.polyPlane(name=H_FLOOR, width=200, height=200, subdivisionsX=4, subdivisionsY=4)[0]
    cmds.parent(floor, grp_h, relative=True)        # under the rope (the group is at x = 260)

    cmds.select(clear=True)
    return top


# ---------------------------------------------------------------------------
# アニメーション（Reference のときは名前空間付きのノードにキーを打つ）
# ---------------------------------------------------------------------------
def _keys(node, attr, pairs, tangent="auto"):
    for f, v in pairs:
        cmds.setKeyframe(node, attribute=attr, time=f, value=v,
                         inTangentType=tangent, outTangentType=tangent)


def _step_keys(node, attr, pairs):
    """Keys that jump in ONE frame (stepped), as a camera cheat."""
    for f, v in pairs:
        cmds.setKeyframe(node, attribute=attr, time=f, value=v, inTangentType="linear", outTangentType="step")


def animate(ns=""):
    n = lambda name: ns + name

    # A. 箱：急発進して止まる → 逆向きに急発進して止まる → 静止（揺れの収まり方を見る）
    _keys(n("box_ctl"), "translateX", [(1, 0), (8, 25), (40, 25), (44, 5), (96, 5)], "linear")
    _keys(n("box_ctl"), "translateY", [(1, 1.5), (60, 1.5), (64, 6), (68, 1.5), (96, 1.5)], "linear")

    # B. 頭：前進して止まる → 振り向く → 戻す → 上下にうなずく → 静止
    head = n("head_ctl")
    _keys(head, "translateZ", [(1, 0), (12, 30), (24, 30)])
    _keys(head, "rotateY", [(24, 0), (32, 70), (44, 70), (52, -20), (58, 0)])
    _keys(head, "rotateX", [(58, 0), (62, 25), (66, -10), (70, 0)])
    _keys(head, "translateY", [(1, 20), (96, 20)])

    # D. 腰：上下に弾みながら前進して、フレーム 64 で止まる
    hip = n("hip_ctl")
    _keys(hip, "translateZ", [(1, 0), (64, 60), (96, 60)], "linear")
    bounce = []
    for f in range(1, 65, 6):
        bounce.append((f, 12.0 if (f // 6) % 2 == 0 else 14.0))
    bounce.append((70, 12.0))
    _keys(hip, "translateY", bounce)
    _keys(hip, "rotateY", [(1, 0), (20, 0), (28, 30), (40, -30), (52, 0)])

    # E. arm: raise the arm, swing it forward / back, drop it and stop
    arm = n("arm_ctl")
    _keys(arm, "rotateZ", [(1, 0), (14, 45), (30, 45), (40, -10), (48, 0), (96, 0)])
    _keys(arm, "rotateY", [(30, 0), (38, 40), (46, -30), (54, 0), (96, 0)])

    # F. a slow walk with a little bounce, then the camera cheats (1 frame each)
    body = n("F_body_ctl")
    walk = 20.0 / 95.0          # cm per frame
    _keys(body, "translateX", [(1, 0), (71, 70 * walk), (72, 71 * walk + 40.0), (96, 95 * walk + 40.0)], "linear")
    cmds.keyTangent(body, attribute="translateX", time=(71, 71), outTangentType="step")
    _keys(body, "translateY", [(f, 20.0 if (f // 4) % 2 == 0 else 20.6) for f in range(1, 97, 4)])
    _keys(body, "rotateY", [(1, 0), (8, 25), (16, -15), (22, 0)])
    move = n("F_move_ctl")
    _step_keys(move, "translateZ", [(1, 0), (23, 0), (24, 80), (96, 80)])
    _step_keys(move, "rotateY", [(1, 0), (39, 0), (40, 60), (96, 60)])
    _step_keys(move, "translateX", [(1, 0), (55, 0), (56, 100), (96, 100)])
    cam = n(CAMERA_F)
    _step_keys(cam, "translateX", [(1, 10), (55, 10), (56, 110), (96, 110)])

    # G. the body moves back fast (the strand lags forward, into the bar), stops, comes back
    _keys(n("G_body_ctl"), "translateZ", [(1, 0), (8, 0), (14, -20), (40, -20), (46, 0), (96, 0)], "linear")

    # H. jump rope: the wrists turn small circles and twist the grips with them; the animator
    # turns the rope with the rope controllers' translate keys (in the grip's space, so the
    # grip's twist swings the keyed rope around); from frame 72 the hands spread wider than the rope
    for f in range(START, END + 1, 2):
        for side, sx in (("L", -1.0), ("R", 1.0)):
            hand = n("H_hand%s_ctl" % side)
            p = h_hand(sx, f)
            for attr, v in zip(("translateX", "translateY", "translateZ"), p):
                cmds.setKeyframe(hand, attribute=attr, time=f, value=v)
            cmds.setKeyframe(n("H_grip%s_ctl" % side), attribute="rotateX", time=f,
                             value=360.0 * H_TURNS * (f - START) / 24.0)
        for i, name in enumerate(H_ROPE):
            gp = h_hand(-1.0 if h_grip_of(i) == 0 else 1.0, f)
            p = h_rope(i, f)
            for attr, v in zip(("translateX", "translateY", "translateZ"), (p[0] - gp[0], p[1] - gp[1], p[2] - gp[2])):
                cmds.setKeyframe(n(name), attribute=attr, time=f, value=v)


def _setup_scene():
    cmds.file(new=True, force=True)
    cmds.currentUnit(linear="cm", angle="deg", time=FPS)
    cmds.playbackOptions(minTime=START, maxTime=END, animationStartTime=START, animationEndTime=END)


def _frame_camera():
    cmds.setAttr("persp.t", 70, 60, 260)
    cmds.setAttr("persp.r", -12, 8, 0)
    cmds.currentTime(START)


def build(reference_dir=None, namespace="chr"):
    """検証用のシーンを作ります。

    reference_dir を指定すると、リグを <reference_dir>/simflow_test_rig.ma に保存し、
    新規シーンに名前空間 namespace で Reference として読み込んでから、アニメーションを付けます。
    """
    import os
    _setup_scene()
    ns = ""
    if reference_dir:
        if not os.path.isdir(reference_dir):
            os.makedirs(reference_dir)
        path = os.path.join(reference_dir, "simflow_test_rig.ma").replace("\\", "/")
        build_rig()
        cmds.file(rename=path)
        cmds.file(save=True, type="mayaAscii", force=True)
        _setup_scene()
        cmds.file(path, reference=True, namespace=namespace)
        ns = namespace + ":"
    else:
        build_rig()
    animate(ns)
    _frame_camera()
    cmds.select(clear=True)

    print("SIMFLOW 検証シーンを作りました（%d〜%d フレーム）。" % (START, END))
    print("  A 単体   : %santenna_ctl" % ns)
    print("  B 髪     : %s（チェーン、上から順に選択）" % ", ".join(ns + x for x in HAIR))
    print("  C 耳飾り : %s（個別）" % ", ".join(ns + x for x in EARS))
    print("  D 尻尾   : %s（チェーン）" % ", ".join(ns + x for x in TAIL))
    print("  E 袖     : %s（3 本 x 3 段。9 個を全部選んで 1 つの揺れもの、つながって揺れる）"
          % ", ".join(ns + x for x in SLEEVE))
    print("  F カメラ合わせ: %s（チェーン）。「カメラ合わせ（ワープ吸収）」をオン、カメラ %s"
          % (", ".join(ns + x for x in PONY), ns + CAMERA_F))
    print("                 24F・40F・56F が吸収され、72F は「画面で見えている動き」になれば正しい")
    print("  G 当たり : カーテン %s（9 個、チェーン 3 本）と髪 %s をそれぞれ別の揺れものに。"
          "髪の「押し出す揺れもの」でカーテンを選ぶ" % (", ".join(ns + x for x in G_CURTAIN),
                                                 ", ".join(ns + x for x in G_HAIR)))
    print("  H 縄跳び : %s の順に選んで、揺れ方「縄・ひも（両端を持つ）」。%s を選んで「＋ 床」"
          % (", ".join(ns + x for x in [H_GRIPS[0]] + H_ROPE + [H_GRIPS[1]]), ns + H_FLOOR))
    return ns


if __name__ == "__main__":
    build()
