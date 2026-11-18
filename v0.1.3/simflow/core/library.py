# -*- coding: utf-8 -*-
"""ライブラリ: one 揺れもの's values and the colliders it needs, saved as an
item in a folder anyone can point at (a shared folder = the team's library).

On disk (the library folder is chosen by the user, sub folders = categories)::

    <library>/<folder>/.../<name>.sfl/
        item.json        the values and colliders (no namespaces)
        thumbnail.png    the viewport when it was saved (optional)
        sequence/        the viewport over a frame range, 0001.jpg ... (optional;
                         played when the mouse moves over the tile / in the details)

item.json::

    {"format": "SIMFLOW library item", "version": 1, "name", "comment",
     "created", "user", "scene",
     "groups": [{"name", "chain", "chain_auto", "rope", "settings": {...},
                 "controllers": [names], "chains": [[names], ...],
                 "chain_values": [{"root": name, "index": i, "values": {...}}],
                 "collide_with": [group names]}, ...],
     "group": the one group when there is exactly one (also read by older
              versions; items written before 2026-11-18 have only this),
     "colliders": [{"name", "kind", "attach", "radius", "length", "size",
                    "t", "r", "s"}],
     "sequence": {"start", "end", "step", "fps", "count"} (when there is one)}

Names are stored WITHOUT their namespace ("chr01:skirt_fk_ctl" ->
"skirt_fk_ctl"); loading puts the namespace of the selected controllers (or
the one typed) back in front, so one item works for every copy of a rig.
An item of ONE group: by default its controllers are found by their saved
names in the namespace, as for several groups below (target "saved"); or the
user chooses them (target "new": the selected controllers, "current": the
group chosen in the panel - as in studioLibrary).
An item of SEVERAL groups (a character's ponytail, fringe, ahoge ... in one
go): every group finds its controllers by their saved names in the namespace;
a group none of whose controllers are found is left out (reported), a group
of the scene that already has those controllers gets the values.

The file part of this module does not need Maya (tests run it in plain Python).
"""
from __future__ import annotations

import datetime
import getpass
import json
import os
import re
import shutil

FORMAT = "SIMFLOW library item"
VERSION = 1
EXT = ".sfl"
ITEM_FILE = "item.json"
THUMB_FILE = "thumbnail.png"
SEQ_DIR = "sequence"
SEQ_EXT = (".jpg", ".jpeg", ".png")
SEQ_MAX = 150          # at most this many images (every n-th frame for long ranges)
SEQ_SIZE = 200
PATH_VAR = "SIMFLOW_libraryPath"
_BAD = re.compile(r'[\\/:*?"<>|]')


class LibraryError(ValueError):
    """User facing library problem."""


# ---------------------------------------------------------------------------
# namespaces
# ---------------------------------------------------------------------------
def strip_ns(name):
    """'|grp|chr01:sub:ctl' -> 'ctl' (no path, no namespace)."""
    return (name or "").split("|")[-1].split(":")[-1]


def namespace_of(name):
    """'|grp|chr01:sub:ctl' -> 'chr01:sub' ('' without)."""
    short = (name or "").split("|")[-1]
    return short.rpartition(":")[0]


def with_ns(name, ns):
    return "%s:%s" % (ns.strip(":"), name) if ns and ns.strip(":") else name


# ---------------------------------------------------------------------------
# folders / items (no Maya)
# ---------------------------------------------------------------------------
def check_name(name):
    name = (name or "").strip()
    if not name:
        raise LibraryError("名前を入力してください。")
    if _BAD.search(name) or name in (".", ".."):
        raise LibraryError('名前に次の文字は使えません: \\ / : * ? " < > |')
    return name


def is_item(path):
    return path.lower().endswith(EXT) and os.path.isfile(os.path.join(path, ITEM_FILE))


def folders(root):
    """Sub folders of the library (relative paths, '' = the library itself),
    items excluded, sorted."""
    out = [""]
    if not root or not os.path.isdir(root):
        return out
    for cur, dirs, _files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if not d.lower().endswith(EXT) and not d.startswith("."))
        rel = os.path.relpath(cur, root)
        if rel != ".":
            out.append(rel.replace("\\", "/"))
    return out


def make_folder(root, rel, name):
    name = check_name(name)
    path = os.path.join(root, rel, name)
    if os.path.exists(path):
        raise LibraryError("同じ名前のフォルダがあります: %s" % name)
    os.makedirs(path)
    return (rel + "/" + name).strip("/")


def items(root, rel="", search="", recursive=True):
    """[(path, data)] of the items in folder ``rel`` (and below), whose name /
    comment / folder contains every word of ``search``; sorted by name."""
    base = os.path.join(root, rel) if root else ""
    if not base or not os.path.isdir(base):
        return []
    words = [w.lower() for w in (search or "").split() if w]
    out = []
    for cur, dirs, _files in os.walk(base):
        found = [d for d in dirs if is_item(os.path.join(cur, d))]
        dirs[:] = [d for d in dirs if d not in found and not d.startswith(".")] if recursive else []
        for d in found:
            path = os.path.join(cur, d)
            try:
                data = read_item(path)
            except LibraryError:
                continue
            text = " ".join([data.get("name", ""), data.get("comment", ""),
                             os.path.relpath(cur, root)]).lower()
            if all(w in text for w in words):
                out.append((path, data))
    return sorted(out, key=lambda x: x[1].get("name", "").lower())


def item_path(root, rel, name):
    return os.path.join(root, rel, check_name(name) + EXT)


def read_item(path):
    try:
        with open(os.path.join(path, ITEM_FILE), encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError) as exc:
        raise LibraryError("ライブラリの項目を読めません: %s (%s)" % (path, exc))
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise LibraryError("SIMFLOW のライブラリの項目ではありません: %s" % path)
    return data


def thumbnail(path):
    p = os.path.join(path, THUMB_FILE)
    return p if os.path.isfile(p) else None


def sequence(path):
    """The item's sequence images in order ([] when it has none)."""
    d = os.path.join(path, SEQ_DIR) if path else ""
    if not d or not os.path.isdir(d):
        return []
    return [os.path.join(d, f) for f in sorted(os.listdir(d)) if f.lower().endswith(SEQ_EXT)]


def sequence_frames(start, end, max_count=SEQ_MAX):
    """Frames to grab between ``start`` and ``end`` (whole frames, both ends
    included): every frame, or every n-th one so there are at most ``max_count``."""
    start, end = int(round(min(start, end))), int(round(max(start, end)))
    n = end - start + 1
    step = max(1, -(-n // max(1, max_count)))
    return list(range(start, end + 1, step)), step


def write_item(path, data, thumb=None, overwrite=False, seq=None):
    """Write ``data`` (and copy the image ``thumb`` and the images ``seq``, in
    order) as the item ``path``.  Written to a temporary folder first, then moved
    into place."""
    if os.path.exists(path) and not overwrite:
        raise LibraryError("同じ名前の項目があります: %s" % os.path.basename(path))
    parent = os.path.dirname(path)
    if not os.path.isdir(parent):
        os.makedirs(parent)
    tmp = path + ".tmp"
    if os.path.exists(tmp):
        shutil.rmtree(tmp)
    os.makedirs(tmp)
    data = dict(data, format=FORMAT, version=VERSION)
    with open(os.path.join(tmp, ITEM_FILE), "w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    if thumb and os.path.isfile(thumb):
        shutil.copyfile(thumb, os.path.join(tmp, THUMB_FILE))
    seq = [f for f in (seq or []) if os.path.isfile(f)]
    if seq:
        os.makedirs(os.path.join(tmp, SEQ_DIR))
        for i, f in enumerate(seq):
            ext = os.path.splitext(f)[1].lower() or ".jpg"
            shutil.copyfile(f, os.path.join(tmp, SEQ_DIR, "%04d%s" % (i + 1, ext)))
    if os.path.exists(path):
        shutil.rmtree(path)
    os.replace(tmp, path)
    return path


def set_pictures(path, thumb=None, seq=None, seq_info=None):
    """Only the pictures of the item ``path``: the still ``thumb`` and the
    sequence ``seq`` (``seq_info`` as from grab_sequence; no ``seq`` = the item
    has no sequence any more).  The values, colliders, name, comment and date
    are kept."""
    if not is_item(path):
        raise LibraryError("ライブラリの項目ではありません: %s" % path)
    data = read_item(path)
    seq = [f for f in (seq or []) if os.path.isfile(f)]
    if seq and seq_info:
        data["sequence"] = dict(seq_info)
    else:
        data.pop("sequence", None)
    keep = None
    if not (thumb and os.path.isfile(thumb)) and thumbnail(path):
        # no new still: keep the old one (copied out first: the folder is replaced)
        keep = path + ".thumb.png"
        shutil.copyfile(thumbnail(path), keep)
        thumb = keep
    try:
        return write_item(path, data, thumb, overwrite=True, seq=seq)
    finally:
        if keep and os.path.isfile(keep):
            os.remove(keep)


def rename_item(path, name):
    """Give the item ``path`` the name ``name`` (its folder and the name in
    item.json).  Returns the new path."""
    name = check_name(name)
    if not is_item(path):
        raise LibraryError("ライブラリの項目ではありません: %s" % path)
    new = os.path.join(os.path.dirname(path), name + EXT)
    same = os.path.normcase(os.path.abspath(new)) == os.path.normcase(os.path.abspath(path))
    if os.path.exists(new) and not same:
        raise LibraryError("同じ名前の項目があります: %s" % name)
    data = read_item(path)
    old_name = data.get("name", "")
    data["name"] = name
    _write_json(path, data)
    try:
        if same:
            if new != path:                 # only the case changes (Windows): through another name
                tmp = path + ".rename"
                os.rename(path, tmp)
                os.rename(tmp, new)
        else:
            os.rename(path, new)
    except OSError as exc:
        data["name"] = old_name
        try:
            _write_json(path, data)
        except OSError:
            pass
        raise LibraryError("名前を変えられません（ほかの人が開いているかもしれません）: %s" % exc)
    return new


def _write_json(path, data):
    with open(os.path.join(path, ITEM_FILE), "w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def delete_item(path):
    if not is_item(path):
        raise LibraryError("ライブラリの項目ではありません: %s" % path)
    shutil.rmtree(path)


def groups_of(data):
    """The groups an item holds (list; old items: their one "group")."""
    gs = data.get("groups")
    if gs:
        return [g for g in gs if g]
    return [data["group"]] if data.get("group") else []


def summary(data):
    """One line for the tile's tooltip / the details: what the item holds."""
    parts = []
    gs = groups_of(data)
    if len(gs) > 1:
        parts.append("値（揺れもの %d 個：%s）" % (len(gs), "・".join(g.get("name", "") for g in gs)))
    elif gs:
        g = gs[0]
        n = len(g.get("chains") or []) or 1
        parts.append("値（%s・%s %d 本・%d 個）" % (g.get("name", ""), "チェーン" if g.get("chain") else "コントローラー",
                                             n if g.get("chain") else len(g.get("controllers") or []),
                                             len(g.get("controllers") or [])))
    cols = data.get("colliders") or []
    if cols:
        parts.append("当たり判定 %d 個" % len(cols))
    return "、".join(parts) or "（空）"


def stamp():
    try:
        user = getpass.getuser()
    except Exception:  # pragma: no cover
        user = ""
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), user


# ---------------------------------------------------------------------------
# Maya: capture / apply
# ---------------------------------------------------------------------------
def _group_data(session, g):
    entries = session.controllers.group_entries(g.id)
    settings = g.settings.to_dict()
    settings["collide_with"] = []
    settings["warp_camera"] = strip_ns(settings.get("warp_camera", ""))
    chains = session.chains(g.id)
    index = {c[0].id: i for i, c in enumerate(chains)}
    names = {e.id: strip_ns(e.path) for e in entries}
    values = []
    for root_id, vals in sorted(g.chain_overrides.items(), key=lambda kv: index.get(kv[0], 1 << 30)):
        if vals and root_id in names:
            values.append({"root": names[root_id], "index": index.get(root_id, -1), "values": dict(vals)})
    others = {x.id: x.name for x in session.controllers.groups}
    return {
        "name": g.name, "chain": bool(g.chain), "chain_auto": bool(g.chain_auto), "rope": bool(g.rope),
        "settings": settings,
        "controllers": [names[e.id] for e in entries],
        "chains": [[names[e.id] for e in c] for c in chains],
        "chain_values": values,
        "collide_with": [others[i] for i in g.settings.collide_with if i in others],
    }


def capture(session, group_id=None, colliders=(), name="", comment=""):
    """The item data of the group(s) ``group_id`` (an id, a list of ids, or
    None = no values) and the collider nodes ``colliders``."""
    import maya.cmds as cmds
    from ..utils.maya_utils import get_string_attr
    data = {"name": check_name(name), "comment": comment or ""}
    data["created"], data["user"] = stamp()
    data["scene"] = os.path.basename(cmds.file(query=True, sceneName=True) or "")
    ids = [group_id] if isinstance(group_id, str) else [x for x in (group_id or []) if x]
    groups = []
    for gid in ids:
        g = session.controllers.get_group(gid)
        if g is None:
            raise LibraryError("揺れものが見つかりません。")
        groups.append(_group_data(session, g))
    data["groups"] = groups
    data["group"] = groups[0] if len(groups) == 1 else None
    cols = []
    for col in colliders:
        if not cmds.objExists(col):
            continue
        follow = (cmds.listRelatives(col, parent=True, fullPath=True) or [None])[0]
        attach = get_string_attr(follow, "simflowAttach") if follow else None
        kind = get_string_attr(col, "simflowKind") or "sphere"
        c = {"name": strip_ns(col), "kind": kind, "attach": strip_ns(attach) if attach else "",
             "t": list(cmds.getAttr(col + ".t")[0]), "r": list(cmds.getAttr(col + ".r")[0]),
             "s": list(cmds.getAttr(col + ".s")[0])}
        if kind == "floor":
            c["size"] = cmds.getAttr(col + ".size")
        else:
            c["radius"] = cmds.getAttr(col + ".radius")
        if kind == "capsule":
            c["length"] = cmds.getAttr(col + ".length")
        cols.append(c)
    data["colliders"] = cols
    return data


def guess_namespace(selection=None):
    """The namespace of the (first) selected node, '' when none."""
    import maya.cmds as cmds
    sel = selection if selection is not None else (cmds.ls(selection=True, long=True) or [])
    return namespace_of(sel[0]) if sel else ""


def _find(name, ns):
    """The one node ``name`` in namespace ``ns`` (long name) or None / 'many'."""
    import maya.cmds as cmds
    hits = cmds.ls(with_ns(name, ns), long=True) or []
    if not hits and not ns:
        hits = cmds.ls(name, long=True) or []
    if len(hits) > 1:
        return "many"
    return hits[0] if hits else None


def select_controllers(data, ns):
    """Select the item's controllers in namespace ``ns``.  Returns (found, missing)."""
    import maya.cmds as cmds
    found, missing = [], []
    for n in [c for g in groups_of(data) for c in g.get("controllers") or []]:
        node = _find(n, ns)
        (found if node and node != "many" else missing).append(node if node and node != "many" else with_ns(n, ns))
    if found:
        cmds.select(found, replace=True)
    return found, missing


def apply(session, data, values=True, colliders=True, target="new", group_id=None, ns="", find_fk=True):
    """Load an item.

    values: put the item's values on 揺れもの.  An item of one group: ``target``
    "saved" finds its saved controllers by name in namespace ``ns`` (as for
    several groups), "new" makes one from the selected controllers (as
    ＋ 新規), "current" changes ``group_id``.  An item of several groups: every
    group finds its controllers by name in namespace ``ns`` (see the module's
    doc; ``target`` is not used).
    colliders: create the item's colliders on the nodes of namespace ``ns``
    (one that is already there - same name on the same node - is kept).
    Everything is one undo step.  Returns (group or None, report lines); for
    several groups the first one made / changed."""
    import maya.cmds as cmds
    from . import collider_manager
    from ..utils.maya_utils import undo_chunk, SimflowError
    report = []
    g = None
    gds = groups_of(data) if values else []
    if values and not gds:
        report.append("この項目には値が入っていません。")
    with undo_chunk("SIMFLOW Library"):
        if len(gds) == 1 and target == "saved":
            g = _apply_groups(session, gds, ns, report)
            if g is None:
                raise SimflowError("保存したコントローラーが見つかりません（%s）。ネームスペースを確かめるか、"
                                   "「選んだコントローラーで作る」を使ってください。"
                                   % "、".join(with_ns(n, ns) for n in (gds[0].get("controllers") or [])[:3]))
        elif len(gds) == 1:
            gd = gds[0]
            if target == "new":
                if not (cmds.ls(selection=True) or []):
                    raise SimflowError("値を入れるコントローラーを選んでください（チェーンなら根元を選び、"
                                       "「FK を自動で検知」で下の階層を読みます）。")
                g, warnings = session.add_group(gd.get("name") or None, from_selection=True, find_fk=find_fk)
                report += warnings
            else:
                g = session.controllers.get_group(group_id)
                if g is None:
                    raise SimflowError("値を入れる揺れものを選んでください。")
            _put_values(session, g, gd, ns, {x.name: x for x in session.controllers.groups}, report)
            session.save()
        elif gds:
            g = _apply_groups(session, gds, ns, report)
        if colliders:
            made = kept = 0
            for c in data.get("colliders") or []:
                attach = None
                if c.get("attach"):
                    attach = _find(c["attach"], ns)
                    if attach == "many":
                        report.append("当たり「%s」: %s が複数あります（ネームスペースを指定してください）。"
                                      % (c["name"], with_ns(c["attach"], ns)))
                        continue
                    if attach is None:
                        report.append("当たり「%s」: 付けるノード %s が見つかりません。" % (c["name"], with_ns(c["attach"], ns)))
                        continue
                if existing_collider_on(c["name"], attach):
                    kept += 1
                    continue
                kind = c.get("kind", "sphere")
                col = collider_manager.create_collider(
                    kind, attach, name=c["name"] if not cmds.objExists(c["name"]) else None,
                    radius=c.get("size") if kind == "floor" else c.get("radius"), length=c.get("length"))
                for a in ("t", "r", "s"):
                    if c.get(a) is not None:
                        cmds.setAttr("%s.%s" % (col, a), *c[a])
                made += 1
            if made or kept:
                report.append("当たり判定を %d 個作りました%s。" % (made, "（同じものが %d 個あったので作っていません）" % kept if kept else ""))
            session.sim.invalidate()
    return g, report


def _put_values(session, g, gd, ns, by_name, report):
    """The item group ``gd``'s values on the session group ``g``.  ``by_name``:
    item group name -> session group, for 押し出す揺れもの."""
    from .settings import SimSettings, CHAIN_FIELDS
    g.chain = bool(gd.get("chain", True))
    g.chain_auto = bool(gd.get("chain_auto", True))
    g.rope = bool(gd.get("rope", False))
    s = SimSettings.from_dict(gd.get("settings") or {})
    s.collide_with = tuple(by_name[n].id for n in (gd.get("collide_with") or [])
                           if n in by_name and by_name[n].id != g.id)
    if s.warp_camera:
        cam = _find(s.warp_camera, ns)
        s.warp_camera = cam.split("|")[-1] if cam and cam != "many" else s.warp_camera
    g.settings = s
    # チェーンごとの値: by the root's name, else by the chain's place
    g.chain_overrides = {}
    chains = session.chains(g.id)
    root_of = {strip_ns(c[0].path): c[0].id for c in chains}
    for cv in gd.get("chain_values") or []:
        rid = root_of.get(cv.get("root"))
        if rid is None and 0 <= cv.get("index", -1) < len(chains) and len(chains) == len(gd.get("chains") or []):
            rid = chains[cv["index"]][0].id
        if rid is None:
            report.append("チェーンごとの値を入れるチェーンが見つかりません: %s" % cv.get("root"))
            continue
        g.chain_overrides[rid] = {k: float(v) for k, v in (cv.get("values") or {}).items() if k in CHAIN_FIELDS}
    want = len(gd.get("controllers") or [])
    have = len(session.controllers.group_entries(g.id))
    if want and have != want:
        report.append("「%s」のコントローラーの数が保存したときと違います（保存 %d 個、今 %d 個）。" % (g.name, want, have))
    session.sim.invalidate([e.id for e in session.controllers.group_entries(g.id)])
    report.append("値を「%s」に入れました。" % g.name)


def _apply_groups(session, gds, ns, report):
    """An item of several groups (see apply).  Returns the first group made / changed."""
    import maya.cmds as cmds
    made = {}          # item group name -> session group
    order = []
    for gd in gds:
        name = gd.get("name") or ""
        found, missing = [], []
        for n in gd.get("controllers") or []:
            node = _find(n, ns)
            if node and node != "many":
                found.append(node)
            else:
                missing.append(with_ns(n, ns) + ("（複数あります）" if node == "many" else ""))
        if not found:
            report.append("「%s」は作りませんでした：コントローラーが見つかりません（%s など）。"
                          % (name, "、".join(missing[:3])))
            continue
        if missing:
            report.append("「%s」：見つからないコントローラー %d 個（%s）。見つかった %d 個で作ります。"
                          % (name, len(missing), "、".join(missing[:5]) + (" ほか" if len(missing) > 5 else ""),
                             len(found)))
        # a group of the scene that already has these controllers gets the values
        want = set(cmds.ls(found, long=True) or found)
        best, overlap = None, 0
        for x in session.controllers.groups:
            nodes = set(session.controllers.resolve(e) for e in session.controllers.group_entries(x.id))
            n = len(want & nodes)
            if n > overlap and x.id not in [y.id for y in made.values()]:
                best, overlap = x, n
        if best is not None:
            g = best
            report.append("「%s」は、同じコントローラーの揺れもの「%s」に値を入れます。" % (name, g.name))
        else:
            cmds.select(found, replace=True)
            g, warnings = session.add_group(name or None, from_selection=True, find_fk=False)
            report += warnings
        made[name] = g
        order.append((g, gd))
    # values (after every group exists: 押し出す揺れもの refers to the others)
    by_name = {x.name: x for x in session.controllers.groups}
    by_name.update(made)
    for g, gd in order:
        _put_values(session, g, gd, ns, by_name, report)
    session.save()
    cmds.select(clear=True)
    return order[0][0] if order else None


def existing_collider_on(name, attach_long):
    """A collider of the scene named ``name`` (namespace ignored) that follows
    exactly ``attach_long`` (None = a world collider)."""
    from . import collider_manager
    from ..utils.maya_utils import get_string_attr
    import maya.cmds as cmds
    want = (cmds.ls(attach_long, long=True) or [attach_long])[0] if attach_long else None
    for col in collider_manager.colliders():
        if strip_ns(col) != name:
            continue
        follow = (cmds.listRelatives(col, parent=True, fullPath=True) or [None])[0]
        a = get_string_attr(follow, "simflowAttach") if follow else None
        a = (cmds.ls(a, long=True) or [a])[0] if a else None
        if a == want:
            return col
    return None


def grab_thumbnail(path, size=256):
    """The active viewport at the current frame as a square PNG (None when it cannot be taken)."""
    import maya.cmds as cmds
    try:
        frame = cmds.currentTime(query=True)
        base = os.path.splitext(path)[0]
        out = cmds.playblast(frame=[frame], format="image", compression="png", completeFilename=base + ".png",
                             widthHeight=(size, size), percent=100, viewer=False, showOrnaments=False,
                             offScreen=True, forceOverwrite=True, clearCache=True)
        p = out if out and os.path.isfile(out) else base + ".png"
        return p if os.path.isfile(p) else None
    except Exception:
        return None


def scene_fps():
    import maya.mel as mel
    try:
        return float(mel.eval("currentTimeUnitToFPS()"))
    except Exception:
        return 24.0


def grab_sequence(folder, start, end, size=SEQ_SIZE, max_count=SEQ_MAX, progress=None):
    """The active viewport from ``start`` to ``end`` as JPEGs in ``folder``
    (every n-th frame when the range is long, see :func:`sequence_frames`).
    Returns ``(files in order, info for item.json)``; ``([], None)`` when it
    cannot be taken.  The current frame is put back."""
    import maya.cmds as cmds
    frames, step = sequence_frames(start, end, max_count)
    if not os.path.isdir(folder):
        os.makedirs(folder)
    for f in os.listdir(folder):
        if f.lower().endswith(SEQ_EXT):
            os.remove(os.path.join(folder, f))
    now = cmds.currentTime(query=True)
    files = []
    try:
        for i, fr in enumerate(frames):
            if progress is not None and progress(i, len(frames)) is False:
                break
            base = os.path.join(folder, "f%05d" % i)
            out = cmds.playblast(frame=[fr], format="image", compression="jpg", quality=85,
                                 completeFilename=base + ".jpg", widthHeight=(size, size), percent=100,
                                 viewer=False, showOrnaments=False, offScreen=True, forceOverwrite=True,
                                 clearCache=True)
            p = out if out and os.path.isfile(out) else base + ".jpg"
            if os.path.isfile(p):
                files.append(p)
    except Exception:
        files = []
    finally:
        try:
            cmds.currentTime(now, edit=True, update=True)
        except Exception:
            pass
    if not files:
        return [], None
    return files, {"start": frames[0], "end": frames[len(files) - 1], "step": step,
                   "fps": scene_fps(), "count": len(files)}


# ---------------------------------------------------------------------------
# the library folder (remembered per user)
# ---------------------------------------------------------------------------
def default_path():
    import maya.cmds as cmds
    return os.path.join(cmds.internalVar(userAppDir=True), "simflow", "library")


def library_path():
    import maya.cmds as cmds
    try:
        if cmds.optionVar(exists=PATH_VAR):
            p = cmds.optionVar(query=PATH_VAR)
            if p:
                return p
    except Exception:
        pass
    return default_path()


def set_library_path(path):
    import maya.cmds as cmds
    cmds.optionVar(stringValue=(PATH_VAR, path or ""))
