# -*- coding: utf-8 -*-
"""Small Maya helpers shared by the core modules."""
from __future__ import annotations

import contextlib

import maya.cmds as cmds
import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma

from .logger import get_logger

LOG = get_logger("maya")
_CHUNK_COUNTER = 0


class SimflowError(RuntimeError):
    """A user-facing error: the operation was refused / aborted safely."""


# ---------------------------------------------------------------------------
# Nodes
# ---------------------------------------------------------------------------

def get_mobject(name):
    sel = om.MSelectionList()
    try:
        sel.add(name)
    except RuntimeError:
        raise SimflowError("Node not found: %s" % name)
    return sel.getDependNode(0)


def long_name(node):
    names = cmds.ls(node, long=True) or []
    if len(names) != 1:
        raise SimflowError("Node is missing or not unique: %s" % node)
    return names[0]


def short_name(node):
    return node.split("|")[-1]


def node_uuid(node):
    return (cmds.ls(node, uuid=True) or [""])[0]


def node_from_uuid(uuid):
    if not uuid:
        return None
    names = cmds.ls(uuid, long=True) or []
    return names[0] if len(names) == 1 else None


def add_string_attr(node, attr, value, lock=False):
    if not cmds.attributeQuery(attr, node=node, exists=True):
        cmds.addAttr(node, longName=attr, dataType="string")
    if cmds.getAttr("%s.%s" % (node, attr), lock=True):
        cmds.setAttr("%s.%s" % (node, attr), lock=False)
    cmds.setAttr("%s.%s" % (node, attr), value or "", type="string")
    if lock:
        cmds.setAttr("%s.%s" % (node, attr), lock=True)


def get_string_attr(node, attr, default=None):
    if not cmds.objExists(node) or not cmds.attributeQuery(attr, node=node, exists=True):
        return default
    return cmds.getAttr("%s.%s" % (node, attr))


# ---------------------------------------------------------------------------
# Units / time
# ---------------------------------------------------------------------------

def fps():
    return om.MTime(1.0, om.MTime.kSeconds).asUnits(om.MTime.uiUnit())


def cm_to_ui(value):
    return om.MDistance(value, om.MDistance.kCentimeters).asUnits(om.MDistance.uiUnit())


def ui_to_cm(value):
    return om.MDistance(value, om.MDistance.uiUnit()).asCentimeters()


def rad_to_ui(value):
    return om.MAngle(value, om.MAngle.kRadians).asUnits(om.MAngle.uiUnit())


def ui_to_rad(value):
    return om.MAngle(value, om.MAngle.uiUnit()).asRadians()


def linear_unit_name():
    return cmds.currentUnit(query=True, linear=True)


def playback_range():
    return (cmds.playbackOptions(query=True, minTime=True),
            cmds.playbackOptions(query=True, maxTime=True))


@contextlib.contextmanager
def dg_time(frame):
    """Make a DG context at ``frame`` current for om2 plug reads.

    At the CURRENT frame the normal context is used instead: a timed context
    at the current time can return values cached before animation layers
    were created / muted (seen with override layers on joints: every other
    frame was right), while normal evaluation is up to date."""
    if abs(oma.MAnimControl.currentTime().asUnits(om.MTime.uiUnit()) - frame) < 1.0e-9:
        yield
        return
    ctx = om.MDGContext(om.MTime(frame, om.MTime.uiUnit()))
    previous = ctx.makeCurrent()
    try:
        yield
    finally:
        previous.makeCurrent()


# ---------------------------------------------------------------------------
# Undo / refresh
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def undo_chunk(name="SIMFLOW", rollback=True):
    """Group everything in one undo step.  If an exception escapes and
    ``rollback`` is True, the chunk is undone so the scene returns to the
    state before the operation (spec 11)."""
    global _CHUNK_COUNTER
    _CHUNK_COUNTER += 1
    # A unique chunk name lets us verify that the top of the undo queue is
    # really *this* chunk before rolling back.  An empty chunk is not recorded
    # by Maya, and blindly calling undo() would revert the user's previous,
    # unrelated operation.
    chunk = "%s #%d" % (name, _CHUNK_COUNTER)
    undo_enabled = cmds.undoInfo(query=True, state=True)
    cmds.undoInfo(openChunk=True, chunkName=chunk)
    failed = False
    try:
        yield
    except BaseException:
        failed = True
        raise
    finally:
        cmds.undoInfo(closeChunk=True)
        if failed and rollback and undo_enabled:
            if cmds.undoInfo(query=True, undoName=True) == chunk:
                try:
                    cmds.undo()
                    LOG.warning("Operation failed - scene was rolled back (%s)", name)
                except Exception as exc:  # pragma: no cover - defensive
                    LOG.error("Rollback failed: %s", exc)
            else:
                LOG.info("Operation failed before changing the scene (%s)", name)


@contextlib.contextmanager
def suspend_refresh():
    if cmds.about(batch=True):
        yield
        return
    cmds.refresh(suspend=True)
    try:
        yield
    finally:
        cmds.refresh(suspend=False)


@contextlib.contextmanager
def keep_current_time():
    current = cmds.currentTime(query=True)
    try:
        yield
    finally:
        cmds.currentTime(current, update=True)

