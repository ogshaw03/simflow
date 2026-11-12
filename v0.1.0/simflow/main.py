# -*- coding: utf-8 -*-
"""Entry point: ``import simflow; simflow.show()``.

The window is hosted in a Maya workspaceControl so it can be docked and is
restored with the workspace.
"""
from __future__ import annotations

import maya.cmds as cmds

_WINDOW = None


def _control_name():
    from .ui.main_window import SimflowWindow
    return SimflowWindow.OBJECT_NAME + "WorkspaceControl"


def show():
    global _WINDOW
    from .ui.main_window import SimflowWindow
    ctrl = _control_name()
    if cmds.workspaceControl(ctrl, query=True, exists=True):
        cmds.workspaceControl(ctrl, edit=True, close=True)
        cmds.deleteUI(ctrl, control=True)
    _WINDOW = SimflowWindow()
    _WINDOW.show(dockable=True, area="right", floating=True, width=380, height=860,
                 uiScript="import simflow.main; simflow.main.restore()")
    return _WINDOW


def restore():
    """Called by Maya when restoring the workspace control."""
    global _WINDOW
    import maya.OpenMayaUI as omui
    from .ui.main_window import SimflowWindow
    restored_control = omui.MQtUtil.getCurrentParent()
    _WINDOW = SimflowWindow()
    mixin_ptr = omui.MQtUtil.findControl(_WINDOW.objectName())
    omui.MQtUtil.addWidgetToMayaLayout(int(mixin_ptr), int(restored_control))
    return _WINDOW
