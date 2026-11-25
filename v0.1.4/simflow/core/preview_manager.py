# -*- coding: utf-8 -*-
"""Preview handling (spec 7).

Phase 1 previews through the Output Proxy only; the controllers themselves
are never driven during preview (spec 6: direct preview is only allowed once
it is proven not to conflict with the original animation input).
"""
from __future__ import annotations

import maya.cmds as cmds

from ..utils.maya_utils import undo_chunk
from . import proxy_manager


class PreviewManager(object):

    def update(self, entries, results, progress=None):
        with undo_chunk("SIMFLOW Preview", rollback=True):
            # the old proxies of all these entries in ONE scan of the scene
            proxy_manager.delete_proxies(entry_ids=[e.id for e in entries if results.get(e.id) is not None])
            for k, entry in enumerate(entries):
                if progress is not None:
                    progress.update(float(k) / max(1, len(entries)), "%s（%d / %d）" % (entry.label, k + 1, len(entries)))
                result = results.get(entry.id)
                if result is not None:
                    proxy_manager.build_proxies(entry, result, delete=False)
            # the result is always shown (the panel has no switch for it any more)
            proxy_manager.set_group_visibility("SIMFLOW_OUTPUT", True)
        # re-evaluate the current frame so the proxies jump to place
        cmds.currentTime(cmds.currentTime(query=True), update=True)

    def clear(self, entry_ids=None):
        with undo_chunk("SIMFLOW Clear Preview", rollback=True):
            if entry_ids is None:
                proxy_manager.delete_proxies()
            else:
                proxy_manager.delete_proxies(entry_ids=entry_ids)

    @staticmethod
    def show_original(visible):
        proxy_manager.set_group_visibility("SIMFLOW_INPUT", visible)

    @staticmethod
    def show_simulated(visible):
        proxy_manager.set_group_visibility("SIMFLOW_OUTPUT", visible)
