# -*- coding: utf-8 -*-
"""nHair solver - Phase 3 (not implemented yet).

This module intentionally raises instead of pretending to work.  The planned
design (spec 2.3): build a NURBS curve through the registered controllers'
pivots, make it dynamic with an existing or new hairSystem / nucleus, drive
the follicle from the Input Proxy, evaluate the Nucleus over the range,
compute per-controller orientation from the curve with a reference up vector
and feed the result into the same SolverResult format used by the Spring
solver so the preview / bake pipeline is shared.
"""
from .base_solver import BaseSolver


class NHairSolver(BaseSolver):
    name = "nhair"

    def calculate(self, progress=None):
        raise NotImplementedError(
            "nHair モードは Phase 3 で実装予定のため、まだ実装されていません。")
