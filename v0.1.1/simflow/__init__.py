# -*- coding: utf-8 -*-
"""SIMFLOW - simulation assign tool for Maya 2025.

Usage inside Maya::

    import simflow
    simflow.show()
"""
__version__ = "0.1.1"
TOOL_NAME = "SIMFLOW"


def show():
    from . import main
    return main.show()
