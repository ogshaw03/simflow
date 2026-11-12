# -*- coding: utf-8 -*-
"""Logging helper.  Messages go to the Maya Script Editor (stdout)."""
import logging
import sys

_LOGGER_NAME = "simflow"


def get_logger(name=None):
    logger = logging.getLogger(_LOGGER_NAME if not name else "%s.%s" % (_LOGGER_NAME, name))
    root = logging.getLogger(_LOGGER_NAME)
    if not root.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter("[SIMFLOW] %(levelname)s: %(message)s"))
        root.addHandler(handler)
        root.setLevel(logging.INFO)
        root.propagate = False
    return logger
