# -*- coding: utf-8 -*-
"""Reusable PySide6 widgets and the SIMFLOW style sheet."""
from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets

ACCENT = "#f2c230"
BG = "#2b2b2b"
PANEL = "#333333"
FIELD = "#232323"
TEXT = "#d6d6d6"
MUTED = "#8a8a8a"

STYLE = """
QWidget#SimflowRoot { background: %(bg)s; color: %(text)s; }
QLabel { color: %(text)s; }
QLabel[muted="true"] { color: %(muted)s; }
QLabel#Title { color: %(accent)s; font-size: 16px; font-weight: bold; letter-spacing: 2px; }
QFrame#SectionBody { background: %(panel)s; border: 1px solid #3c3c3c; border-top: none;
                     border-bottom-left-radius: 3px; border-bottom-right-radius: 3px; }
QToolButton#SectionHeader { background: #3a3a3a; color: %(text)s; border: 1px solid #3c3c3c;
                     border-top-left-radius: 3px; border-top-right-radius: 3px;
                     padding: 4px 6px; font-weight: bold; text-align: left; }
QToolButton#SectionHeader:checked { border-left: 3px solid %(accent)s; }
QToolButton { background: #444; color: %(text)s; border: 1px solid #505050; border-radius: 3px;
              padding: 3px 8px; }
QToolButton:hover { background: #4d4d4d; }
QToolButton::menu-indicator { image: none; }
QPushButton { background: #444; color: %(text)s; border: 1px solid #505050; border-radius: 3px;
              padding: 4px 8px; }
QPushButton:hover { background: #4d4d4d; }
QPushButton:pressed { background: #3a3a3a; }
QPushButton:disabled { color: #666; background: #383838; border-color: #404040; }
QPushButton[accent="true"] { background: %(accent)s; color: #1e1e1e; font-weight: bold;
                             border-color: %(accent)s; }
QPushButton[accent="true"]:hover { background: #ffd451; }
QPushButton[accent="true"]:disabled { background: #6b5b22; color: #333; }
QListWidget, QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background: %(field)s; color: %(text)s; border: 1px solid #444; border-radius: 2px; padding: 1px 3px; }
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled { color: #666; background: #2a2a2a; }
QListWidget::item:selected { background: #4b4228; color: #fff; }
QSlider::groove:horizontal { height: 4px; background: #1d1d1d; border-radius: 2px; }
QSlider::sub-page:horizontal { background: %(accent)s; border-radius: 2px; }
QSlider::handle:horizontal { background: #e8e8e8; width: 10px; margin: -5px 0; border-radius: 5px; }
QSlider::handle:horizontal:disabled { background: #666; }
QSlider::sub-page:horizontal:disabled { background: #6b5b22; }
QCheckBox::indicator:checked, QRadioButton::indicator:checked { background: %(accent)s;
    border: 1px solid %(accent)s; }
QCheckBox::indicator, QRadioButton::indicator { width: 11px; height: 11px; background: %(field)s;
    border: 1px solid #555; }
QRadioButton::indicator { border-radius: 6px; }
QLabel#Status { color: %(muted)s; padding: 2px; }
QCheckBox::indicator:indeterminate { background: #6b5b22; border: 1px solid %(accent)s; }
""" % dict(accent=ACCENT, bg=BG, panel=PANEL, field=FIELD, text=TEXT, muted=MUTED)


class Section(QtWidgets.QWidget):
    """Collapsible titled section."""

    def __init__(self, title, parent=None, expanded=True, note=""):
        super().__init__(parent)
        self.header = QtWidgets.QToolButton(text=title + (("   " + note) if note else ""))
        self.header.setObjectName("SectionHeader")
        self.header.setCheckable(True)
        self.header.setChecked(expanded)
        self.header.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.header.setArrowType(QtCore.Qt.DownArrow if expanded else QtCore.Qt.RightArrow)
        self.header.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.body = QtWidgets.QFrame()
        self.body.setObjectName("SectionBody")
        self.body_layout = QtWidgets.QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(8, 6, 8, 8)
        self.body_layout.setSpacing(5)
        self.body.setVisible(expanded)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self.header)
        lay.addWidget(self.body)
        self.header.toggled.connect(self._toggle)

    def _toggle(self, on):
        self.header.setArrowType(QtCore.Qt.DownArrow if on else QtCore.Qt.RightArrow)
        self.body.setVisible(on)

    def add(self, widget):
        self.body_layout.addWidget(widget)
        return widget

    def add_layout(self, layout):
        self.body_layout.addLayout(layout)
        return layout


class ParamSlider(QtWidgets.QWidget):
    """Label + slider + spin box.  The spin box may exceed the slider range
    (e.g. Gravity > 100)."""

    valueChanged = QtCore.Signal(float)

    def __init__(self, label, value=0.0, minimum=0.0, maximum=100.0, slider_max=None,
                 decimals=1, tooltip="", label_width=80, parent=None):
        super().__init__(parent)
        self._scale = 10.0 ** decimals
        slider_max = maximum if slider_max is None else slider_max
        self.label = QtWidgets.QLabel(label)
        self.label.setFixedWidth(label_width)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(int(minimum * self._scale), int(slider_max * self._scale))
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setDecimals(decimals)
        self.spin.setRange(minimum, maximum)
        self.spin.setFixedWidth(64)
        self.spin.setKeyboardTracking(False)
        self.spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.label)
        lay.addWidget(self.slider, 1)
        lay.addWidget(self.spin)
        if tooltip:
            for w in (self.label, self.slider, self.spin):
                w.setToolTip(tooltip)
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)
        self.setValue(value)

    def _from_slider(self, v):
        value = v / self._scale
        if abs(self.spin.value() - value) > 0.5 / self._scale:
            self.spin.blockSignals(True)
            self.spin.setValue(value)
            self.spin.blockSignals(False)
            self.valueChanged.emit(value)

    def _from_spin(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(min(value, self.slider.maximum() / self._scale) * self._scale)))
        self.slider.blockSignals(False)
        self.valueChanged.emit(value)

    def value(self):
        return self.spin.value()

    def setValue(self, value):
        self.spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.spin.setValue(value)
        self.slider.setValue(int(round(min(value, self.slider.maximum() / self._scale) * self._scale)))
        self.spin.blockSignals(False)
        self.slider.blockSignals(False)


class AxisToggles(QtWidgets.QWidget):
    changed = QtCore.Signal(tuple)

    def __init__(self, label, values=(False, False, False), label_width=80, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lbl = QtWidgets.QLabel(label)
        lbl.setFixedWidth(label_width)
        lay.addWidget(lbl)
        self.boxes = []
        for axis, on in zip("XYZ", values):
            cb = QtWidgets.QCheckBox(axis)
            cb.setChecked(on)
            cb.toggled.connect(lambda _=None: self.changed.emit(self.values()))
            lay.addWidget(cb)
            self.boxes.append(cb)
        lay.addStretch(1)

    def values(self):
        return tuple(cb.isChecked() for cb in self.boxes)

    def setValues(self, values):
        for cb, v in zip(self.boxes, values):
            cb.blockSignals(True)
            cb.setChecked(bool(v))
            cb.blockSignals(False)


class Vec3Edit(QtWidgets.QWidget):
    changed = QtCore.Signal(tuple)

    def __init__(self, label, values=(0.0, 0.0, 0.0), label_width=80, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lbl = QtWidgets.QLabel(label)
        lbl.setFixedWidth(label_width)
        lay.addWidget(lbl)
        self.spins = []
        for v in values:
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setDecimals(3)
            sp.setSingleStep(0.1)
            sp.setValue(v)
            sp.setKeyboardTracking(False)
            sp.valueChanged.connect(lambda _=None: self.changed.emit(self.values()))
            lay.addWidget(sp)
            self.spins.append(sp)

    def values(self):
        return tuple(sp.value() for sp in self.spins)

    def setValues(self, values):
        for sp, v in zip(self.spins, values):
            sp.blockSignals(True)
            sp.setValue(v)
            sp.blockSignals(False)


def labeled(label, widget, label_width=80):
    w = QtWidgets.QWidget()
    lay = QtWidgets.QHBoxLayout(w)
    lay.setContentsMargins(0, 0, 0, 0)
    lbl = QtWidgets.QLabel(label)
    lbl.setFixedWidth(label_width)
    lay.addWidget(lbl)
    lay.addWidget(widget, 1)
    return w


def accent_button(text, tooltip=""):
    b = QtWidgets.QPushButton(text)
    b.setProperty("accent", True)
    b.setMinimumHeight(28)
    if tooltip:
        b.setToolTip(tooltip)
    return b


def button(text, tooltip=""):
    b = QtWidgets.QPushButton(text)
    if tooltip:
        b.setToolTip(tooltip)
    return b


def muted_label(text):
    lbl = QtWidgets.QLabel(text)
    lbl.setProperty("muted", True)
    lbl.setWordWrap(True)
    return lbl
