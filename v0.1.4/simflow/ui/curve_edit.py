# -*- coding: utf-8 -*-
"""Curve editor for a root -> tip profile (like Maya's ramp / taper curves).

Same handling as Maya's ramp attributes:

* click empty space: add a point there (keep the button down to drag it)
* drag a point (end points move only up / down)
* right-click a point, or select it and press Delete / Backspace: delete it
  (the end points cannot be deleted)
* the vertical ticks / dots show where each controller of the chain sits and
  the value it gets
"""
from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets

from ..core import curve
from . import widgets as W

HIT = 8           # px


class CurveEdit(QtWidgets.QWidget):
    curveChanged = QtCore.Signal(tuple)      # while editing and on release
    selectionChanged = QtCore.Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(96)
        self.setMinimumWidth(120)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.ends = ("根元", "先端")          # axis labels (a rope: grips -> middle)
        self._pts = list(curve.FLAT)
        self._sel = 0
        self._drag = False
        self._markers = 0
        self.setToolTip("クリック：点を追加（そのままドラッグで位置を決める）\nドラッグ：点を動かす\n"
                        "右クリック / Delete キー：点を削除（両端の点は消せません）")

    # ---- data -------------------------------------------------------------
    def points(self):
        return tuple(self._pts)

    def setPoints(self, points):
        self._pts = list(curve.normalise(points))
        self._sel = min(self._sel, len(self._pts) - 1)
        self.update()

    def setMarkers(self, count):
        """Number of controllers of the chain (0 = no markers)."""
        self._markers = int(count)
        self.update()

    def selected(self):
        return self._sel

    def selectedValue(self):
        return self._pts[self._sel][1]

    def setSelectedValue(self, y):
        x = self._pts[self._sel][0]
        self._pts[self._sel] = (x, min(1.0, max(0.0, float(y))))
        self.update()
        self.curveChanged.emit(self.points())

    # ---- geometry -----------------------------------------------------------
    def _area(self):
        return QtCore.QRectF(10, 8, self.width() - 20, self.height() - 26)

    def _to_px(self, x, y):
        r = self._area()
        return QtCore.QPointF(r.left() + x * r.width(), r.bottom() - y * r.height())

    def _from_px(self, pos):
        r = self._area()
        x = (pos.x() - r.left()) / max(1.0, r.width())
        y = (r.bottom() - pos.y()) / max(1.0, r.height())
        return min(1.0, max(0.0, x)), min(1.0, max(0.0, y))

    def _hit(self, pos):
        best, dist = None, HIT * HIT
        for i, (x, y) in enumerate(self._pts):
            p = self._to_px(x, y)
            d = (p.x() - pos.x()) ** 2 + (p.y() - pos.y()) ** 2
            if d <= dist:
                best, dist = i, d
        return best

    # ---- painting -----------------------------------------------------------
    def paintEvent(self, _event):
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing)
        r = self._area()
        qp.fillRect(self.rect(), QtGui.QColor(W.PANEL))
        qp.fillRect(r, QtGui.QColor(W.FIELD))
        grid = QtGui.QPen(QtGui.QColor("#3a3a3a"), 1, QtCore.Qt.DashLine)
        qp.setPen(grid)
        for v in (0.25, 0.5, 0.75):
            p = self._to_px(0, v)
            qp.drawLine(QtCore.QPointF(r.left(), p.y()), QtCore.QPointF(r.right(), p.y()))
        # controller positions
        n = self._markers
        if n > 1:
            qp.setPen(QtGui.QPen(QtGui.QColor("#4a4a4a"), 1))
            for i in range(n):
                p = self._to_px(i / (n - 1.0), 0)
                qp.drawLine(QtCore.QPointF(p.x(), r.top()), QtCore.QPointF(p.x(), r.bottom()))
        # curve
        path = QtGui.QPainterPath()
        steps = max(40, int(r.width() / 3))
        for k in range(steps + 1):
            x = k / float(steps)
            p = self._to_px(x, curve.evaluate(self._pts, x))
            if k == 0:
                path.moveTo(p)
            else:
                path.lineTo(p)
        qp.setPen(QtGui.QPen(QtGui.QColor(W.ACCENT), 2))
        qp.drawPath(path)
        # value each controller gets
        if n > 1:
            qp.setPen(QtCore.Qt.NoPen)
            qp.setBrush(QtGui.QColor("#e8e8e8"))
            for i in range(n):
                x = i / (n - 1.0)
                qp.drawEllipse(self._to_px(x, curve.evaluate(self._pts, x)), 2.5, 2.5)
        # control points
        for i, (x, y) in enumerate(self._pts):
            p = self._to_px(x, y)
            qp.setPen(QtGui.QPen(QtGui.QColor("#1e1e1e"), 1))
            qp.setBrush(QtGui.QColor(W.ACCENT if i == self._sel else "#e8e8e8"))
            qp.drawRect(QtCore.QRectF(p.x() - 4, p.y() - 4, 8, 8))
        # labels
        qp.setPen(QtGui.QColor(W.MUTED))
        f = qp.font()
        f.setPointSizeF(max(6.5, f.pointSizeF() - 1))
        qp.setFont(f)
        qp.drawText(QtCore.QRectF(r.left(), r.bottom() + 2, 80, 14), QtCore.Qt.AlignLeft, self.ends[0])
        qp.drawText(QtCore.QRectF(r.right() - 80, r.bottom() + 2, 80, 14), QtCore.Qt.AlignRight, self.ends[1])
        qp.end()

    # ---- mouse ----------------------------------------------------------------
    def mousePressEvent(self, ev):
        i = self._hit(ev.position())
        if ev.button() == QtCore.Qt.RightButton:
            if i is not None:
                self._delete(i)
            return
        if ev.button() != QtCore.Qt.LeftButton:
            return
        self.setFocus(QtCore.Qt.MouseFocusReason)
        if i is None:
            x, y = self._from_px(ev.position())
            i = self.addPoint(x, y)
            if i is None:
                return
        self._select(i)
        self._drag = True

    def mouseMoveEvent(self, ev):
        if not self._drag:
            self.setCursor(QtCore.Qt.SizeAllCursor if self._hit(ev.position()) is not None
                           else QtCore.Qt.ArrowCursor)
            return
        x, y = self._from_px(ev.position())
        i = self._sel
        if i == 0:
            x = 0.0
        elif i == len(self._pts) - 1:
            x = 1.0
        else:
            x = min(max(x, self._pts[i - 1][0] + 0.02), self._pts[i + 1][0] - 0.02)
        self._pts[i] = (x, y)
        self.update()
        self.curveChanged.emit(self.points())

    def mouseReleaseEvent(self, ev):
        if self._drag:
            self._drag = False
            self.curveChanged.emit(self.points())

    def mouseDoubleClickEvent(self, ev):
        # the second click of a double click arrives here instead of
        # mousePressEvent: handle it as a normal press (no extra point)
        self.mousePressEvent(ev)

    def event(self, ev):
        # Delete / Backspace are Maya hotkeys (delete the selected nodes): while
        # the curve has the focus they must reach the curve, not Maya
        if ev.type() == QtCore.QEvent.ShortcutOverride and \
                ev.key() in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
            ev.accept()
            return True
        return super().event(ev)

    def keyPressEvent(self, ev):
        if ev.key() in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
            self._delete(self._sel)
            return
        super().keyPressEvent(ev)

    # ---- edits ----------------------------------------------------------------
    def addPoint(self, x, y):
        """Add a point; returns its index (None when too close to another)."""
        x = min(0.98, max(0.02, x))
        if any(abs(px - x) < 0.02 for px, _ in self._pts):
            return None
        self._pts.append((x, min(1.0, max(0.0, y))))
        self._pts.sort()
        i = [p[0] for p in self._pts].index(x)
        self._select(i)
        self.update()
        self.curveChanged.emit(self.points())
        return i

    def _delete(self, i):
        if i in (0, len(self._pts) - 1):
            return
        del self._pts[i]
        self._select(min(i, len(self._pts) - 1))
        self.update()
        self.curveChanged.emit(self.points())

    def _select(self, i):
        self._sel = i
        self.update()
        self.selectionChanged.emit(i)
