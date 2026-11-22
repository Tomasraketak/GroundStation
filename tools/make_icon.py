#!/usr/bin/env python3
# Usage: QT_QPA_PLATFORM=offscreen python3 tools/make_icon.py tools/windows/groundstation.ico tools/windows/groundstation.png
# Draws the ground station icon (antenna + rocket) and packs a PNG-in-ICO.
import struct
import sys
from PyQt6 import QtCore, QtGui
from PyQt6.QtGui import QGuiApplication
app = QGuiApplication(sys.argv)

def draw(n):
    img = QtGui.QImage(n, n, QtGui.QImage.Format.Format_ARGB32)
    img.fill(0)
    p = QtGui.QPainter(img)
    p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
    s = n / 256.0
    p.scale(s, s)
    p.setPen(QtCore.Qt.PenStyle.NoPen)
    p.setBrush(QtGui.QColor("#161826"))
    p.drawRoundedRect(QtCore.QRectF(8, 8, 240, 240), 52, 52)
    # radio waves from the dish
    amber = QtGui.QColor("#f5a51c")
    for r, w in ((70, 14), (108, 14), (146, 14)):
        pen = QtGui.QPen(amber, w, cap=QtCore.Qt.PenCapStyle.RoundCap)
        p.setPen(pen)
        p.drawArc(QtCore.QRectF(70 - r, 190 - r, 2 * r, 2 * r), 10 * 16, 70 * 16)
    # dish
    p.setPen(QtCore.Qt.PenStyle.NoPen)
    p.setBrush(QtGui.QColor("#e9e9ed"))
    p.drawPie(QtCore.QRectF(30, 150, 80, 80), 45 * 16, 180 * 16)
    p.drawRect(QtCore.QRectF(62, 196, 14, 36))
    p.drawRoundedRect(QtCore.QRectF(44, 226, 50, 10), 4, 4)
    # rocket (top right), nose up-right
    p.save()
    p.translate(188, 70)
    p.rotate(45)
    p.setBrush(QtGui.QColor("#e9e9ed"))
    body = QtGui.QPainterPath()
    body.moveTo(0, -46); body.cubicTo(14, -30, 14, -10, 13, 24); body.lineTo(-13, 24); body.cubicTo(-14, -10, -14, -30, 0, -46)
    p.drawPath(body)
    p.setBrush(QtGui.QColor("#5dd39e"))
    for sgn in (-1, 1):
        fin = QtGui.QPainterPath()
        fin.moveTo(sgn * 12, 6); fin.lineTo(sgn * 26, 30); fin.lineTo(sgn * 12, 24); fin.closeSubpath()
        p.drawPath(fin)
    p.setBrush(QtGui.QColor("#f2777a"))
    flame = QtGui.QPainterPath()
    flame.moveTo(-9, 26); flame.quadTo(0, 60, 9, 26); flame.closeSubpath()
    p.drawPath(flame)
    p.setBrush(QtGui.QColor("#161826"))
    p.drawEllipse(QtCore.QPointF(0, -14), 6, 6)
    p.restore()
    p.end()
    return img

sizes = [16, 24, 32, 48, 64, 128, 256]
blobs = []
for n in sizes:
    ba = QtCore.QByteArray()
    buf = QtCore.QBuffer(ba)
    buf.open(QtCore.QIODevice.OpenModeFlag.WriteOnly)
    draw(n).save(buf, "PNG")
    blobs.append(bytes(ba))
draw(256).save(sys.argv[2], "PNG")
hdr = struct.pack("<HHH", 0, 1, len(sizes))
off = 6 + 16 * len(sizes)
ent = b""
for n, b in zip(sizes, blobs):
    ent += struct.pack("<BBBBHHII", n % 256, n % 256, 0, 0, 1, 32, len(b), off)
    off += len(b)
open(sys.argv[1], "wb").write(hdr + ent + b"".join(blobs))
