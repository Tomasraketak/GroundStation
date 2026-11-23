"""Widgets and the look of the ground station GUI.

Two themes, as on the Daidalos station: "VENKU" (light, high contrast, for a
~300 nit laptop panel in daylight — a dark UI drowns in reflections there)
and "NOC" (dark, for dusk). Status is shown by solid fills ("dark cockpit":
a healthy value is neutral with a green stripe, only a warning (amber) or an
alarm (red) shouts).
"""
from __future__ import annotations

import concurrent.futures
import math
import os
import sys
import urllib.request
from collections import deque
from pathlib import Path

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

# ---------------------------------------------------------------- data folder
def user_dir() -> str:
    """<Documents>/GroundStation (or $GROUNDSTATION_DATA): logs, map cache, pad.json."""
    base = os.environ.get("GROUNDSTATION_DATA", "")
    if not base:
        docs = QtCore.QStandardPaths.writableLocation(
            QtCore.QStandardPaths.StandardLocation.DocumentsLocation) or os.path.expanduser("~")
        base = os.path.join(docs, "GroundStation")
    os.makedirs(base, exist_ok=True)
    return base


# ---------------------------------------------------------------- themes
THEMES = {
    "light": dict(
        bg="#dfe4e9", panel="#ffffff", panel2="#eef1f4", border="#8995a1", dim="#6c7783",
        text="#070b0f", muted="#2f3943", accent="#c2410c",
        ok="#0d7a34", ok_text="#ffffff", warn="#f5ab00", warn_text="#000000",
        bad="#c8102e", bad_text="#ffffff", info="#1356b5", info_text="#ffffff",
        idle="#d2d8de", idle_text="#38424c", plain="#ffffff", plain_text="#070b0f",
        grid="#c3cad1", s1="#1356b5", s2="#c2410c", s3="#0d7a34", s4="#7b2fbf",
        map_bg="#cfd6dc"),
    "dark": dict(
        bg="#12161c", panel="#1b2129", panel2="#222a34", border="#3a4552", dim="#7f8b97",
        text="#e9edf2", muted="#b4bec8", accent="#ff8a3d",
        ok="#2ea043", ok_text="#ffffff", warn="#e3a008", warn_text="#000000",
        bad="#f04848", bad_text="#ffffff", info="#3b82f6", info_text="#ffffff",
        idle="#2b333d", idle_text="#aab4be", plain="#222a34", plain_text="#e9edf2",
        grid="#2b333d", s1="#58a6ff", s2="#ff8a3d", s3="#3fb950", s4="#bc8cff",
        map_bg="#1b2129"),
}
T = dict(THEMES["light"])
THEME_LABEL = {"light": "☀ VENKU", "dark": "☾ NOC"}


def C(key: str) -> QtGui.QColor:
    return QtGui.QColor(T[key])


def build_qss() -> str:
    t = T
    lv = ""
    # repeated with the panel prefix: "QFrame#Panel QWidget" (transparent) would win otherwise
    for pre in ("", "QFrame#Panel "):
        for name in ("ok", "warn", "bad", "info", "idle", "plain"):
            lv += f'{pre}QLabel[level="{name}"] {{ background: {t[name]}; color: {t[name + "_text"]}; }}\n'
        lv += f'{pre}QLabel[level="accent"] {{ background: {t["accent"]}; color: #ffffff; }}\n'
        lv += (f'{pre}QLabel[level="okline"] {{ background: {t["panel2"]}; color: {t["text"]}; '
               f'border-left: 6px solid {t["ok"]}; }}\n')
    return f"""
QWidget {{ background: {t['bg']}; color: {t['text']}; font-size: 13px; }}
QFrame#Panel {{ background: {t['panel']}; border: 1px solid {t['border']}; border-radius: 8px; }}
QFrame#Panel QLabel, QFrame#Panel QCheckBox, QWidget#Plain {{ background: transparent; }}
QLabel#Caption {{ color: {t['muted']}; font-size: 11px; font-weight: 800; letter-spacing: 1px; background: transparent; }}
QLabel#Brand {{ font-size: 20px; font-weight: 900; letter-spacing: 2px; background: transparent; }}
QLabel#BrandSub {{ color: {t['muted']}; font-size: 11px; font-weight: 700; background: transparent; }}
QLabel#State {{ font-size: 24px; font-weight: 900; border-radius: 8px; padding: 4px 14px; }}
QLabel#Clock {{ font-size: 26px; font-weight: 900; border-radius: 8px; padding: 2px 12px;
               font-family: "Consolas", "JetBrains Mono", monospace; }}
QLabel#Badge {{ font-size: 13px; font-weight: 800; border-radius: 6px; padding: 4px 10px; }}
QLabel#Lamp {{ font-size: 12px; font-weight: 900; border-radius: 5px; }}
QFrame#Tile {{ background: {t['panel']}; border: 1px solid {t['border']}; border-radius: 8px; }}
QFrame#Tile QLabel {{ background: transparent; }}
QFrame#Tile[level="ok"]     {{ border-left: 6px solid {t['ok']}; }}
QFrame#Tile[level="warn"]   {{ background: {t['warn']}; }}
QFrame#Tile[level="warn"] QLabel {{ color: {t['warn_text']}; }}
QFrame#Tile[level="bad"]    {{ background: {t['bad']}; }}
QFrame#Tile[level="bad"] QLabel {{ color: {t['bad_text']}; }}
QFrame#Tile[level="info"]   {{ border-left: 6px solid {t['info']}; }}
QFrame#Tile[level="accent"] {{ border-left: 6px solid {t['accent']}; }}
QLabel#TileTitle {{ color: {t['muted']}; font-size: 11px; font-weight: 800; }}
QLabel#TileValue {{ font-size: 22px; font-weight: 900; font-family: "Consolas", "JetBrains Mono", monospace; }}
QLabel#TileValueHero {{ font-size: 34px; font-weight: 900; font-family: "Consolas", "JetBrains Mono", monospace; }}
QLabel#TileSub {{ color: {t['muted']}; font-size: 12px; font-weight: 700; }}
QPushButton {{ background: {t['panel2']}; border: 1px solid {t['border']}; border-radius: 6px; padding: 6px 12px;
              font-weight: 800; color: {t['text']}; }}
QPushButton:hover {{ border-color: {t['info']}; }}
QPushButton:disabled {{ color: {t['dim']}; border-color: {t['grid']}; }}
QPushButton:checked {{ background: {t['info']}; color: {t['info_text']}; border-color: {t['info']}; }}
QPushButton[kind="abort"] {{ background: {t['bad']}; color: #ffffff; border: 2px solid #5c0010; font-size: 22px;
                            font-weight: 900; padding: 8px 26px; }}
QPushButton[kind="abort"]:hover {{ background: #e0142f; }}
QPushButton[kind="go"] {{ background: {t['ok']}; color: #ffffff; border-color: {t['ok']}; }}
QPushButton[kind="warn"] {{ background: {t['warn']}; color: #000000; border-color: #8a5d00; }}
QPushButton[kind="danger"] {{ color: {t['bad']}; border-color: {t['bad']}; }}
QPushButton[kind="primary"] {{ color: {t['info']}; border-color: {t['info']}; }}
QPushButton[kind="abort"]:disabled, QPushButton[kind="go"]:disabled, QPushButton[kind="warn"]:disabled,
QPushButton[kind="primary"]:disabled, QPushButton[kind="danger"]:disabled {{
    background: {t['idle']}; color: {t['dim']}; border-color: {t['grid']}; }}
QPushButton#Rocket {{ font-size: 15px; padding: 6px 16px; }}
QPushButton#Rocket:checked {{ background: {t['accent']}; color: #ffffff; border-color: {t['accent']}; }}
QLineEdit, QSpinBox, QComboBox, QPlainTextEdit {{ background: {t['panel']}; border: 1px solid {t['border']};
    border-radius: 5px; padding: 3px 6px; }}
QCheckBox {{ background: transparent; font-weight: 700; spacing: 8px; }}
QCheckBox::indicator {{ width: 18px; height: 18px; }}
QTabBar::tab {{ background: {t['panel2']}; color: {t['muted']}; padding: 8px 18px; font-weight: 900;
               border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px; }}
QTabBar::tab:selected {{ background: {t['panel']}; color: {t['text']}; border-bottom: 3px solid {t['accent']}; }}
QTabWidget::pane {{ border: none; }}
QStatusBar {{ color: {t['muted']}; font-weight: 700; }}
{lv}
"""


def apply_theme(app: QtWidgets.QApplication, key: str) -> None:
    T.clear()
    T.update(THEMES.get(key, THEMES["light"]))
    app.setStyleSheet(build_qss())


def set_level(w: QtWidgets.QWidget, level: str, prop: str = "level") -> None:
    if w.property(prop) == level:
        return
    w.setProperty(prop, level)
    w.style().unpolish(w)
    w.style().polish(w)


def badge(text: str, level: str = "idle") -> QtWidgets.QLabel:
    lb = QtWidgets.QLabel(text)
    lb.setObjectName("Badge")
    lb.setProperty("level", level)
    lb.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return lb


def kbutton(text: str, kind: str = "", tip: str = "") -> QtWidgets.QPushButton:
    b = QtWidgets.QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if tip:
        b.setToolTip(tip)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    return b


def caption(text: str) -> QtWidgets.QLabel:
    lb = QtWidgets.QLabel(text.upper())
    lb.setObjectName("Caption")
    return lb


def panel(title: str) -> tuple[QtWidgets.QFrame, QtWidgets.QVBoxLayout]:
    fr = QtWidgets.QFrame()
    fr.setObjectName("Panel")
    v = QtWidgets.QVBoxLayout(fr)
    v.setContentsMargins(10, 6, 10, 10)
    v.setSpacing(6)
    v.addWidget(caption(title))
    return fr, v


class Tile(QtWidgets.QFrame):
    def __init__(self, title: str, hero: bool = False):
        super().__init__()
        self.setObjectName("Tile")
        self.setProperty("level", "idle")
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(10, 5, 10, 6)
        v.setSpacing(0)
        self.title = QtWidgets.QLabel(title.upper())
        self.title.setObjectName("TileTitle")
        self.value = QtWidgets.QLabel("--")
        self.value.setObjectName("TileValueHero" if hero else "TileValue")
        self.sub = QtWidgets.QLabel(" ")
        self.sub.setObjectName("TileSub")
        for w in (self.title, self.value, self.sub):
            v.addWidget(w)

    def set(self, value: str, sub: str | None = None, level: str | None = None, title: str | None = None):
        self.value.setText(value)
        if sub is not None:
            self.sub.setText(sub)
        if title is not None:
            self.title.setText(title.upper())
        if level is not None:
            set_level(self, level)


class Strip(QtWidgets.QFrame):
    """A row of small lamps with labels (health bits, pyro, events)."""

    def __init__(self, title: str = ""):
        super().__init__()
        self.setObjectName("Panel")
        self.v = QtWidgets.QVBoxLayout(self)
        self.v.setContentsMargins(8, 4, 8, 6)
        self.cap = caption(title)
        self.v.addWidget(self.cap)
        self.grid = QtWidgets.QGridLayout()
        self.grid.setSpacing(4)
        self.v.addLayout(self.grid)
        self.lamps: dict[str, QtWidgets.QLabel] = {}
        self.names: list[str] = []

    def set(self, title: str, items: list[tuple[str, bool, bool]], cols: int = 5):
        names = [n for n, _, _ in items]
        if names != self.names:
            while self.grid.count():
                w = self.grid.takeAt(0).widget()
                if w:
                    w.hide()
                    w.setParent(None)
                    w.deleteLater()
            self.lamps = {}
            for i, n in enumerate(names):
                lb = badge(n)
                lb.setMinimumWidth(56)
                self.lamps[n] = lb
                self.grid.addWidget(lb, i // cols, i % cols)
            self.names = names
        self.cap.setText(title.upper())
        for n, on, warn_off in items:
            set_level(self.lamps[n], "ok" if on else ("warn" if warn_off else "idle"))


class CheckRow(QtWidgets.QFrame):
    def __init__(self):
        super().__init__()
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(8)
        self.lamp = QtWidgets.QLabel("—")
        self.lamp.setObjectName("Lamp")
        self.lamp.setProperty("level", "idle")
        self.lamp.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lamp.setFixedSize(64, 24)
        self.name = QtWidgets.QLabel()
        self.name.setStyleSheet("font-weight: 800; background: transparent;")
        self.name.setMinimumWidth(190)
        self.detail = QtWidgets.QLabel()
        self.detail.setStyleSheet("background: transparent;")
        self.detail.setWordWrap(True)
        h.addWidget(self.lamp)
        h.addWidget(self.name)
        h.addWidget(self.detail, 1)
        self.level = "idle"

    def set(self, name: str, level: str, detail: str):
        self.level = level
        self.name.setText(name)
        self.detail.setText(detail)
        self.lamp.setText({"ok": "OK", "warn": "POZOR", "bad": "NO-GO", "info": "INFO"}.get(level, "—"))
        set_level(self.lamp, level)


class StripChart(QtWidgets.QWidget):
    """Rolling time chart, several series, grid and labels. No pyqtgraph needed."""

    def __init__(self, title: str, series: list[tuple[str, str]], span_s: float = 120.0, unit: str = "",
                 floor: float | None = None, min_span: float = 0.0):
        super().__init__()
        self.title, self.series, self.span, self.unit, self.floor = title, series, span_s, unit, floor
        self.min_span = min_span          # smallest y range shown (keeps sensor noise from filling the chart)
        self.data = {name: deque(maxlen=8000) for name, _ in series}
        self.setMinimumHeight(140)

    def add(self, t: float, **values):
        for k, v in values.items():
            if k in self.data and v is not None and isinstance(v, (int, float)) and math.isfinite(v):
                self.data[k].append((t, v))

    def clear(self):
        for d in self.data.values():
            d.clear()
        self.update()

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QtGui.QPen(C("border"), 1))
        p.setBrush(C("panel"))
        p.drawRoundedRect(QtCore.QRectF(0.5, 0.5, w - 1, h - 1), 8, 8)
        l, r, t, b = 54, 10, 24, 20
        pw, ph = w - l - r, h - t - b
        f = p.font()
        f.setBold(True)
        f.setPixelSize(12)
        p.setFont(f)
        p.setPen(C("text"))
        p.drawText(10, 16, f"{self.title} [{self.unit}]" if self.unit else self.title)
        now = max((d[-1][0] for d in self.data.values() if d), default=0.0)
        t0 = now - self.span
        vals = [v for d in self.data.values() for (tt, v) in d if tt >= t0]
        if not vals or pw < 20 or ph < 20:
            p.setPen(C("dim"))
            p.drawText(l + 10, t + ph // 2, "žádná data")
            return
        lo, hi = min(vals), max(vals)
        if self.floor is not None:
            lo = min(lo, self.floor)
            hi = max(hi, self.floor + 10)
        if hi - lo < 1e-6:
            hi, lo = hi + 1, lo - 1
        if hi - lo < self.min_span:
            mid = (hi + lo) / 2
            hi, lo = mid + self.min_span / 2, mid - self.min_span / 2
        pad = (hi - lo) * 0.08
        lo, hi = lo - pad, hi + pad
        if self.floor is not None:
            lo = max(lo, self.floor)
        f.setBold(False)
        f.setPixelSize(11)
        p.setFont(f)
        for i in range(5):
            y = t + ph * i / 4
            p.setPen(QtGui.QPen(C("grid"), 1))
            p.drawLine(l, int(y), l + pw, int(y))
            p.setPen(C("muted"))
            val = hi - (hi - lo) * i / 4
            p.drawText(2, int(y) + 4, f"{val:7.2f}" if hi - lo < 5 else f"{val:7.1f}" if hi - lo < 200 else f"{val:7.0f}")
        for i in range(7):
            x = l + pw * i / 6
            p.setPen(QtGui.QPen(C("grid"), 1))
            p.drawLine(int(x), t, int(x), t + ph)
            p.setPen(C("muted"))
            p.drawText(int(x) - 14, h - 5, f"{-self.span + self.span * i / 6:.0f}s")
        f.setBold(True)
        p.setFont(f)
        for k, (name, col) in enumerate(self.series):
            pts = [(tt, v) for (tt, v) in self.data[name] if tt >= t0]
            color = C(col) if col in T else QtGui.QColor(col)
            p.setPen(QtGui.QPen(color, 2))
            p.drawText(l + 10 + 90 * k + 160, 16, f"■ {name}")
            if len(pts) < 2:
                continue
            path = QtGui.QPainterPath()
            for i, (tt, v) in enumerate(pts):
                x = l + (tt - t0) / self.span * pw
                y = t + (hi - v) / (hi - lo) * ph
                path.moveTo(x, y) if i == 0 else path.lineTo(x, y)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(path)


class TiltView(QtWidgets.QWidget):
    """Side view of the rocket + a bull's-eye: where the nose leans and how far."""

    def __init__(self):
        super().__init__()
        self.tilt = None
        self.az = None
        self.nose_up = True
        self.setMinimumSize(220, 150)

    def set(self, tilt, az, nose_up):
        self.tilt, self.az, self.nose_up = tilt, az, True if nose_up is None else nose_up
        self.update()

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QtGui.QPen(C("border"), 1))
        p.setBrush(C("panel"))
        p.drawRoundedRect(QtCore.QRectF(0.5, 0.5, w - 1, h - 1), 8, 8)
        f = p.font()
        f.setBold(True)
        f.setPixelSize(12)
        p.setFont(f)
        p.setPen(C("text"))
        if self.tilt is None:
            p.drawText(10, 18, "NÁKLON –")
            return
        az_txt = f" · směr {self.az:.0f}°" if self.az is not None else ""
        p.drawText(10, 18, f"NÁKLON {self.tilt:.1f}°{az_txt} · špička {'NAHORU' if self.nose_up else 'DOLŮ'}")
        # side view
        cx, cy, L = w * 0.27, h * 0.58, min(w * 0.2, h * 0.36)
        a = math.radians(self.tilt) * (1 if (self.az is None or self.az < 180) else -1)
        dx, dy = math.sin(a), -math.cos(a)
        if not self.nose_up:
            dx, dy = -dx, -dy
        p.setPen(QtGui.QPen(C("dim"), 1, Qt.PenStyle.DashLine))
        p.drawLine(int(cx), int(cy - L * 1.1), int(cx), int(cy + L * 1.1))
        lvl = "ok" if self.tilt < 4 else "warn" if self.tilt < 10 else "bad"
        p.setPen(QtGui.QPen(C(lvl if self.tilt >= 4 else "s1"), 9, cap=Qt.PenCapStyle.RoundCap))
        p.drawLine(QtCore.QPointF(cx - dx * L, cy - dy * L), QtCore.QPointF(cx + dx * L, cy + dy * L))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C("accent"))
        p.drawEllipse(QtCore.QPointF(cx + dx * L, cy + dy * L), 6, 6)
        # bull's-eye
        bx, by, R = w * 0.72, h * 0.58, min(w * 0.22, h * 0.36)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QtGui.QPen(C("grid"), 1))
        for deg in (5, 10, 20):
            p.drawEllipse(QtCore.QPointF(bx, by), R * deg / 20, R * deg / 20)
        p.setPen(C("muted"))
        for txt, ox, oy in (("S", -4, -R - 4), ("J", -4, R + 13), ("V", R + 4, 4), ("Z", -R - 13, 4)):
            p.drawText(int(bx + ox), int(by + oy), txt)
        r = min(1.0, self.tilt / 20.0) * R
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(lvl))
        if self.az is None:
            p.setPen(QtGui.QPen(C(lvl), 3))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QtCore.QPointF(bx, by), max(r, 3), max(r, 3))
        else:
            az = math.radians(self.az)
            p.drawEllipse(QtCore.QPointF(bx + r * math.sin(az), by - r * math.cos(az)), 8, 8)


# ---------------------------------------------------------------- map
# Aerial imagery cached for offline use at the field (the Daidalos sources):
# CUZK orthophoto (0.15 m/px, Czech open data), Google outside the CZ.
TILE_SOURCES = (
    ("CUZK ortofoto", "https://ags.cuzk.cz/arcgis1/rest/services/ORTOFOTO_WM/MapServer/tile/{z}/{y}/{x}"),
    ("Google", "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"),
)
ZOOM_STRATEGY = {15: 4.0, 16: 2.5, 17: 1.5, 18: 1.0, 19: 0.6, 20: 0.35}
MAX_NATIVE_ZOOM = 20


def deg2num(lat, lon, zoom):
    n = 2.0 ** zoom
    return int((lon + 180.0) / 360.0 * n), int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)


class MapDownloader(QtCore.QThread):
    progress = QtCore.pyqtSignal(int, str)

    def __init__(self, cache_dir: str, lat: float, lon: float):
        super().__init__()
        self.cache, self.lat, self.lon = cache_dir, lat, lon

    def run(self):
        tiles = []
        for z, rkm in ZOOM_STRATEGY.items():
            dlat, dlon = rkm / 111.0, rkm / (111.0 * math.cos(math.radians(self.lat)))
            x0, y1 = deg2num(self.lat - dlat, self.lon - dlon, z)
            x1, y0 = deg2num(self.lat + dlat, self.lon + dlon, z)
            tiles += [(z, x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]
        total = max(1, len(tiles))

        def fetch(z, x, y):
            d = os.path.join(self.cache, "aerial", str(z), str(x))
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, f"{y}.png")
            if os.path.exists(path):
                return
            for _n, tmpl in TILE_SOURCES:
                try:
                    req = urllib.request.Request(tmpl.format(z=z, x=x, y=y), headers={"User-Agent": "GroundStation/2"})
                    with urllib.request.urlopen(req, timeout=6) as r:
                        data, ctype = r.read(), r.headers.get("Content-Type", "")
                    if ctype.startswith("image/") and len(data) > 1000:
                        with open(path + ".part", "wb") as fh:
                            fh.write(data)
                        os.replace(path + ".part", path)
                        return
                except Exception:
                    continue

        done = 0
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as ex:
            for _ in ex.map(lambda a: fetch(*a), tiles):
                done += 1
                if done % 20 == 0:
                    self.progress.emit(int(done / total * 100), f"mapa {done}/{total}")
        self.progress.emit(100, "mapa hotová")


LEAFLET_HTML = """<!DOCTYPE html><html><head><meta charset="utf-8">
<style>html,body,#map{height:100%;width:100%;margin:0;padding:0;background:#1e1e1e}
.ic{background:transparent;border:none}</style>
<link rel="stylesheet" href="LEAFLET_CSS"/><script src="LEAFLET_JS"></script></head>
<body><div id="map"></div><script>
var map=L.map('map',{zoomControl:false,attributionControl:false,preferCanvas:true}).setView([PAD_LAT,PAD_LON],17);
var blank='data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';
var o={maxZoom:22,maxNativeZoom:MAXZ,minZoom:12,keepBuffer:4,errorTileUrl:blank};
L.tileLayer('LIVE_URL',o).addTo(map);
L.tileLayer('CACHE_URI/aerial/{z}/{x}/{y}.png',o).addTo(map);
L.control.scale({imperial:false}).addTo(map);
var info=L.control({position:'topright'});
info.onAdd=function(){this._d=L.DomUtil.create('div');this._d.style.cssText='background:rgba(0,0,0,.78);color:#fff;font:bold 13px Consolas,monospace;padding:6px 9px;border-radius:6px';return this._d};
info.addTo(map);
var pad=[PAD_LAT,PAD_LON],last=pad,lastView=0;
var padM=L.circleMarker(pad,{radius:9,color:'#00ff88',weight:3,fillOpacity:.25}).addTo(map);
var rk=L.circleMarker(pad,{radius:8,color:'#fff',weight:3,fillColor:'#ff0000',fillOpacity:1}).addTo(map);
var tr=L.polyline([],{color:'#ff8c00',weight:4,opacity:.9}).addTo(map);
function upd(){var d=map.distance(pad,last);info._d.innerHTML='RAMPA '+pad[0].toFixed(6)+', '+pad[1].toFixed(6)+'<br>RAKETA '+last[0].toFixed(6)+', '+last[1].toFixed(6)+'<br>OD RAMPY <b>'+d.toFixed(0)+' m</b>'}
function view(){var b=L.latLngBounds([[pad[0]+.001,pad[1]+.001],[pad[0]-.001,pad[1]-.001]]);b.extend(last);map.fitBounds(b,{padding:[40,40],maxZoom:20})}
function setPad(a,b){pad=[a,b];padM.setLatLng(pad);upd();view()}
function rocket(a,b){last=[a,b];rk.setLatLng(last);tr.addLatLng(last);upd();var n=Date.now();if(n-lastView>700){view();lastView=n}}
function clearTrack(){tr.setLatLngs([])}
upd();setTimeout(view,200);
</script></body></html>"""


class LocalTrack(QtWidgets.QWidget):
    """Ground track around the pad without a browser: east/north in metres."""

    def __init__(self):
        super().__init__()
        self.pad = None
        self.pts: deque = deque(maxlen=5000)
        self.setMinimumSize(240, 200)

    def set_pad(self, lat, lon):
        self.pad = (lat, lon)
        self.update()

    def rocket(self, lat, lon):
        self.pts.append((lat, lon))
        self.update()

    def clear_track(self):
        self.pts.clear()
        self.update()

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QtGui.QPen(C("border"), 1))
        p.setBrush(C("map_bg"))
        p.drawRoundedRect(QtCore.QRectF(0.5, 0.5, w - 1, h - 1), 8, 8)
        f = p.font()
        f.setBold(True)
        f.setPixelSize(12)
        p.setFont(f)
        p.setPen(C("text"))
        if not self.pad:
            p.drawText(12, 20, "TRAŤ – čeká na polohu rampy")
            return
        lat0, lon0 = self.pad
        ky, kx = 111320.0, 111320.0 * math.cos(math.radians(lat0))
        xy = [((lo - lon0) * kx, (la - lat0) * ky) for la, lo in self.pts]
        ext = max([50.0] + [max(abs(x), abs(y)) for x, y in xy]) * 1.15
        cx, cy, sc = w / 2, h / 2 + 8, (min(w, h) / 2 - 18) / ext
        p.setPen(QtGui.QPen(C("grid"), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        step = 10 ** math.floor(math.log10(ext))
        for k in (1, 2, 3, 4, 5):
            rr = step * k
            if rr <= ext:
                p.drawEllipse(QtCore.QPointF(cx, cy), rr * sc, rr * sc)
        if len(xy) >= 2:
            path = QtGui.QPainterPath(QtCore.QPointF(cx + xy[0][0] * sc, cy - xy[0][1] * sc))
            for x, y in xy[1:]:
                path.lineTo(cx + x * sc, cy - y * sc)
            p.setPen(QtGui.QPen(C("s2"), 3))
            p.drawPath(path)
        p.setPen(QtGui.QPen(C("ok"), 3))
        p.drawEllipse(QtCore.QPointF(cx, cy), 7, 7)
        dist = 0.0
        if xy:
            x, y = xy[-1]
            dist = math.hypot(x, y)
            p.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2))
            p.setBrush(C("bad"))
            p.drawEllipse(QtCore.QPointF(cx + x * sc, cy - y * sc), 7, 7)
        p.setPen(C("text"))
        p.drawText(12, 20, f"TRAŤ (bez mapy) · od rampy {dist:.0f} m · kruhy po {step:.0f} m · nahoře sever")


def make_map(cache_dir: str, lat: float, lon: float) -> QtWidgets.QWidget:
    """Leaflet over aerial tiles when PyQt6-WebEngine is installed, else LocalTrack."""
    if os.environ.get("GROUNDSTATION_NO_WEBENGINE"):
        return LocalTrack()
    try:
        from PyQt6.QtWebEngineWidgets import QWebEngineView
        from PyQt6.QtWebEngineCore import QWebEngineSettings
    except Exception:
        return LocalTrack()

    class LeafletMap(QWebEngineView):
        def __init__(self):
            super().__init__()
            s = self.settings()
            s.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
            s.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
            os.makedirs(cache_dir, exist_ok=True)
            css, js = os.path.join(cache_dir, "leaflet.css"), os.path.join(cache_dir, "leaflet.js")
            html = (LEAFLET_HTML.replace("PAD_LAT", f"{lat:.7f}").replace("PAD_LON", f"{lon:.7f}")
                    .replace("MAXZ", str(MAX_NATIVE_ZOOM)).replace("LIVE_URL", TILE_SOURCES[0][1])
                    .replace("CACHE_URI", Path(cache_dir).as_uri())
                    .replace("LEAFLET_CSS", Path(css).as_uri() if os.path.exists(css) else "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css")
                    .replace("LEAFLET_JS", Path(js).as_uri() if os.path.exists(js) else "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"))
            self.setHtml(html, QtCore.QUrl.fromLocalFile(cache_dir + "/"))
            self.setMinimumSize(240, 200)
            self._cache_leaflet(css, js)

        @staticmethod
        def _cache_leaflet(css, js):
            """Keep a copy of Leaflet for the field (no internet at the pad)."""
            def get():
                for url, path in (("https://unpkg.com/leaflet@1.9.4/dist/leaflet.css", css),
                                  ("https://unpkg.com/leaflet@1.9.4/dist/leaflet.js", js)):
                    if not os.path.exists(path):
                        try:
                            with urllib.request.urlopen(url, timeout=8) as r:
                                data = r.read()
                            with open(path, "wb") as fh:
                                fh.write(data)
                        except Exception:
                            pass
            concurrent.futures.ThreadPoolExecutor(max_workers=1).submit(get)

        def set_pad(self, a, b):
            self.page().runJavaScript(f"setPad({a:.7f},{b:.7f});")

        def rocket(self, a, b):
            self.page().runJavaScript(f"rocket({a:.7f},{b:.7f});")

        def clear_track(self):
            self.page().runJavaScript("clearTrack();")

    return LeafletMap()


# ---------------------------------------------------------------- narrator
class Narrator(QtCore.QObject):
    """Says the countdown and the flight events (English), Qt TTS or Windows System.Speech."""

    PS = ("Add-Type -AssemblyName System.Speech;$s=New-Object System.Speech.Synthesis.SpeechSynthesizer;"
          "try{$s.SelectVoiceByHints('NotSet','NotSet',0,[Globalization.CultureInfo]'en-US')}catch{};$s.Rate=1;"
          "while(($l=[Console]::In.ReadLine()) -ne $null){if($l.StartsWith('!')){$s.SpeakAsyncCancelAll();"
          "$l=$l.Substring(1)};if($l){[void]$s.SpeakAsync($l)}}")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.enabled = False
        self.backend = "žádný"
        self._tts = None
        self._ps = None
        if os.environ.get("QT_QPA_PLATFORM") == "offscreen":
            return
        try:
            from PyQt6.QtTextToSpeech import QTextToSpeech
            engines = QTextToSpeech.availableEngines()
            eng = next((e for e in ("winrt", "sapi", "speechd", "flite", "macos") if e in engines),
                       engines[0] if engines else None)
            if eng is not None:
                tts = QTextToSpeech(eng, self)
                en = [lo for lo in tts.availableLocales() if lo.language() == QtCore.QLocale.Language.English]
                if en:
                    tts.setLocale(en[0])
                if tts.availableVoices():
                    self._tts, self.backend = tts, f"Qt {eng}"
        except Exception:
            self._tts = None
        if self._tts is None and sys.platform == "win32":
            self._ps = QtCore.QProcess(self)
            self._ps.start("powershell", ["-NoProfile", "-NonInteractive", "-Command", self.PS])
            self.backend = "Windows System.Speech"

    @property
    def available(self) -> bool:
        return self._tts is not None or self._ps is not None

    def say(self, text: str, urgent: bool = False, force: bool = False):
        if not (self.enabled or force) or not text:
            return
        if self._tts is not None:
            if urgent or not hasattr(self._tts, "enqueue"):
                self._tts.say(text)
            else:
                self._tts.enqueue(text)
        elif self._ps is not None and self._ps.state() == QtCore.QProcess.ProcessState.Running:
            self._ps.write((("!" if urgent else "") + text.replace("\n", " ") + "\n").encode())

    def close(self):
        if self._ps is not None:
            self._ps.closeWriteChannel()
            self._ps.waitForFinished(500)
            self._ps.kill()


# ---------------------------------------------------------------- app icon
def app_icon() -> QtGui.QIcon:
    here = os.path.dirname(os.path.abspath(__file__))
    for path in (os.path.join(here, "..", "tools", "windows", "groundstation.ico"),
                 os.path.join(os.environ.get("LOCALAPPDATA", ""), "GroundStation", "groundstation.ico")):
        if os.path.isfile(path):
            return QtGui.QIcon(path)
    return QtGui.QIcon()


def set_windows_app_id(app_id: str):
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(app_id)
        except Exception:
            pass
