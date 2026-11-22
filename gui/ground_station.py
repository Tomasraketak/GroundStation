#!/usr/bin/env python3
"""
GROUND STATION — one window for both rockets and the motor test stand

    ZENITH   Sapphire rocket, Zenith v2.4 flight computer
    ARIADNA  Daidalos landing rocket, Ariadna V2 avionics
    STAND    the rampa alone as a static-fire thrust stand ("TEST MOTORU"):
             live thrust / chamber pressure / temperatures, ARM + FIRE,
             results and the thrust curve of every test (gs_stand.py)

Plug the ground station (Pico 2 W + Ra-02 + OLED, firmware/Ground_Station)
into the laptop and pick the rocket at the top of the window. The choice is
sent to the station ("MODE ZENITH" / "MODE ARIADNA"), which retunes its radio
at once and remembers it — no code change, no reflashing. Holding the
station's left button for 3 s switches it too, and the window follows.

    pip install -r requirements.txt
    python ground_station.py                    # pick the port in the window
    python ground_station.py --port COM7 --rocket ARIADNA
    python ground_station.py --rocket STAND --demo   # simulated thrust stand
    python ground_station.py --demo             # no hardware: simulated station + flight

Every line from the station is written to <Documents>/GroundStation/logs.
Keys: F1/F2/F3 pages, Ctrl+Shift+A = ABORT.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gs_protocol as gp  # noqa: E402
import gs_widgets as gw  # noqa: E402
import gs_stand  # noqa: E402
from gs_widgets import Tile, Strip, CheckRow, StripChart, TiltView, badge, kbutton, panel, caption, set_level  # noqa: E402

try:
    import serial
    import serial.tools.list_ports
except ImportError:                       # the window still starts (demo) without pyserial
    serial = None

SETTINGS = ("VoltinoLabs", "GroundStation")
LAUNCH_CD_S = 30


# ============================================================================
#  Serial
# ============================================================================
class SerialReader(QtCore.QThread):
    """Reads text lines in a thread, reconnects by itself after an unplug."""
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.port, self.baud = port, baud
        self._run = True
        self._tx: list[str] = []
        self._lock = QtCore.QMutex()

    def send(self, text: str):
        with QtCore.QMutexLocker(self._lock):
            self._tx.append(text)

    def stop(self):
        self._run = False
        self.wait(1500)

    def run(self):
        if serial is None:
            self.status.emit("chybí pyserial (pip install pyserial)")
            return
        while self._run:
            ser = None
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.05)
                ser.dtr = True
                self.status.emit(f"připojeno {self.port}")
                buf = b""
                while self._run:
                    with QtCore.QMutexLocker(self._lock):
                        tx, self._tx = self._tx, []
                    for t in tx:
                        ser.write((t + "\n").encode())
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        # text lines only; a binary legacy frame (0xA5 0x1D) never has a '$' start
                        s = raw.decode("ascii", errors="ignore").strip()
                        k = max(s.rfind("$"), s.rfind("#"))
                        if k > 0:
                            s = s[k:]
                        if s:
                            self.line.emit(s)
                    if len(buf) > 8192:
                        buf = buf[-1024:]
            except Exception as ex:          # unplugged, busy, wrong port ...
                self.status.emit(f"odpojeno: {ex}")
                for _ in range(20):
                    if not self._run:
                        break
                    time.sleep(0.05)
            finally:
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass


def list_ports() -> list[str]:
    if serial is None:
        return []
    ports = list(serial.tools.list_ports.comports())
    # the Pico (2E8A) first
    ports.sort(key=lambda p: (0 if (p.vid == 0x2E8A) else 1, p.device))
    return [p.device for p in ports]


# ============================================================================
#  Main window
# ============================================================================
class GroundStation(QtWidgets.QMainWindow):
    def __init__(self, port: str | None = None, demo: bool = False, demo_speed: float = 1.0, rocket: str | None = None):
        super().__init__()
        self.settings = QtCore.QSettings(*SETTINGS)
        self.key = (rocket or str(self.settings.value("rocket", "ZENITH"))).upper()
        if self.key not in gp.PROFILES:
            self.key = "ZENITH"
        self.link: gp.RocketLink = gp.PROFILES[self.key]()
        self.station = gp.Station()
        self.reader = None
        self.demo_speed = demo_speed
        self.t_start = time.time()
        self._mode_req = None                # (mode, time) of our last MODE command
        self._rx_times: list[float] = []
        self._last_state = None
        self._ev_prev = None
        self._cd_last = None
        self._los = False
        self._ul_name = None
        self._launch_note = None
        self._pad = None
        self._pad_src = ""
        self.user_dir = gw.user_dir()
        self._open_log()

        self.setWindowTitle("Pozemní stanice – Zenith / Ariadna / test motoru")
        self.resize(1480, 900)
        self._build()
        self._apply_rocket(initial=True)
        QtCore.QTimer(self, timeout=self.refresh, interval=100).start()
        QtCore.QTimer(self, timeout=self._utc_sync, interval=5000).start()
        if demo:
            self.start_demo()
        elif port:
            self.connect_port(port)

    # ------------------------------------------------------------------ layout
    def _build(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        v = QtWidgets.QVBoxLayout(root)
        v.setContentsMargins(10, 8, 10, 4)
        v.setSpacing(6)

        # --- always visible: rocket choice, what it does, ABORT -----------------------
        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(8)
        v.addLayout(bar)
        brand = QtWidgets.QVBoxLayout()
        brand.setSpacing(0)
        b1 = QtWidgets.QLabel("POZEMNÍ STANICE")
        b1.setObjectName("Brand")
        self.brand_sub = QtWidgets.QLabel("")
        self.brand_sub.setObjectName("BrandSub")
        brand.addWidget(b1)
        brand.addWidget(self.brand_sub)
        bar.addLayout(brand)

        self.rocket_btns = {}
        grp = QtWidgets.QButtonGroup(self)
        grp.setExclusive(True)
        for key, txt, tip in (("ZENITH", "ZENITH · Sapphire", "Raketa Sapphire s letovým počítačem Zenith v2.4"),
                              ("ARIADNA", "ARIADNA · Daidalos", "Přistávací raketa Daidalos s avionikou Ariadna V2"),
                              ("STAND", "TEST MOTORU · tah", "Rampa bez rakety jako zkušební stav motoru: zážeh, tah, tlak, teploty")):
            b = kbutton(txt, "", tip + " – přepne i pozemní stanici")
            b.setObjectName("Rocket")
            b.setCheckable(True)
            b.clicked.connect(lambda _=False, k=key: self.select_rocket(k))
            grp.addButton(b)
            self.rocket_btns[key] = b
            bar.addWidget(b)
        bar.addSpacing(6)

        self.state_banner = QtWidgets.QLabel("BEZ DAT")
        self.state_banner.setObjectName("State")
        self.state_banner.setProperty("level", "idle")
        self.state_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.state_banner.setMinimumWidth(230)
        bar.addWidget(self.state_banner)
        self.clock = QtWidgets.QLabel("T  --:--")
        self.clock.setObjectName("Clock")
        self.clock.setProperty("level", "plain")
        self.clock.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.clock.setMinimumWidth(200)
        self.clock.setToolTip("Odpočet do startu (T−) podle rakety / čas letu (T+)")
        bar.addWidget(self.clock)
        col = QtWidgets.QVBoxLayout()
        col.setSpacing(3)
        self.link_badge = badge("LoRa –")
        self.link_badge.setMinimumWidth(190)
        self.win_badge = badge("RX okno –")
        self.win_badge.setToolTip("Raketa poslouchá 300 ms každých 10 s (+ poslední okno pro ABORT v T−1 s)")
        col.addWidget(self.link_badge)
        col.addWidget(self.win_badge)
        bar.addLayout(col)
        bar.addStretch(1)
        self.abort_btn = kbutton("✖  ABORT", "abort", "Okamžitě zařadí ABORT do nejbližšího RX okna rakety (Ctrl+Shift+A)")
        self.abort_btn.clicked.connect(self.send_abort)
        bar.addWidget(self.abort_btn)

        # --- connection row ------------------------------------------------------------
        con = QtWidgets.QHBoxLayout()
        con.setSpacing(6)
        v.addLayout(con)
        con.addWidget(caption("Stanice"))
        self.port_cb = QtWidgets.QComboBox()
        self.port_cb.setMinimumWidth(150)
        self.port_cb.addItems(list_ports())
        bt = kbutton("⟳", "", "Znovu načíst porty")
        bt.clicked.connect(self.refresh_ports)
        self.con_btn = kbutton("PŘIPOJIT", "primary", "Připojit / odpojit pozemní stanici (USB)")
        self.con_btn.clicked.connect(self.toggle_connection)
        bd = kbutton("DEMO", "", "Simulovaná stanice a let – bez hardwaru")
        bd.clicked.connect(self.start_demo)
        self.conn_lb = QtWidgets.QLabel("nepřipojeno")
        self.conn_lb.setStyleSheet("font-weight: 700; background: transparent;")
        self.station_lb = badge("stanice –")
        self.station_lb.setToolTip("Režim, ve kterém stanice právě poslouchá, a stav jejího rádia")
        for w in (self.port_cb, bt, self.con_btn, bd, self.conn_lb):
            con.addWidget(w)
        con.addStretch(1)
        con.addWidget(self.station_lb)
        self.narr_btn = kbutton("🔊 HLAS", "", "Narátor (anglicky): odpočet 10, 5, 3, start a události letu")
        self.narr_btn.setCheckable(True)
        self.narr_btn.clicked.connect(self.toggle_narrator)
        self.theme_btn = kbutton("", "", "Vzhled: VENKU (světlý, na slunce) / NOC (tmavý)")
        self.theme_btn.clicked.connect(self.toggle_theme)
        con.addWidget(self.narr_btn)
        con.addWidget(self.theme_btn)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self.tabs)
        v.addWidget(self.stack, 1)
        self._build_flight()
        self._build_start()
        self._build_diag()
        self.stand = gs_stand.StandPanel(self)       # MODE STAND replaces the rocket tabs
        self.stack.addWidget(self.stand)

        self.narrator = gw.Narrator(self)
        if not self.narrator.available:
            self.narr_btn.setEnabled(False)
            self.narr_btn.setToolTip("Hlasový výstup není k dispozici")
        elif str(self.settings.value("narrator", "false")).lower() == "true":
            self.narr_btn.setChecked(True)
            self.narrator.enabled = True
        self._theme = str(self.settings.value("theme", "light"))
        self.theme_btn.setText(gw.THEME_LABEL.get(self._theme, "☀ VENKU"))

        for i, k in enumerate((Qt.Key.Key_F1, Qt.Key.Key_F2, Qt.Key.Key_F3)):
            QtGui.QShortcut(QtGui.QKeySequence(k), self).activated.connect(lambda i=i: self.tabs.setCurrentIndex(i))
        QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Shift+A"), self).activated.connect(self.send_abort)
        self.status = self.statusBar()

    def _page(self, title):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(w)
        lay.setContentsMargins(0, 8, 0, 0)
        lay.setSpacing(10)
        self.tabs.addTab(w, f"  {title}  ")
        return lay

    def _build_flight(self):
        l1 = self._page("1 · LET")
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(7)
        l1.addLayout(left, 24)
        self.ft = {}
        for key, title, hero in (("alt", "Výška nad rampou", True), ("vz", "Svislá rychlost", True),
                                 ("max", "Max. výška", False), ("x1", "", False), ("acc", "Zrychlení", False),
                                 ("dist", "Od rampy", False)):
            t = Tile(title, hero)
            self.ft[key] = t
            left.addWidget(t)
        left.addStretch(1)
        mid = QtWidgets.QVBoxLayout()
        mid.setSpacing(7)
        l1.addLayout(mid, 40)
        self.ch_alt = StripChart("Výška", [("výška", "s1"), ("max", "dim")], 120, "m", floor=0)
        self.ch_vz = StripChart("Svislá rychlost", [("vz", "s2")], 120, "m/s")
        mid.addWidget(self.ch_alt, 1)
        mid.addWidget(self.ch_vz, 1)
        self.flight_strip = Strip("")
        mid.addWidget(self.flight_strip)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(7)
        l1.addLayout(right, 36)
        pad = self._load_pad()
        self.map = gw.make_map(os.path.join(self.user_dir, "map_cache"), *pad)
        right.addWidget(self.map, 3)
        self.tilt = TiltView()
        right.addWidget(self.tilt, 2)

    def _build_start(self):
        l2 = self._page("2 · START")
        chk, cl = panel("Kontrola před startem (z telemetrie)")
        l2.addWidget(chk, 52)
        self.go_banner = QtWidgets.QLabel("BEZ DAT")
        self.go_banner.setObjectName("State")
        self.go_banner.setProperty("level", "idle")
        self.go_banner.setWordWrap(True)
        self.go_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cl.addWidget(self.go_banner)
        self.check_rows: list[CheckRow] = []
        self.check_box = QtWidgets.QVBoxLayout()
        self.check_box.setSpacing(5)
        cl.addLayout(self.check_box)
        cl.addStretch(1)
        self.rampa_lb = badge("RAMPA –")
        self.rampa_lb.setWordWrap(True)
        cl.addWidget(caption("Rampa"))
        cl.addWidget(self.rampa_lb)
        note = QtWidgets.QLabel("Orientační kontrola. O startu rozhoduje vedoucí startu.")
        note.setObjectName("Caption")
        cl.addWidget(note)

        rcol = QtWidgets.QVBoxLayout()
        rcol.setSpacing(8)
        l2.addLayout(rcol, 48)
        proc, pl = panel("Předstartovní procedura")
        rcol.addWidget(proc)
        self.proc = {}
        steps = (("link", "Spojení s raketou čerstvé (paket < 2 s)", False, None),
                 ("go", "Raketa GO, kontrola bez NO-GO", False, None),
                 ("prep", "PŘÍPRAVA odeslána a potvrzena raketou", False, ("PŘÍPRAVA", self.send_prepare)),
                 ("rampa", "Rampa READY (klíč, rozbuška)", False, None),
                 ("area", "Prostor vyklizen, všichni v bezpečné vzdálenosti", True, None),
                 ("rbf", "RBF vytažen, klíč rampy zapnut", True, None),
                 ("lead", "Vedoucí startu dal souhlas", True, None))
        for key, text, manual, btn in steps:
            row = QtWidgets.QHBoxLayout()
            lamp = QtWidgets.QLabel("—")
            lamp.setObjectName("Lamp")
            lamp.setProperty("level", "idle")
            lamp.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lamp.setFixedSize(54, 24)
            row.addWidget(lamp)
            if manual:
                w = QtWidgets.QCheckBox(text)
                w.toggled.connect(self._update_procedure)
            else:
                w = QtWidgets.QLabel(text)
                w.setStyleSheet("font-weight: 700; background: transparent;")
            row.addWidget(w, 1)
            if btn:
                b = kbutton(btn[0], "go", "Raketa si změří rampu (tlak, GPS), rampa zkontroluje klíč a rozbušku")
                b.clicked.connect(btn[1])
                b.setFixedWidth(110)
                row.addWidget(b)
            pl.addLayout(row)
            self.proc[key] = (lamp, w if manual else None)
        cdrow = QtWidgets.QHBoxLayout()
        cdrow.addWidget(QtWidgets.QLabel("Odpočet"))
        self.cd_spin = QtWidgets.QSpinBox()
        self.cd_spin.setSuffix(" s")
        self.cd_spin.setValue(LAUNCH_CD_S)
        cdrow.addWidget(self.cd_spin)
        self.ch_lb = QtWidgets.QLabel("rozbuška rampy")
        self.ch_cb = QtWidgets.QComboBox()
        self.ch_cb.addItems(["1", "2", "3"])
        cdrow.addWidget(self.ch_lb)
        cdrow.addWidget(self.ch_cb)
        cdrow.addStretch(1)
        pl.addLayout(cdrow)
        self.launch_btn = kbutton("▶  START ODPOČTU", "warn", "Pošle raketě synchronizovaný odpočet; rampa zažehne v T-0. "
                                                            "Potvrzuje se napsáním START.")
        self.launch_btn.setMinimumHeight(46)
        self.launch_btn.setStyleSheet("font-size: 17px;")
        self.launch_btn.clicked.connect(self.send_launch)
        pl.addWidget(self.launch_btn)
        self.launch_lb = badge("Procedura nezahájena")
        self.launch_lb.setWordWrap(True)
        pl.addWidget(self.launch_lb)

        up, ul = panel("Uplink · příkazy raketě (1 příkaz za 10 s okno)")
        rcol.addWidget(up, 1)
        self.ul_lb = badge("žádný příkaz")
        self.ul_lb.setWordWrap(True)
        ul.addWidget(self.ul_lb)
        self.cmd_grid = QtWidgets.QGridLayout()
        self.cmd_grid.setSpacing(6)
        ul.addLayout(self.cmd_grid)
        self.main_row = QtWidgets.QWidget()
        self.main_row.setObjectName("Plain")
        mr = QtWidgets.QHBoxLayout(self.main_row)
        mr.setContentsMargins(0, 0, 0, 0)
        self.main_spin = QtWidgets.QSpinBox()
        self.main_spin.setRange(100, 1000)
        self.main_spin.setSingleStep(25)
        self.main_spin.setValue(300)
        self.main_spin.setSuffix(" m")
        bm = kbutton("Nastavit výšku hlavního padáku", "", "Zenith: výška vypuštění main (100–1000 m)")
        bm.clicked.connect(lambda: self.cmd(f"MAINALT {self.main_spin.value()}"))
        mr.addWidget(self.main_spin)
        mr.addWidget(bm, 1)
        ul.addWidget(self.main_row)
        bc = kbutton("Zrušit čekající příkaz", "", "Odebere příkaz, který ještě čeká na RX okno")
        bc.clicked.connect(lambda: self.send("CANCEL"))
        ul.addWidget(bc)
        ul.addStretch(1)

    def _build_diag(self):
        l3 = self._page("3 · DIAGNOSTIKA")
        c1 = QtWidgets.QVBoxLayout()
        c1.setSpacing(7)
        l3.addLayout(c1, 55)
        self.diag_grid = QtWidgets.QGridLayout()
        self.diag_grid.setSpacing(7)
        c1.addLayout(self.diag_grid)
        self.diag_tiles: dict[str, Tile] = {}
        self.diag_strips = [Strip(""), Strip(""), Strip("")]
        for s in self.diag_strips:
            c1.addWidget(s)
        self.ch_acc = StripChart("Zrychlení", [("acc", "s3")], 120, "g")
        self.ch_rssi = StripChart("Signál", [("rssi", "s1"), ("snr×5", "s4")], 300, "dBm")
        c1.addWidget(self.ch_acc, 1)
        c1.addWidget(self.ch_rssi, 1)

        c2 = QtWidgets.QVBoxLayout()
        c2.setSpacing(7)
        l3.addLayout(c2, 45)
        st, sl = panel("Pozemní stanice")
        c2.addWidget(st)
        self.station_info = QtWidgets.QLabel("–")
        self.station_info.setWordWrap(True)
        self.station_info.setStyleSheet("font-family: Consolas, monospace; background: transparent;")
        sl.addWidget(self.station_info)
        row = QtWidgets.QHBoxLayout()
        bt = kbutton("TEST ze stanice", "", "Stanice sama generuje telemetrii zvolené rakety (zkouška USB a GUI)")
        bt.setCheckable(True)
        bt.toggled.connect(lambda on: self.send(f"TEST {1 if on else 0}"))
        self.test_btn = bt
        bh = kbutton("HELLO", "", "Zeptat se stanice na verzi a režim")
        bh.clicked.connect(lambda: self.send("HELLO"))
        self.map_btn = kbutton("MAPA ⭳ okolí rampy", "", "Stáhnout letecké snímky okolí rampy pro offline použití")
        self.map_btn.clicked.connect(self.download_map)
        bl = kbutton("Složka s logy", "", self.user_dir)
        bl.clicked.connect(lambda: QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(self.user_dir)))
        for b in (bt, bh, self.map_btn, bl):
            row.addWidget(b)
        sl.addLayout(row)
        lg, ll = panel("Události a odpovědi stanice")
        c2.addWidget(lg, 1)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        ll.addWidget(self.log)
        rw, rl = panel("Poslední řádek telemetrie")
        c2.addWidget(rw)
        self.raw_lb = QtWidgets.QLabel("–")
        self.raw_lb.setWordWrap(True)
        self.raw_lb.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.raw_lb.setStyleSheet("font-family: Consolas, monospace; font-size: 11px; background: transparent;")
        rl.addWidget(self.raw_lb)

    # ------------------------------------------------------------------ rocket choice
    def select_rocket(self, key: str, send: bool = True):
        key = key.upper()
        if key not in gp.PROFILES:
            return
        if send:
            self._mode_req = (key, time.time())
            self.send("MODE " + key, quiet=True)
        if key == self.key:
            self.rocket_btns[key].setChecked(True)
            return
        self.key = key
        self.link = gp.PROFILES[key]()
        self.settings.setValue("rocket", key)
        self._apply_rocket()
        self.note(f"RAKETA: {key} ({self.link.SUBTITLE})")

    def _apply_rocket(self, initial: bool = False):
        L = self.link
        self.rocket_btns[self.key].setChecked(True)
        self.brand_sub.setText(f"{L.TITLE} · {L.SUBTITLE}")
        self.setWindowTitle(f"Pozemní stanice – {L.TITLE} ({L.SUBTITLE})")
        self.cd_spin.setRange(L.CD_MIN, L.CD_MAX)
        self.ch_lb.setVisible(L.PYRO_CHANNEL_SELECT)
        self.ch_cb.setVisible(L.PYRO_CHANNEL_SELECT)
        self.main_row.setVisible(self.key == "ZENITH")
        while self.cmd_grid.count():
            w = self.cmd_grid.takeAt(0).widget()
            if w:
                w.hide()
                w.setParent(None)
                w.deleteLater()
        for i, (txt, c, question, kind) in enumerate(L.COMMANDS):
            b = kbutton(txt, kind)
            b.clicked.connect(lambda _=False, c=c, q=question: self.confirm(q, c) if q else self.cmd(c))
            self.cmd_grid.addWidget(b, i // 2, i % 2)
        for t in self.diag_tiles.values():           # the other rocket's tiles
            self.diag_grid.removeWidget(t)
            t.hide()
            t.setParent(None)
            t.deleteLater()
        self.diag_tiles = {}
        for ch in (self.ch_alt, self.ch_vz, self.ch_acc, self.ch_rssi):
            ch.clear()
        if hasattr(self.map, "clear_track"):
            self.map.clear_track()
        self._last_state = self._ev_prev = self._cd_last = None
        self._ul_name = None
        self._launch_note = None
        self._rx_times.clear()
        self._reset_procedure()
        pad = self._load_pad()
        self._pad = pad
        if hasattr(self.map, "set_pad"):
            QtCore.QTimer.singleShot(400, lambda p=pad: self.map.set_pad(*p))
        self.state_banner.setText("BEZ DAT")
        set_level(self.state_banner, "idle")
        stand = self.key == "STAND"
        self.stack.setCurrentIndex(1 if stand else 0)
        self.stand.reset()
        self.win_badge.setToolTip("Test motoru: povely jdou hned po telemetrii rampy (bez RX oken)" if stand else
                                  "Raketa poslouchá 300 ms každých 10 s (+ poslední okno pro ABORT v T−1 s)")
        self.abort_btn.setToolTip("Okamžitě pošle ABORT rampě (Ctrl+Shift+A)" if stand else
                                  "Okamžitě zařadí ABORT do nejbližšího RX okna rakety (Ctrl+Shift+A)")
        if not initial:
            self._open_log()

    # ------------------------------------------------------------------ connection
    def refresh_ports(self):
        cur = self.port_cb.currentText()
        self.port_cb.clear()
        self.port_cb.addItems(list_ports())
        if cur:
            self.port_cb.setCurrentText(cur)

    def toggle_connection(self):
        if self.reader is not None:
            self.stop_reader()
            self.conn_lb.setText("odpojeno")
            self.con_btn.setText("PŘIPOJIT")
            return
        self.connect_port(self.port_cb.currentText())

    def connect_port(self, port: str):
        if not port:
            self.note("žádný sériový port – připoj stanici USB kabelem a stiskni ⟳")
            return
        self.stop_reader()
        self.reader = SerialReader(port)
        self.reader.line.connect(self.on_line)
        self.reader.status.connect(self._on_status)
        self.reader.start()
        self.con_btn.setText("ODPOJIT")
        self.settings.setValue("port", port)

    def _on_status(self, s: str):
        self.conn_lb.setText(s)
        if s.startswith("připojeno"):
            # a fresh connection: who are you, and listen for MY rocket
            QtCore.QTimer.singleShot(300, lambda: self.send("HELLO", quiet=True))
            QtCore.QTimer.singleShot(500, lambda: self.select_rocket(self.key))
            QtCore.QTimer.singleShot(700, self._utc_sync)

    def start_demo(self):
        from gs_demo import DemoStation
        self.stop_reader()
        self.reader = DemoStation(self.key, self.demo_speed)
        self.reader.line.connect(self.on_line)
        self.conn_lb.setText("DEMO – simulovaná stanice a let")
        self.con_btn.setText("ODPOJIT")
        self.send("HELLO", quiet=True)

    def stop_reader(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.station = gp.Station()

    def send(self, text: str, quiet: bool = False) -> bool:
        if self.reader is None:
            if not quiet:
                self.note("NEPŘIPOJENO – příkaz nebyl odeslán")
                self.status.showMessage("Pozemní stanice není připojená", 4000)
            return False
        self.reader.send(text)
        return True

    def _utc_sync(self):
        if self.reader is not None:
            self.send(f"UTC {int((time.time() % 86400.0) * 1000)}", quiet=True)

    # ------------------------------------------------------------------ commands
    def cmd(self, c: str) -> bool:
        if self.send("CMD " + c):
            self.note(f"→ {c} (čeká na RX okno)")
            return True
        return False

    def confirm(self, question: str, c: str):
        if QtWidgets.QMessageBox.question(self, "Potvrdit", question) == QtWidgets.QMessageBox.StandardButton.Yes:
            self.cmd(c)

    def send_abort(self):
        self.narrator.say("Abort", urgent=True)
        if self.cmd("ABORT"):
            self._launch_note = "abort"
            self.launch_lb.setText("ABORT odeslán – čeká na RX okno")
            set_level(self.launch_lb, "bad")
            self._reset_procedure()

    def send_prepare(self):
        q = ("Přepnout raketu do přípravy ke startu?\n\n• raketa si změří rampu (tlak, GPS, vyrovnání)\n"
             "• rampa zkontroluje klíč, rozbušku a baterii")
        if QtWidgets.QMessageBox.question(self, "PŘÍPRAVA", q) == QtWidgets.QMessageBox.StandardButton.Yes:
            self.cmd("PREPARE")

    def send_launch(self):
        n = self.cd_spin.value()
        ch = int(self.ch_cb.currentText())
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Potvrzení startu")
        v = QtWidgets.QVBoxLayout(dlg)
        head = QtWidgets.QLabel(f"{self.link.TITLE}: START ZA {n} s")
        head.setObjectName("State")
        head.setProperty("level", "warn")
        head.setAlignment(Qt.AlignmentFlag.AlignCenter)
        v.addWidget(head)
        extra = f"• Rampa zapálí rozbušku {ch}.\n" if self.link.PYRO_CHANNEL_SELECT else ""
        info = QtWidgets.QLabel(f"Raketě se odešle synchronizovaný odpočet T-{n} s.\n"
                                "• Raketa ho potvrdí v nejbližším RX okně (do ~10 s) a předá čas rampě.\n"
                                "• V T-1 s je poslední nouzové okno pro ABORT.\n" + extra +
                                "• V T-0 rampa zažehne motor.\n\nPro potvrzení napiš velkými písmeny START:")
        info.setStyleSheet("font-size: 14px; font-weight: 600;")
        v.addWidget(info)
        edit = QtWidgets.QLineEdit()
        edit.setStyleSheet("font-size: 20px; font-weight: 800;")
        v.addWidget(edit)
        row = QtWidgets.QHBoxLayout()
        cancel = kbutton("ZRUŠIT")
        go = kbutton(f"ODESLAT START ZA {n} s", "warn")
        go.setEnabled(False)
        edit.textChanged.connect(lambda t: go.setEnabled(t.strip() == "START"))
        cancel.clicked.connect(dlg.reject)
        go.clicked.connect(dlg.accept)
        row.addWidget(cancel)
        row.addWidget(go, 1)
        v.addLayout(row)
        dlg.setMinimumWidth(480)
        if dlg.exec() != QtWidgets.QDialog.DialogCode.Accepted:
            return
        if self.cmd(self.link.countdown_cmd(n, ch)):
            self._launch_note = "sent"
            self.launch_lb.setText("ODESLÁNO – čeká na potvrzení od rakety (nejbližší RX okno)…")
            set_level(self.launch_lb, "warn")

    # ------------------------------------------------------------------ incoming
    def on_line(self, s: str):
        now = time.time()
        self._log_line(now, s)
        p = s.split(",")
        tag = p[0]
        if tag in ("$G", "$MODE", "$ID"):
            self.station.feed(tag, p, now)
            if tag == "$ID":
                self.note(s)
            if tag in ("$MODE", "$ID") and self.station.mode and self.station.mode != self.key:
                req = self._mode_req
                if req and now - req[1] < 3.0:
                    return                                  # our MODE is on its way
                self.note(f"stanice přepnuta na {self.station.mode} (tlačítkem) – okno ji následuje")
                self.select_rocket(self.station.mode, send=False)
            elif tag == "$MODE" and self._mode_req and self.station.mode == self._mode_req[0]:
                self._mode_req = None
            return
        if tag == "$U":
            r = gp.parse_uplink(p, self.link)
            if r:
                txt, level, what = r
                self.ul_lb.setText(txt)
                set_level(self.ul_lb, level)
                self.note(txt)
                self._on_uplink(p[2], what, gp.to_int(p[5]))
                if self.key == "STAND":
                    self.stand.on_uplink(p[2], what, gp.to_int(p[5]))
            return
        if tag.startswith("#"):
            self.note(s)
            if s.startswith("#ERR"):
                self.status.showMessage(s, 6000)
            return
        if self.link.feed(tag, p, now):
            self.raw_lb.setText(s)
            self._rx_times.append(now)
            del self._rx_times[:-60]
            snap = self.link.snapshot(now)
            tt = now - self.t_start
            self.ch_alt.add(tt, **{"výška": snap.alt, "max": snap.max_alt})
            self.ch_vz.add(tt, vz=snap.vz)
            self.ch_acc.add(tt, acc=snap.acc_g)
            self.ch_rssi.add(tt, rssi=snap.rssi, **{"snr×5": (snap.snr or 0) * 5})
            self._on_telemetry(snap)
            if self.key == "STAND":
                self.stand.on_telemetry(now)
        elif tag in ("$B", "$AS"):
            self.raw_lb.setText(s)
        elif tag == "$SR" and self.key == "STAND":
            self.stand.on_result()
        elif tag == "$SC" and self.key == "STAND":
            self.stand.on_curve()

    def _on_uplink(self, name: str, what: str, rc: int):
        if what == "QUEUED":
            self._ul_name = name
        if name == "COUNTDOWN" and what == "ACK":
            ok = rc == 1
            self._launch_note = "ok" if ok else "rejected"
            self.launch_lb.setText("✔ POTVRZENO RAKETOU – odpočet běží" if ok
                                   else f"✖ RAKETA ODMÍTLA: {self.link.ack_text(rc)}")
            set_level(self.launch_lb, "ok" if ok else "bad")
            self.narrator.say("Countdown confirmed" if ok else "Countdown rejected", urgent=not ok)
        elif name == "COUNTDOWN" and what in ("FAILED", "CANCELLED"):
            self._launch_note = "timeout"
            self.launch_lb.setText("⚠ Odpočet raketa nepotvrdila – zkontroluj spojení, případně ABORT")
            set_level(self.launch_lb, "bad")
        elif name == "ABORT" and what == "ACK":
            self.narrator.say("Abort confirmed", urgent=True)

    def _on_telemetry(self, s: gp.Snap):
        L = self.link
        # a new attempt clears the old track
        if s.on_pad and self._last_state is not None and self._last_state not in L.PAD:
            for ch in (self.ch_alt, self.ch_vz, self.ch_acc):
                ch.clear()
            if hasattr(self.map, "clear_track"):
                self.map.clear_track()
            L.max_alt = 0.0
        if self._last_state is not None and s.state != self._last_state:
            self.note(f"STAV: {s.state_name} – {s.state_cz}")
            phrase = L.STATE_SPEECH.get(s.state)
            if phrase:
                if s.state == 6 and L.KEY == "ZENITH" and s.max_alt:
                    phrase = f"Apogee. {s.max_alt:.0f} meters. Drogue"
                self.narrator.say(phrase, urgent=s.state in (L.LIFTOFF_STATE, L.ABORT_STATE))
            if s.state == L.LIFTOFF_STATE:
                self.tabs.setCurrentIndex(0)
                self._reset_procedure()
        self._last_state = s.state
        if s.event_names:
            if self._ev_prev is not None:
                new = s.events & ~self._ev_prev
                for i, n in enumerate(s.event_names):
                    if new & (1 << i):
                        self.note(f"UDÁLOST: {n}")
                        ph = L.EVENT_SPEECH.get(n, n)
                        if n == "APOGEE" and s.max_alt:
                            ph = f"Apogee. {s.max_alt:.0f} meters"
                        self.narrator.say(ph)
            self._ev_prev = s.events
        if s.lat and s.lon and abs(s.lat) > 0.01:
            if hasattr(self.map, "rocket"):
                self.map.rocket(s.lat, s.lon)
        if s.pad_hint:
            self._track_pad(s.pad_hint)

    # ------------------------------------------------------------------ refresh 10 Hz
    def refresh(self):
        now = time.time()
        s = self.link.snapshot(now)
        st = self.station
        # --- header
        self.state_banner.setText(s.state_name)
        set_level(self.state_banner, s.state_level)
        self.state_banner.setToolTip(s.state_cz)
        if s.t_rem is not None:
            self._narrate_countdown(s.t_rem)
            sign = "−" if s.t_rem >= 0 else "+"
            a = abs(s.t_rem)
            self.clock.setText(f"T{sign}{int(a // 60):02d}:{a % 60:04.1f}")
            set_level(self.clock, "warn" if s.t_rem > 10 else "bad")
        elif s.t_flight is not None:
            self._cd_last = None
            self.clock.setText(f"T+{s.t_flight:5.1f} s")
            set_level(self.clock, "ok")
        else:
            self._cd_last = None
            self.clock.setText("T  --:--")
            set_level(self.clock, "plain")
        connected = self.reader is not None
        if not connected:
            self.link_badge.setText("STANICE NEPŘIPOJENA")
            set_level(self.link_badge, "idle")
        elif s.age > 1e8:
            self.link_badge.setText(f"čekám na {self.link.TITLE}…")
            set_level(self.link_badge, "warn")
        else:
            recent = [t for t in self._rx_times if now - t < 5.0]
            hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) >= 3 and recent[-1] > recent[0] else 0.0
            rs = f"{s.rssi:.0f} dBm" if s.rssi is not None else ""
            if s.age > 3:
                self.link_badge.setText(f"ZTRÁTA SIGNÁLU {s.age:.0f} s")
                set_level(self.link_badge, "bad")
            else:
                self.link_badge.setText(f"LoRa {hz:.1f} Hz · {rs}")
                set_level(self.link_badge, "warn" if s.age > 1.5 or (s.rssi or 0) < -112 else "okline")
            lost = s.age > 3 and s.have
            if lost != self._los:
                self.narrator.say("Signal lost" if lost else "Signal regained", urgent=lost)
                self._los = lost
        if self.key == "STAND":
            self.win_badge.setText("povely ihned" + (" · příkaz čeká na potvrzení" if st.pending else ""))
            set_level(self.win_badge, "warn" if st.pending else "info")
            self.stand.refresh()
        elif s.win_in is not None and s.age < 30:
            w = s.win_in
            if s.win_final and w > -0.35:
                self.win_badge.setText(f"⚠ T−1 s ABORT OKNO za {max(0.0, w):.1f} s")
                set_level(self.win_badge, "bad")
            else:
                while w < -0.35:
                    w += 10.0
                pend = " · příkaz čeká" if st.pending else ""
                self.win_badge.setText(("RX OKNO OTEVŘENO" if w <= 0 else f"RX okno za {w:4.1f} s") + pend)
                set_level(self.win_badge, "ok" if w <= 0 else "warn" if st.pending else "info")
        else:
            self.win_badge.setText("RX okno –" + (" · příkaz čeká" if st.pending else ""))
            set_level(self.win_badge, "warn" if st.pending else "idle")
        if connected:
            if st.mode is None:
                self.station_lb.setText("stanice: čekám na odpověď")
                set_level(self.station_lb, "warn")
            elif st.mode != self.key:
                self.station_lb.setText(f"stanice v režimu {st.mode} – přepínám…")
                set_level(self.station_lb, "bad")
            elif not st.lora_ok:
                self.station_lb.setText("stanice: LoRa NEFUNGUJE")
                set_level(self.station_lb, "bad")
            else:
                self.station_lb.setText(f"stanice: {st.mode} ✔")
                set_level(self.station_lb, "okline")
        else:
            self.station_lb.setText("stanice –")
            set_level(self.station_lb, "idle")
        self.station_info.setText(
            f"režim {st.mode or '?'} · fw {st.fw or '?'} · LoRa {'OK' if st.lora_ok else 'CHYBA'}\n"
            f"přijato {st.rx_ok} · vadných {st.rx_bad} · okno {'zamčeno' if st.win_locked else '–'}"
            f"{' (T-1 s)' if st.win_final else ''} · UTC z PC {'ano' if st.utc_valid else 'ne'}\n"
            f"čekající příkaz: {'ANO' if st.pending else 'ne'}")

        # --- page 1
        def f(v, fmt):
            return "--" if v is None or v != v else fmt.format(v)
        self.ft["alt"].set(f(s.alt, "{:.1f} m"), s.state_cz or " ", "ok" if s.have else "idle")
        vz = s.vz
        self.ft["vz"].set(f(vz, "{:+.1f} m/s"), "↑ stoupá" if (vz or 0) > 0.5 else "↓ klesá" if (vz or 0) < -0.5 else "—",
                          "ok" if s.have else "idle")
        self.ft["max"].set(f(s.max_alt, "{:.0f} m"), " ", "info" if s.have else "idle")
        self.ft["acc"].set(f(s.acc_g, "{:.2f} g"), " ", "warn" if (s.acc_g or 0) > 15 else "ok" if s.have else "idle")
        if s.tiles:
            first = next((t for t in s.tiles if t[0] in ("next", "mach")), s.tiles[0])
            _k, title, val, sub, lvl = first
            self.ft["x1"].set(val, sub, lvl, title)
        else:
            self.ft["x1"].set("--", " ", "idle", "Mach" if self.key == "ZENITH" else "Další událost")
        if s.lat and self._pad:
            d = gp.haversine_m(self._pad[0], self._pad[1], s.lat, s.lon)
            self.ft["dist"].set(f"{d:.0f} m", f"směr {gp.bearing_deg(self._pad[0], self._pad[1], s.lat, s.lon):.0f}°"
                                f" · {s.sats or 0} sat", "ok")
        else:
            self.ft["dist"].set("--", " ", "idle")
        if s.strips:
            title, items = s.strips[0]
            self.flight_strip.set(title, items, cols=5)
        self.flight_strip.setVisible(bool(s.strips))
        self.tilt.set(s.tilt, s.tilt_az, s.nose_up)

        # --- page 2
        while len(self.check_rows) < len(s.checks):
            r = CheckRow()
            self.check_rows.append(r)
            self.check_box.addWidget(r)
        for i, r in enumerate(self.check_rows):
            if i < len(s.checks):
                c = s.checks[i]
                r.set(c.name, c.level, c.detail)
                r.show()
            else:
                r.hide()
        if s.rampa_alone:
            self.go_banner.setText("RAMPA BEZ RAKETY – raketa není na pogu")
            set_level(self.go_banner, "info")
        elif not s.have:
            self.go_banner.setText("BEZ DAT" if connected else "STANICE NEPŘIPOJENA")
            set_level(self.go_banner, "idle")
        elif s.on_pad:
            self.go_banner.setText(s.go_text)
            set_level(self.go_banner, "ok" if s.go else "bad")
        else:
            self.go_banner.setText(f"{s.state_name} – kontrola jen na rampě" + (f" · {s.go_text}" if s.go_text else ""))
            set_level(self.go_banner, "bad" if s.state == self.link.ABORT_STATE else "idle")
        self.rampa_lb.setText(s.rampa_text)
        set_level(self.rampa_lb, {"ok": "okline"}.get(s.rampa_level, s.rampa_level))
        self._update_procedure(snap=s)

        # --- page 3
        for key, title, val, sub, lvl in s.tiles:
            t = self.diag_tiles.get(key)
            if t is None:
                n = len(self.diag_tiles)
                t = Tile(title)
                self.diag_tiles[key] = t
                self.diag_grid.addWidget(t, n // 4, n % 4)
            t.set(val, sub, lvl, title)
        for i, strip in enumerate(self.diag_strips):
            if i < len(s.strips):
                title, items = s.strips[i]
                strip.set(title, items, cols=5)
                strip.show()
            else:
                strip.hide()

    def _update_procedure(self, *_a, snap: gp.Snap | None = None):
        s = snap or self.link.snapshot(time.time())
        auto = {"link": s.have and s.age < 2.0, "go": bool(s.go), "prep": s.prepared, "rampa": s.rampa_ready}
        if self._launch_note in ("abort", "rejected", "timeout") and any(
                chk is not None and chk.isChecked() for _l, chk in self.proc.values()):
            self._launch_note = None
        all_ok = True
        for key, (lamp, chk) in self.proc.items():
            ok = chk.isChecked() if chk is not None else auto.get(key, False)
            all_ok &= ok
            lamp.setText("OK" if ok else "—")
            set_level(lamp, "ok" if ok else "idle")
        counting = s.t_rem is not None and s.t_rem > 0
        self.launch_btn.setEnabled(all_ok and not counting and not self.station.pending)
        if self._launch_note is None:
            self.launch_lb.setText("Vše splněno – můžeš odeslat start" if all_ok else "Čeká se na splnění všech kroků")
            set_level(self.launch_lb, "okline" if all_ok else "idle")

    def _reset_procedure(self):
        for _lamp, chk in getattr(self, "proc", {}).values():
            if chk is not None:
                chk.blockSignals(True)
                chk.setChecked(False)
                chk.blockSignals(False)

    # ------------------------------------------------------------------ narrator
    def toggle_narrator(self):
        on = self.narr_btn.isChecked()
        self.narrator.enabled = on
        self.settings.setValue("narrator", on)
        if on:
            self.narrator.say("Narrator on", urgent=True)

    def _narrate_countdown(self, left: float):
        prev, self._cd_last = self._cd_last, left
        if prev is None:
            return
        for th, phrase in ((10, "T minus ten"), (5, "five"), (3, "three"), (0, "Start!")):
            if prev > th + 0.25 >= left:
                self.narrator.say(phrase, urgent=True)

    # ------------------------------------------------------------------ look
    def toggle_theme(self):
        self._theme = "dark" if self._theme == "light" else "light"
        self.settings.setValue("theme", self._theme)
        gw.apply_theme(QtWidgets.QApplication.instance(), self._theme)
        self.theme_btn.setText(gw.THEME_LABEL[self._theme])
        for w in self.findChildren(QtWidgets.QWidget):
            w.update()

    # ------------------------------------------------------------------ pad, map
    def _pad_file(self):
        return os.path.join(self.user_dir, f"pad_{self.key.lower()}.json")

    def _load_pad(self) -> tuple[float, float]:
        try:
            with open(self._pad_file(), encoding="utf-8") as fh:
                d = json.load(fh)
            return float(d["lat"]), float(d["lon"])
        except (OSError, ValueError, KeyError):
            return 49.7985411, 16.6912772          # Czech Rocket Challenge pad

    def _track_pad(self, hint):
        if self._pad and gp.haversine_m(self._pad[0], self._pad[1], *hint) < 2.0 and self._pad_src == "rocket":
            return
        self._pad, self._pad_src = hint, "rocket"
        if hasattr(self.map, "set_pad"):
            self.map.set_pad(*hint)
        try:
            with open(self._pad_file(), "w", encoding="utf-8") as fh:
                json.dump({"lat": hint[0], "lon": hint[1], "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, fh)
        except OSError:
            pass

    def download_map(self):
        if getattr(self, "_dl", None) is not None and self._dl.isRunning():
            return
        lat, lon = self._pad or self._load_pad()
        self._dl = gw.MapDownloader(os.path.join(self.user_dir, "map_cache"), lat, lon)
        self._dl.progress.connect(lambda v, t: self.map_btn.setText(f"MAPA {v} %"))
        self._dl.finished.connect(lambda: self.map_btn.setText("MAPA ⭳ okolí rampy"))
        self._dl.start()

    # ------------------------------------------------------------------ log
    def _open_log(self):
        if getattr(self, "_log_f", None):
            try:
                self._log_f.close()
            except Exception:
                pass
        d = os.path.join(self.user_dir, "logs")
        os.makedirs(d, exist_ok=True)
        self.log_path = os.path.join(d, time.strftime(f"gs_%Y%m%d_%H%M%S_{self.key.lower()}.csv"))
        self._log_f = open(self.log_path, "w", encoding="utf-8", newline="")
        self._log_f.write(f"# ground station log, rocket {self.key}; columns: pc_time, then the station line "
                          "(docs/USB_PROTOCOL.md)\n")
        self._log_flush = time.time()

    def _log_line(self, now: float, s: str):
        try:
            self._log_f.write(f"{now:.3f},{s}\n")
            if now - self._log_flush > 5:
                self._log_f.flush()
                self._log_flush = now
        except Exception:
            pass

    def note(self, s: str):
        if hasattr(self, "log"):
            self.log.appendPlainText(time.strftime("%H:%M:%S ") + s)

    def closeEvent(self, ev):
        self.stop_reader()
        try:
            self._log_f.close()
        except Exception:
            pass
        self.narrator.close()
        super().closeEvent(ev)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--rocket", choices=["ZENITH", "ARIADNA", "STAND", "zenith", "ariadna", "stand"])
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--demo-speed", type=float, default=1.0, help="simulated time runs this many times faster")
    ap.add_argument("--screenshot", help="save a screenshot after --seconds and quit")
    ap.add_argument("--seconds", type=float, default=3.0)
    a = ap.parse_args()
    gw.set_windows_app_id("VoltinoLabs.GroundStation")
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    f = QtGui.QFont("Segoe UI")
    f.setPixelSize(13)
    app.setFont(f)
    gw.apply_theme(app, str(QtCore.QSettings(*SETTINGS).value("theme", "light")))
    icon = gw.app_icon()
    if not icon.isNull():
        app.setWindowIcon(icon)
    w = GroundStation(a.port, a.demo, a.demo_speed, a.rocket)
    if a.screenshot:
        w.resize(1536, 864)
        w.show()
        QtCore.QTimer.singleShot(int(a.seconds * 1000), lambda: (w.grab().save(a.screenshot), app.quit()))
    else:
        w.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
