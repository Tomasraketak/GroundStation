"""MODE STAND — the rampa alone as a static-fire thrust stand.

The page the ground-station window shows instead of the rocket tabs:

    left    live values (thrust, peak, chamber pressure, temperatures, battery)
    middle  live charts and the result of the last test (curve, impulse, class)
    right   the firing procedure: automatic checks from the rampa, the few
            settings, manual safety confirmations, TARE / zero, and the two
            big steps ① ODJISTIT (ARM) and ② ZAPÁLIT (typed confirmation)

Safety rules the page follows (the rampa enforces the same on its side):
  * ARM only when every automatic check is green and every manual box ticked;
  * FIRE only while ARMED, confirmed by typing ZAPAL;
  * ABORT (top bar, Ctrl+Shift+A, station button BC) works in every state;
  * after a firing without thrust the page shows a hang-fire warning with a
    5-minute timer.
After every test the 100 Hz curve is pulled over LoRa and saved as CSV to
<Documents>/GroundStation/stand. The full-rate record (2000 S/s) stays on
the rampa's flash: download it with the rampa diagnostic GUI over USB.
"""
from __future__ import annotations

import csv
import math
import os
import time

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

import gs_protocol as gp
import gs_widgets as gw
from gs_widgets import Tile, CheckRow, StripChart, badge, kbutton, panel, caption, set_level, C

HANGFIRE_WAIT_S = 300


def fmt(v, f: str, none: str = "--") -> str:
    return none if v is None or v != v else f.format(v)


class CurvePlot(QtWidgets.QWidget):
    """Thrust (left axis) and chamber pressure (right axis) of one test vs time from T-0."""

    def __init__(self):
        super().__init__()
        self.t: list = []
        self.f: list = []
        self.p: list = []
        self.unit = "N"
        self.title = "Křivka posledního testu"
        self.burn = None            # (t_start, t_end) shaded
        self.setMinimumHeight(170)

    def set(self, t, f, p, unit="N", title=None, burn=None):
        self.t, self.f, self.p, self.unit = list(t), list(f), list(p), unit
        self.burn = burn
        if title:
            self.title = title
        self.update()

    def clear(self, title="Křivka posledního testu"):
        self.t, self.f, self.p, self.burn, self.title = [], [], [], None, title
        self.update()

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QtGui.QPen(C("border"), 1))
        p.setBrush(C("panel"))
        p.drawRoundedRect(QtCore.QRectF(0.5, 0.5, w - 1, h - 1), 8, 8)
        l, r, t, b = 56, 56, 38, 24
        pw, ph = w - l - r, h - t - b
        f = p.font()
        f.setBold(True)
        f.setPixelSize(12)
        p.setFont(f)
        p.setPen(C("text"))
        p.drawText(10, 16, self.title)
        if len(self.t) < 2 or pw < 20 or ph < 20:
            p.setPen(C("dim"))
            p.drawText(l + 10, t + ph // 2, "zatím žádná křivka")
            return
        t0, t1 = self.t[0], self.t[-1]
        fmax = max(1e-6, max(self.f) * 1.1)
        fmin = min(0.0, min(self.f) * 1.1)
        pvals = [v for v in self.p if v == v]
        pmax = max(1e-6, max(pvals) * 1.1) if pvals else 1.0
        pmin = min(0.0, min(pvals)) if pvals else 0.0

        def X(tt):
            return l + (tt - t0) / (t1 - t0) * pw

        def YF(v):
            return t + ph - (v - fmin) / (fmax - fmin) * ph

        def YP(v):
            return t + ph - (v - pmin) / (pmax - pmin) * ph

        if self.burn:
            p.fillRect(QtCore.QRectF(X(self.burn[0]), t, X(self.burn[1]) - X(self.burn[0]), ph),
                       QtGui.QColor(C("warn").red(), C("warn").green(), C("warn").blue(), 40))
        f.setBold(False)
        f.setPixelSize(11)
        p.setFont(f)
        for i in range(5):
            y = t + ph * i / 4
            p.setPen(QtGui.QPen(C("grid"), 1))
            p.drawLine(l, int(y), l + pw, int(y))
            p.setPen(C("s1"))
            p.drawText(2, int(y) + 4, f"{fmax - (fmax - fmin) * i / 4:7.1f}")
            p.setPen(C("s2"))
            p.drawText(l + pw + 4, int(y) + 4, f"{pmax - (pmax - pmin) * i / 4:6.0f}")
        p.setPen(C("muted"))
        for i in range(7):
            tt = t0 + (t1 - t0) * i / 6
            p.drawText(int(X(tt)) - 14, t + ph + 16, f"{tt:+.1f}s")
        if t0 < 0 < t1:
            p.setPen(QtGui.QPen(C("bad"), 1, Qt.PenStyle.DashLine))
            p.drawLine(int(X(0)), t, int(X(0)), t + ph)
            p.drawText(int(X(0)) + 3, t + 12, "T-0")
        for vals, Y, col in ((self.p, YP, "s2"), (self.f, YF, "s1")):
            path = QtGui.QPainterPath()
            first = True
            for tt, v in zip(self.t, vals):
                if v != v:
                    continue
                pt = QtCore.QPointF(X(tt), Y(v))
                if first:
                    path.moveTo(pt)
                    first = False
                else:
                    path.lineTo(pt)
            p.setPen(QtGui.QPen(C(col), 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(path)
        p.setPen(C("s1"))
        p.drawText(l, t - 6, f"tah [{self.unit}]")
        p.setPen(C("s2"))
        p.drawText(l + pw - 60, t - 6, "tlak [psi]")


class StandPanel(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win                      # GroundStation window: send(), cmd(), note(), settings
        self.t_start = time.time()
        self._last_test = None              # test_no of the result on screen
        self._dl_test = None                # curve being downloaded
        self._dl_off = None
        self._dl_t = 0.0
        self._saved = set()
        self._hang_since = None
        self._prev_state = None
        self._note = None                   # (text, level) of the last action
        self._build()

    # ------------------------------------------------------------------ layout
    def _build(self):
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 8, 0, 0)
        lay.setSpacing(10)

        left = QtWidgets.QVBoxLayout()
        left.setSpacing(7)
        lay.addLayout(left, 22)
        self.tiles = {}
        for key, title, hero in (("f", "Tah", True), ("peak", "Špička (od posl. TLM)", False), ("p", "Tlak v komoře", True),
                                 ("t1", "Teplota 1", False), ("t2", "Teplota 2", False), ("bat", "Baterie rampy", False)):
            tl = Tile(title, hero)
            self.tiles[key] = tl
            left.addWidget(tl)
        left.addStretch(1)

        mid = QtWidgets.QVBoxLayout()
        mid.setSpacing(7)
        lay.addLayout(mid, 46)
        self.ch_f = StripChart("Tah", [("tah", "s1")], 30, "N", min_span=10)
        self.ch_p = StripChart("Tlak", [("tlak", "s2")], 30, "psi", min_span=20)
        self.ch_t = StripChart("Teploty", [("T1", "s3"), ("T2", "s4")], 120, "°C", min_span=5)
        mid.addWidget(self.ch_f, 3)
        mid.addWidget(self.ch_p, 2)
        mid.addWidget(self.ch_t, 2)
        res, rl = panel("Výsledek posledního testu")
        mid.addWidget(res, 4)
        self.hang_lb = badge("")
        self.hang_lb.setWordWrap(True)
        self.hang_lb.setStyleSheet("font-size: 16px; padding: 8px;")
        self.hang_lb.hide()
        rl.addWidget(self.hang_lb)
        self.res_lb = QtWidgets.QLabel("Zatím žádný test.")
        self.res_lb.setWordWrap(True)
        self.res_lb.setTextFormat(Qt.TextFormat.RichText)
        self.res_lb.setStyleSheet("background: transparent; font-size: 13px;")
        rl.addWidget(self.res_lb)
        self.curve = CurvePlot()
        rl.addWidget(self.curve, 1)
        row = QtWidgets.QHBoxLayout()
        self.dl_btn = kbutton("Stáhnout křivku", "", "Stáhne 100 Hz křivku posledního testu z rampy (LoRa)")
        self.dl_btn.clicked.connect(lambda: self.start_download(force=True))
        self.res_btn = kbutton("Vyžádat výsledek", "", "Rampa znovu pošle souhrn posledního testu")
        self.res_btn.clicked.connect(lambda: self.win.cmd("RESULT"))
        bf = kbutton("Složka s testy", "", self.stand_dir())
        bf.clicked.connect(lambda: QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(self.stand_dir())))
        self.dl_lb = QtWidgets.QLabel("")
        self.dl_lb.setStyleSheet("background: transparent; font-weight: 700;")
        for wdg in (self.dl_btn, self.res_btn, bf):
            row.addWidget(wdg)
        row.addWidget(self.dl_lb, 1)
        rl.addLayout(row)

        right = QtWidgets.QVBoxLayout()
        right.setSpacing(8)
        lay.addLayout(right, 32)
        chk, cl = panel("Kontrola rampy (automaticky)")
        right.addWidget(chk)
        self.check_rows = []
        for _ in range(8):
            r = CheckRow()
            self.check_rows.append(r)
            cl.addWidget(r)

        proc, pl = panel("Test motoru – postup")
        right.addWidget(proc, 1)
        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(8)
        self.ch_cb = QtWidgets.QComboBox()
        self.ch_cb.addItems(["1", "2", "3"])
        self.ch_cb.setToolTip("Výstup rampy J5 / J6 / J7, do kterého je zapojená rozbuška motoru")
        self.pulse_sp = QtWidgets.QSpinBox()
        self.pulse_sp.setRange(50, 5000)
        self.pulse_sp.setSingleStep(100)
        self.pulse_sp.setValue(int(self.win.settings.value("stand_pulse", 1000)))
        self.pulse_sp.setSuffix(" ms")
        self.pulse_sp.setToolTip("Jak dlouho teče proud do rozbušky")
        self.cd_sp = QtWidgets.QSpinBox()
        self.cd_sp.setRange(3, 60)
        self.cd_sp.setValue(int(self.win.settings.value("stand_cd", 10)))
        self.cd_sp.setSuffix(" s")
        self.rec_sp = QtWidgets.QSpinBox()
        self.rec_sp.setRange(2, 120)
        self.rec_sp.setValue(int(self.win.settings.value("stand_rec", 20)))
        self.rec_sp.setSuffix(" s")
        self.rec_sp.setToolTip("Jak dlouho po T-0 se zaznamenává (musí pokrýt celé hoření + dohoření)")
        brt = kbutton("Uložit", "", "Pošle délku záznamu rampě (uloží se v ní)")
        brt.clicked.connect(lambda: self.win.cmd(f"RECTIME {self.rec_sp.value()}"))
        g.addWidget(QtWidgets.QLabel("Rozbuška"), 0, 0)
        g.addWidget(self.ch_cb, 0, 1)
        g.addWidget(QtWidgets.QLabel("Puls"), 0, 2)
        g.addWidget(self.pulse_sp, 0, 3)
        g.addWidget(QtWidgets.QLabel("Odpočet"), 1, 0)
        g.addWidget(self.cd_sp, 1, 1)
        g.addWidget(QtWidgets.QLabel("Záznam po T-0"), 1, 2)
        rr = QtWidgets.QHBoxLayout()
        rr.addWidget(self.rec_sp)
        rr.addWidget(brt)
        g.addLayout(rr, 1, 3)
        pl.addLayout(g)
        for spn, key in ((self.pulse_sp, "stand_pulse"), (self.cd_sp, "stand_cd"), (self.rec_sp, "stand_rec")):
            spn.valueChanged.connect(lambda v, k=key: self.win.settings.setValue(k, v))

        row = QtWidgets.QHBoxLayout()
        self.tare_btn = kbutton("TARE – vynulovat tah", "primary", "Tenzometr bez zatížení (motor upevněn, nehoří) → nula")
        self.tare_btn.clicked.connect(self.tare)
        self.pz_btn = kbutton("Nula tlaku", "", "Čidlo tlaku na atmosféře → nula (jen 0,5 V čidla)")
        self.pz_btn.clicked.connect(lambda: self._confirm("Nastavit nulu tlakového čidla?\n\nČidlo musí být na atmosféře (motor bez tlaku).", "PZERO"))
        self.cal_btn = kbutton("Kalibrace závažím…", "", "Zavěs známé závaží, zadej hmotnost → měřítko tenzometru")
        self.cal_btn.clicked.connect(self.calibrate)
        for b in (self.tare_btn, self.pz_btn, self.cal_btn):
            row.addWidget(b)
        pl.addLayout(row)
        row = QtWidgets.QHBoxLayout()
        self.rec_cold_sp = QtWidgets.QSpinBox()
        self.rec_cold_sp.setRange(1, 120)
        self.rec_cold_sp.setValue(10)
        self.rec_cold_sp.setSuffix(" s")
        self.cold_btn = kbutton("Zkušební záznam (bez zážehu)", "", "Záznam všech čidel na flash bez pyro – kalibrace, test tlaku")
        self.cold_btn.clicked.connect(lambda: self.win.cmd(f"RECORD {self.rec_cold_sp.value()}"))
        self.stop_btn = kbutton("Stop záznamu", "", "Ukončí probíhající záznam")
        self.stop_btn.clicked.connect(lambda: self.win.cmd("STOP"))
        self.buz_btn = kbutton("Bzučák", "", "Bzučák rampy (najít ji / upozornit okolí)")
        self.buz_btn.setCheckable(True)
        self.buz_btn.toggled.connect(lambda on: self.win.cmd(f"BUZZER {1 if on else 0}"))
        for b in (self.rec_cold_sp, self.cold_btn, self.stop_btn, self.buz_btn):
            row.addWidget(b)
        pl.addLayout(row)

        pl.addWidget(caption("Bezpečnost – potvrď ručně"))
        self.manual = []
        for text in ("Motor pevně upevněn na stojanu, tryska míří do bezpečného prostoru",
                     "Prostor vyklizen, všichni za bezpečnou vzdáleností / krytem",
                     "Rozbuška zapojena až teď, klíč rampy zapnut (servo spínač sepne rampa při ARM)",
                     "Hasicí přístroj připraven, vedoucí testu dal souhlas"):
            cb = QtWidgets.QCheckBox(text)
            cb.toggled.connect(lambda _on: self.refresh())
            self.manual.append(cb)
            pl.addWidget(cb)

        self.arm_btn = kbutton("①  ODJISTIT (ARM)", "warn", "Rampa zkontroluje klíč, rozbušku, baterii a čidla a připraví záznam")
        self.arm_btn.setMinimumHeight(42)
        self.arm_btn.setStyleSheet("font-size: 16px;")
        self.arm_btn.clicked.connect(self.arm)
        self.fire_btn = kbutton("②  ZAPÁLIT MOTOR", "abort", "Odpočet a zážeh – potvrzuje se napsáním ZAPAL")
        self.fire_btn.setMinimumHeight(52)
        self.fire_btn.clicked.connect(self.fire)
        self.disarm_btn = kbutton("ZAJISTIT (DISARM)", "go", "Zpět do bezpečného stavu, zruší i zablokovaný ABORT")
        self.disarm_btn.clicked.connect(lambda: self.win.cmd("DISARM"))
        pl.addWidget(self.arm_btn)
        pl.addWidget(self.fire_btn)
        pl.addWidget(self.disarm_btn)
        self.act_lb = badge("Připoj stanici a zapni rampu (bez rakety na pogu)")
        self.act_lb.setWordWrap(True)
        pl.addWidget(self.act_lb)
        pl.addStretch(1)

    # ------------------------------------------------------------------ helpers
    def stand_dir(self) -> str:
        d = os.path.join(gw.user_dir(), "stand")
        os.makedirs(d, exist_ok=True)
        return d

    @property
    def link(self) -> gp.StandLink:
        return self.win.link

    def reset(self):
        for ch in (self.ch_f, self.ch_p, self.ch_t):
            ch.clear()
        self.curve.clear()
        self.res_lb.setText("Zatím žádný test.")
        self._last_test = self._dl_test = self._dl_off = None
        self._hang_since = None
        self._prev_state = None
        self._note = None
        for cb in self.manual:
            cb.blockSignals(True)
            cb.setChecked(False)
            cb.blockSignals(False)

    def _say(self, text: str, level: str):
        self._note = (text, level, time.time())
        self.act_lb.setText(text)
        set_level(self.act_lb, level)

    def _confirm(self, q: str, c: str):
        if QtWidgets.QMessageBox.question(self, "Potvrdit", q) == QtWidgets.QMessageBox.StandardButton.Yes:
            self.win.cmd(c)

    # ------------------------------------------------------------------ actions
    def tare(self):
        if self.link.t and gp.to_int(self.link.t.get("state")) not in (0, 2):
            return
        self._confirm("Vynulovat tah (TARE)?\n\nNa tenzometru nesmí nic tlačit kromě upevněného motoru.", "TARE")

    def calibrate(self):
        kg, ok = QtWidgets.QInputDialog.getDouble(
            self, "Kalibrace tenzometru",
            "1) Nejdřív TARE bez zátěže.\n2) Zatěž tenzometr ve směru tahu známým závažím.\n3) Zadej jeho hmotnost:",
            1.000, 0.010, 100000.0, 3)
        if ok and kg > 0:
            self.win.cmd(f"CAL {kg * 9.80665:.4f}")
            self._say(f"Kalibrace {kg:.3f} kg ({kg * 9.80665:.2f} N) odeslána", "info")

    def arm(self):
        ch, pulse = int(self.ch_cb.currentText()), self.pulse_sp.value()
        q = (f"ODJISTIT rampu?\n\n• rozbuška {ch}, puls {pulse} ms\n• rampa smaže místo pro záznam ({self.rec_sp.value()} s po T-0)\n"
             "• rampa bude pípat; od teď se k motoru nepřibližuj\n• ZAJISTIT / ABORT kdykoli vrátí bezpečný stav")
        if QtWidgets.QMessageBox.question(self, "ODJISTIT", q) == QtWidgets.QMessageBox.StandardButton.Yes:
            if self.win.cmd(f"ARM {ch} {pulse}"):
                self._say("ARM odeslán – čekám na potvrzení rampy…", "warn")

    def fire(self):
        n = self.cd_sp.value()
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Potvrzení zážehu")
        v = QtWidgets.QVBoxLayout(dlg)
        head = QtWidgets.QLabel(f"ZÁŽEH MOTORU ZA {n} s")
        head.setObjectName("State")
        head.setProperty("level", "bad")
        head.setAlignment(Qt.AlignmentFlag.AlignCenter)
        v.addWidget(head)
        info = QtWidgets.QLabel(f"• Rampa začne odpočet T-{n} s a v T-3 s záznam.\n"
                                "• Odpočet se sám přeruší při ztrátě spojení (2,5 s), vypnutí klíče,\n"
                                "  ztrátě kontinuity, tlačítku na rampě nebo ABORT ze stanice.\n"
                                "• V T-0 rampa pustí proud do rozbušky.\n\nPro potvrzení napiš velkými písmeny ZAPAL:")
        info.setStyleSheet("font-size: 14px; font-weight: 600;")
        v.addWidget(info)
        edit = QtWidgets.QLineEdit()
        edit.setStyleSheet("font-size: 20px; font-weight: 800;")
        v.addWidget(edit)
        row = QtWidgets.QHBoxLayout()
        cancel = kbutton("ZRUŠIT")
        go = kbutton(f"ZAPÁLIT ZA {n} s", "abort")
        go.setEnabled(False)
        edit.textChanged.connect(lambda t: go.setEnabled(t.strip() == "ZAPAL"))
        cancel.clicked.connect(dlg.reject)
        go.clicked.connect(dlg.accept)
        row.addWidget(cancel)
        row.addWidget(go, 1)
        v.addLayout(row)
        dlg.setMinimumWidth(520)
        if dlg.exec() != QtWidgets.QDialog.DialogCode.Accepted:
            return
        if self.win.cmd(f"FIRE {n}"):
            self._say("POVEL K ZÁŽEHU odeslán – čekám na potvrzení rampy…", "warn")

    # ------------------------------------------------------------------ from the window
    def on_telemetry(self, now: float):
        L = self.link
        tt = now - self.t_start
        for ts, fv in L.samples:
            self.ch_f.add(ts - self.t_start, tah=fv)
        self.ch_p.add(tt, tlak=gp.to_float(L.t.get("p")))
        self.ch_t.add(tt, T1=gp.to_float(L.t.get("t1")), T2=gp.to_float(L.t.get("t2")))
        unit = L.force_unit()
        if self.ch_f.unit != unit:
            self.ch_f.unit = unit
        st = gp.to_int(L.t.get("state"))
        if self._prev_state is not None and st != self._prev_state:
            self._note = None
            if st == 0 and self._prev_state != 0:
                for cb in self.manual:          # after every test / disarm / abort the confirmations start over
                    cb.setChecked(False)
            if st == 3:
                self.ch_f.clear()
                self.ch_p.clear()
        self._prev_state = st
        if L.flag("HANGFIRE"):
            if self._hang_since is None:
                self._hang_since = now
        else:
            self._hang_since = None

    def on_result(self):
        r = self.link.res
        test = gp.to_int(r.get("test_no"))
        if test != self._last_test:
            self._last_test = test
            self.win.note(f"VÝSLEDEK testu #{test}: špička {gp.to_float(r.get('peak')):.1f}, impulz "
                          f"{gp.to_float(r.get('impulse')):.2f}, hoření {gp.to_float(r.get('burn')):.2f} s")
            self.start_download()
        self._show_result()

    def on_curve(self):
        if self._dl_test is None:
            return
        self._dl_next()

    def on_uplink(self, name: str, what: str, rc: int):
        txt = gp.SL_RC[rc] if 0 <= rc < len(gp.SL_RC) else str(rc)
        if name == "ARM" and what == "ACK":
            self._say("✔ Rampa odjištěna – maže místo pro záznam, pak ODJIŠTĚNO" if rc == 1 else f"✖ ARM odmítnut: {txt}",
                      "warn" if rc == 1 else "bad")
        elif name == "FIRE" and what == "ACK":
            self._say("✔ ODPOČET BĚŽÍ NA RAMPĚ" if rc == 1 else f"✖ Zážeh odmítnut: {txt}", "bad" if rc != 1 else "warn")
            self.win.narrator.say("Countdown confirmed" if rc == 1 else "Fire rejected", urgent=True)
        elif name in ("ARM", "FIRE") and what in ("FAILED", "CANCELLED"):
            self._say(f"⚠ {name} rampa nepotvrdila – zkontroluj spojení", "bad")
        elif name in ("TARE", "PZERO", "CAL") and what == "ACK":
            self._say(f"{name}: {'hotovo' if rc == 1 else txt}", "okline" if rc == 1 else "bad")
        elif name == "ABORT" and what == "ACK":
            self._say("ABORT potvrzen rampou", "bad")

    # ------------------------------------------------------------------ curve download
    def start_download(self, force: bool = False):
        r = self.link.res
        if not r:
            self.win.cmd("RESULT")
            return
        test = gp.to_int(r.get("test_no"))
        if not (gp.to_int(r.get("flags")) & gp.SL_RF["CURVE"]) or gp.to_int(r.get("curve_n")) == 0:
            self.dl_lb.setText("rampa nemá křivku v paměti")
            return
        if force:
            self.link.curve.pop(test, None)
        self._dl_test = test
        self._dl_next()

    def _dl_next(self):
        test = self._dl_test
        st = gp.to_int(self.link.t.get("state")) if self.link.t else 0
        if st not in (0, 6):            # never during a test: the air belongs to the live data
            return
        nxt = self.link.next_missing(test)
        n = gp.to_int(self.link.res.get("curve_n"))
        if nxt is None:
            self._dl_test = None
            self.dl_lb.setText(f"křivka #{test} stažena ({n} bodů)")
            self._show_result()
            self._save_csv(test)
            return
        self._dl_off, self._dl_t = nxt, time.time()
        self.dl_lb.setText(f"stahuji křivku #{test}: {100 * nxt // max(1, n)} %")
        self.win.send(f"CMD CURVE {nxt}", quiet=True)

    def _save_csv(self, test: int):
        if test in self._saved:
            return
        pts = self.link.curve_points(test)
        if not pts:
            return
        r = self.link.res
        path = os.path.join(self.stand_dir(), time.strftime(f"test_{test:03d}_%Y%m%d_%H%M%S.csv"))
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                wr = csv.writer(fh)
                for k in self.link.SR_FIELDS:
                    wr.writerow([f"# {k}", r.get(k, "")])
                wr.writerow([f"# force unit", self.link.force_unit() if gp.to_int(r.get('flags')) & gp.SL_RF["LC_CAL"] else "uV/V"])
                wr.writerow(["t_s", "force", "pressure_psi"])
                for tt, f, p in zip(*pts):
                    wr.writerow([f"{tt:.3f}", f"{f:.2f}", f"{p:.1f}"])
            self._saved.add(test)
            self.win.note(f"křivka testu #{test} uložena: {path}")
        except OSError as ex:
            self.win.note(f"CSV se nepodařilo uložit: {ex}")

    def _show_result(self):
        r = self.link.res
        if not r:
            return
        test = gp.to_int(r.get("test_no"))
        fl = gp.to_int(r.get("flags"))
        unit = "N" if fl & gp.SL_RF["LC_CAL"] else "µV/V"
        peak, imp, burn = gp.to_float(r.get("peak")), gp.to_float(r.get("impulse")), gp.to_float(r.get("burn"))
        avg, pmax = gp.to_float(r.get("avg")), gp.to_float(r.get("pmax"))
        kind = "zkušební záznam" if fl & gp.SL_RF["COLD"] else "zážeh"
        state = []
        if fl & gp.SL_RF["ABORTED"]:
            state.append("<b style='color:%s'>PŘERUŠENO</b>" % gw.T["bad"])
        if (fl & gp.SL_RF["FIRED"]) and not (fl & gp.SL_RF["IGNITION"]):
            state.append("<b style='color:%s'>MOTOR NEZAŽEHL</b>" % gw.T["bad"])
        state.append("uloženo na flash" if fl & gp.SL_RF["SAVED"] else "<b>NEULOŽENO</b>")
        cls = gp.motor_class(imp, avg) if unit == "N" and fl & gp.SL_RF["IGNITION"] else "–"
        self.res_lb.setText(
            f"<span style='font-size:15px'><b>Test #{test}</b> · {kind} · {' · '.join(state)}</span><br>"
            f"Špička <b>{peak:.1f} {unit}</b> v T+{gp.to_float(r.get('t_peak')):.2f} s · "
            f"impulz <b>{imp:.2f} {unit}·s</b> · hoření <b>{burn:.2f} s</b> · průměr {avg:.1f} {unit} · "
            f"třída <b>{cls}</b><br>"
            f"zpoždění zážehu {gp.to_float(r.get('ign_delay')):.2f} s · max. tlak <b>{pmax:.0f} psi</b> "
            f"({pmax * 0.0689476:.1f} bar) · max. T1 {fmt(gp.to_float(r.get('t1max')), '{:.1f}')} °C · "
            f"T2 {fmt(gp.to_float(r.get('t2max')), '{:.1f}')} °C · "
            f"{gp.to_int(r.get('n_frames'))} vzorků @ {gp.to_int(r.get('sps'))} S/s na flash")
        pts = self.link.curve_points(test)
        if pts:
            ign = gp.to_float(r.get("ign_delay"), 0.0)
            self.curve.set(*pts, unit=unit, title=f"Test #{test}: tah a tlak (100 Hz)",
                           burn=(ign, ign + burn) if burn > 0 else None)
        elif self.curve.title != f"Test #{test}: stahuji křivku…":
            self.curve.clear(f"Test #{test}: stahuji křivku…")

    # ------------------------------------------------------------------ 10 Hz
    def refresh(self):
        L = self.link
        if not isinstance(L, gp.StandLink):
            return
        now = time.time()
        t = L.t
        age = now - L.t_rx if t else 1e9
        fresh = bool(t) and age < 2.5
        connected = self.win.reader is not None
        unit = L.force_unit()
        st = gp.to_int(t.get("state"), -1) if t else -1

        # --- tiles
        if t:
            f9 = gp.to_float(t.get("f9"))
            lvl = "accent" if st in (4, 5) else "ok" if fresh else "warn"
            self.tiles["f"].set(fmt(f9, "{:.1f} " + unit), "kalibrováno" if unit == "N" else "NEKALIBROVÁNO – µV/V", lvl)
            self.tiles["peak"].set(fmt(gp.to_float(t.get("fpeak")), "{:.1f} " + unit), " ", "info")
            pv = gp.to_float(t.get("p"))
            self.tiles["p"].set(fmt(pv, "{:.0f} psi"), fmt(pv * 0.0689476 if pv == pv else None, "{:.2f} bar") if L.flag("PRESS") else "čidlo nepřipojeno",
                                "accent" if st in (4, 5) and (pv or 0) > 5 else "ok" if L.flag("PRESS") else "idle")
            for k, flag in (("t1", "TH1"), ("t2", "TH2")):
                tv = gp.to_float(t.get(k))
                self.tiles[k].set(fmt(tv, "{:.1f} °C"), " " if L.flag(flag) else "termistor nepřipojen",
                                  ("warn" if (tv or 0) > 80 else "ok") if L.flag(flag) else "idle")
            vb = gp.to_float(t.get("vbat"))
            self.tiles["bat"].set(fmt(vb, "{:.2f} V"), f"čidla {gp.to_float(t.get('v5')):.2f} V · {gp.to_int(t.get('sps'))} S/s",
                                  "bad" if vb < 6.6 else "warn" if vb < 7.2 else "ok")
        else:
            for tl in self.tiles.values():
                tl.set("--", " ", "idle")

        # --- automatic checks
        cont = gp.to_int(t.get("cont")) if t else 0
        ch = int(self.ch_cb.currentText())
        free = gp.to_int(t.get("flash_free")) if t else 0
        checks = [
            ("Spojení s rampou", "ok" if fresh else "bad",
             (f"paket před {age:.1f} s · {gp.to_float(t.get('rssi')):.0f} dBm" if t else
              ("rampa slyšet, přepínám ji…" if L.b else "bez spojení – rampa zapnutá, bez rakety?")) if connected else "stanice nepřipojena"),
            ("Klíč rampy (pyro napájení)", "ok" if L.flag("KEY") or (L.servo_arming() and st in (0, 1)) else "bad",
             ("sepnut" + (" (klíč + servo)" if L.servo_arming() else "")) if L.flag("KEY") else
             ("druhý spínač sepne servo při ARM – lidský klíč zapni u rampy" if L.servo_arming() and st in (0, 1) else "VYPNUT")),
            (f"Rozbuška {ch} – kontinuita", "ok" if cont & (1 << (ch - 1)) else "bad",
             "OK" if cont & (1 << (ch - 1)) else "BEZ KONTINUITY" + (f" (OK je na {', '.join(str(i + 1) for i in range(3) if cont & (1 << i))})" if cont else "")),
            ("Baterie rampy", "ok" if t and gp.to_float(t.get("vbat")) >= 7.2 else "warn" if t and gp.to_float(t.get("vbat")) >= 6.6 else "bad",
             fmt(gp.to_float(t.get("vbat")) if t else None, "{:.2f} V (2S, min 6,6 V)")),
            ("Tenzometr (ADS1220)", "ok" if L.flag("ADS") and unit == "N" else "warn" if L.flag("ADS") else "bad",
             ("měří, kalibrován" if unit == "N" else "měří, NEKALIBROVÁN (µV/V)") + (" · TARE ✔" if L.flag("TARED") else " · bez TARE")
             if L.flag("ADS") else "NEMĚŘÍ"),
            ("Záznam na flash", "ok" if L.flag("FLASH") else "bad", f"volno {free} %" if L.flag("FLASH") else "chybí nebo plná"),
            ("Tlakové čidlo", "ok" if L.flag("PRESS") else "warn", "připojeno" if L.flag("PRESS") else "nepřipojeno (test půjde i bez něj)"),
            ("Termistory", "ok" if L.flag("TH1") and L.flag("TH2") else "warn",
             f"T1 {'OK' if L.flag('TH1') else '–'} · T2 {'OK' if L.flag('TH2') else '–'}"),
        ]
        for row, (name, lvl, det) in zip(self.check_rows, checks):
            row.set(name, lvl if t or name.startswith("Spojení") else "idle", det)
        must = checks[:6]
        auto_ok = fresh and all(lvl == "ok" or (n.startswith("Tenzometr") and lvl == "warn") for n, lvl, _ in must)
        manual_ok = all(cb.isChecked() for cb in self.manual)

        # --- buttons
        self.arm_btn.setEnabled(fresh and st == 0 and auto_ok and manual_ok)
        self.fire_btn.setEnabled(fresh and st == 2 and manual_ok)
        self.disarm_btn.setEnabled(bool(t) and st in (0, 1, 2, 3, 6))
        safe = fresh and st in (0, 2)
        self.tare_btn.setEnabled(safe)
        for b in (self.pz_btn, self.cal_btn, self.cold_btn):
            b.setEnabled(fresh and st == 0)
        self.stop_btn.setEnabled(fresh and st in (5, 7))
        self.dl_btn.setEnabled(bool(L.res) and st in (0, 6))
        self.res_btn.setEnabled(fresh)

        # --- curve download that lost a packet: ask again
        if self._dl_test is not None and now - self._dl_t > 4.0 and st in (0, 6):
            self._dl_next()

        # --- hang-fire
        if self._hang_since is not None:
            left = HANGFIRE_WAIT_S - (now - self._hang_since)
            self.hang_lb.setText("⚠ ROZBUŠKA ODPÁLENA, TAH NEZAZNAMENÁN – MOTOR MŮŽE ZAŽEHNOUT POZDĚ. "
                                 + (f"Nepřibližuj se ještě {int(left // 60)}:{int(left % 60):02d}." if left > 0
                                    else "Čekací doba uplynula – přibližuj se opatrně, nejdřív vypni klíč."))
            set_level(self.hang_lb, "bad" if left > 0 else "warn")
            self.hang_lb.show()
        else:
            self.hang_lb.hide()

        # --- status line
        if self._note and now - self._note[2] < 6:
            return
        if not connected:
            txt, lvl = "Stanice nepřipojena", "idle"
        elif not t:
            txt, lvl = ("Rampa slyšet jako rampa – přepínám ji do režimu test motoru…" if L.b else
                        "Čekám na rampu… (rampa zapnutá, žádná raketa na pogu)"), "warn"
        elif not fresh:
            txt, lvl = f"ZTRÁTA SPOJENÍ {age:.0f} s – odpočet by se na rampě sám přerušil", "bad"
        elif st == 6:
            flt = L.faults()
            txt, lvl = "PŘERUŠENO" + (f": {', '.join(flt)}" if flt else "") + " – ZAJISTIT vrátí bezpečný stav", "bad"
        elif st == 1:
            txt, lvl = f"Odjišťuji – mažu flash pro záznam {gp.to_int(t.get('progress'))} %", "warn"
        elif st == 2:
            txt, lvl = "ODJIŠTĚNO – rampa pípá. ② ZAPÁLIT nebo ZAJISTIT.", "warn"
        elif st == 3:
            txt, lvl = "ODPOČET – ABORT nahoře kdykoli zastaví", "bad"
        elif st in (4, 5):
            txt, lvl = f"Záznam běží ({gp.to_int(t.get('progress'))} % místa)", "accent"
        elif st == 7:
            txt, lvl = f"Zkušební záznam běží ({gp.to_int(t.get('progress'))} %)", "info"
        elif not auto_ok:
            bad = [n for n, lvl_, _ in must if lvl_ == "bad"]
            txt, lvl = "Rampa nepřipravena: " + ", ".join(bad), "warn"
        elif not manual_ok:
            txt, lvl = "Rampa připravena – potvrď bezpečnostní body", "info"
        else:
            txt, lvl = "Vše splněno – můžeš ODJISTIT", "okline"
        self.act_lb.setText(txt)
        set_level(self.act_lb, lvl)

