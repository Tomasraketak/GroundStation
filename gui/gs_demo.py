"""A simulated ground station + rocket, for trying the GUI without hardware.

It speaks exactly the station's USB text protocol (docs/USB_PROTOCOL.md) in
either mode, so the window runs through its real parsing and command paths:
commands wait for the next 10 s RX window, the rocket answers with an ACK,
PREPARE / COUNTDOWN / ABORT change its state, and a countdown ends in a
simulated flight — a supersonic Zenith flight with drogue and main, or a
Daidalos hop with apogee, a nose-first fall and a propulsive landing.
"""
from __future__ import annotations

import math
import time

from PyQt6 import QtCore


class DemoStation(QtCore.QObject):
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self, mode: str = "ZENITH", speed: float = 1.0):
        super().__init__()
        self.speed = speed
        self.t0 = time.time()
        self.mode = mode
        self.seq = 0
        self._tick_n = 0
        self._reset_rocket()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(15, int(50 / speed)))

    # ------------------------------------------------------------------ station side
    def _reset_rocket(self):
        self.state = 1
        self.t_launch = None
        self.cd_t0 = None
        self.pending = None
        self.last_seq, self.last_rc = 0, 0
        self.main_alt = 300
        self.max_alt = 0.0
        self.apo = 0.0
        self.launch_soon = False
        self.channel = 1
        self._win = None
        self._next_t = self._next_s = self._next_g = 0.0
        self._ack_due = False

    def now(self) -> float:
        return (time.time() - self.t0) * self.speed

    def send(self, text: str):
        p = text.split()
        if not p:
            return
        c = p[0].upper()
        if c == "HELLO":
            self.line.emit(f"$ID,Ground Station,2.00,{self.mode}")
            self.line.emit(f"$MODE,{self.mode}")
        elif c == "MODE":
            if len(p) > 1 and p[1].upper() in ("ZENITH", "ARIADNA") and p[1].upper() != self.mode:
                if self.pending:
                    self._u("CANCELLED")
                self.mode = p[1].upper()
                self._reset_rocket()
            self.line.emit(f"$MODE,{self.mode}")
            self.line.emit(f"#OK MODE {self.mode}")
        elif c == "CANCEL":
            if self.pending:
                self._u("CANCELLED")
            self.pending = None
            self.line.emit("#OK CANCEL")
        elif c == "CMD" and len(p) >= 2:
            name = p[1].upper()
            if self.mode == "ARIADNA" and name in ("MAINALT", "LOCATE", "CAMERA", "ROCKETPWR"):
                self.line.emit(f"#ERR {name}: Ariadna does not have this command")
                return
            if self.pending and self.pending[1] == "ABORT" and name != "ABORT":
                self.line.emit("#ERR abort pending")
                return
            self.seq = self.seq % 255 + 1
            dest = 1 if (name == "ROCKETPWR" or "RAMPA" in p[2:]) else 0
            self.pending = [self.seq, name, p[2:], dest, 0, self.now()]
            self._u("QUEUED")
            self.line.emit(f"#OK queued {name} seq {self.seq}")

    def _u(self, what: str, rc: int = 0):
        seq, name, _a, dest, tries, _t = self.pending
        self.line.emit(f"$U,{seq},{name},{dest},{what},{rc},{tries}")

    def stop(self):
        self.timer.stop()

    # ------------------------------------------------------------------ rocket side
    def _apply(self, name: str, args: list[str], t: float) -> int:
        """Execute an uplink command on the simulated rocket, return the result code."""
        z = self.mode == "ZENITH"
        flying = self.t_launch is not None and self.state not in ((8, 9) if z else (7, 8, 9))
        if name == "ABORT":
            if self.state in ((2, 3) if z else (1, 2)) or self.cd_t0 is not None:
                self.state, self.cd_t0, self.launch_soon = 9, None, False
                return 6
            return 2 if flying else 3
        if flying:
            return 2
        a0 = args[0] if args else "0"
        if name == "PREPARE":
            if self.state in (1, 2, 9):
                self.state, self.launch_soon = 2, True
                return 1
            return 3
        if name == "COUNTDOWN":
            if self.state != 2:
                return 7 if z else 3
            self.cd_t0 = t + float(a0)
            self.channel = int(args[1]) if len(args) > 1 else 1
            if z:
                self.state = 3
            return 1
        if name == "SAFE":
            self.state, self.cd_t0, self.launch_soon = 1, None, False
            return 1
        if name == "MAINALT":
            self.main_alt = int(float(a0))
            return 1
        if name == "REBOOT":
            self._reset_rocket()
            return 1
        return 1

    def tick(self):
        t = self.now()
        self._tick_n += 1
        win = int(t // 10.0)
        crossed = self._win is not None and win != self._win
        self._win = win
        if self.pending and crossed:
            self.pending[4] += 1
            seq, name, args, _dest, tries, _tq = self.pending
            self._u("SENT")
            if name in ("ABORT", "COUNTDOWN"):
                self._u("SENT2")
            if name == "COUNTDOWN" and self.mode == "ARIADNA":
                args = [str(max(0.0, float(args[0]) - (t - self.pending[5])))] + args[1:]
            self.last_seq, self.last_rc = seq, self._apply(name, args, t)
            self._u("ACK", self.last_rc)
            self._ack_due = True
            self.pending = None
        if self.mode == "ZENITH":
            self._zenith(t)
        else:
            self._ariadna(t)
        if t >= self._next_g:
            self._next_g = t + 1.0
            nxt = int((10.0 - t % 10.0) * 1000)
            self.line.emit(f"$G,{int(t * 7)},0,90,1,{nxt},{1 if self.pending else 0},0,-1,{self.mode},0,1,1")

    # ---------------------------------------------------------------- Zenith flight
    def _zenith(self, t: float):
        if t < self._next_t:
            return
        self._next_t = t + 0.2
        alt = vz = acc = mach = 0.0
        tf = -1.0
        if self.state == 3 and self.cd_t0 is not None and t >= self.cd_t0:
            self.state, self.t_launch, self.cd_t0 = 4, t, None
        if self.t_launch is not None and self.state != 9:
            tf = t - self.t_launch
            if tf < 2.4:
                acc, vz, alt = 21.0, 206 * tf, 103 * tf * tf
            elif tf < 15.0:
                c = tf - 2.4
                vz = 494 * (1 - c / 12.6) ** 1.6
                alt = 593 + 494 * 12.6 / 2.6 * (1 - (1 - c / 12.6) ** 2.6)
                acc = -2.0 * math.exp(-c / 4)
                self.apo = alt
            else:
                c = tf - 15.0
                top = self.apo
                if top - 25 * c > self.main_alt:
                    alt, vz = top - 25 * c, -25.0
                else:
                    alt = max(0.0, self.main_alt - 6 * (c - (top - self.main_alt) / 25))
                    vz = -6.0 if alt > 0 else 0.0
                acc = 1.0
            self.max_alt = max(self.max_alt, alt)
            mach = abs(vz) / 340
            self.state = 4 if tf < 2.4 else 5 if tf < 15.0 else 6 if alt > self.main_alt else 7 if alt > 0 else 8
        rem = int((self.cd_t0 - t) * 1000) if self.state == 3 and self.cd_t0 else (int(tf * 1000) if tf >= 0 else -1)
        nxt = int((10.0 - t % 10.0) * 1000)
        pyro = 0x07 | (0x40 if self.state in (3, 4, 5, 6, 7) else 0)
        if 6 <= self.state <= 8:
            pyro |= 0x08
        if 7 <= self.state <= 8:
            pyro |= 0x10
        flags = 0x02 | 0x04 | 0x08 | (0x10 if self.state < 4 else 0) | 0x40 | (0x80 if self.state in (1, 2, 3) else 0)
        if pyro & 0x40:
            flags |= 0x01
        lat = 49.7985411 + (alt / 111000) * 0.05
        lon = 16.6912772 + max(tf, 0) * 0.00002
        rampa = 3 if self.state == 3 else 2 if self.state == 2 else 0 if self.state in (4, 5, 6, 7, 8) else 1
        rflags = (0x83 if rampa == 2 else 0x93 if rampa == 3 else 0x43) if rampa else 0
        vals = [-62 - int(alt / 200), 9.0, self.state, flags, int(t * 1000), f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}",
                rem, f"{vz:.1f}", "3.0", f"{acc:.2f}", int(self.max_alt), "8.05", nxt, self.main_alt, "1.8",
                f"{mach:.2f}", 13, pyro, 0x3F, rampa, rflags, self.last_seq, self.last_rc, 16 if self.state == 9 else 0, 0]
        self.line.emit("$T," + ",".join(str(v) for v in vals))

    # ---------------------------------------------------------------- Daidalos hop
    def _ariadna(self, t: float):
        if self.cd_t0 is not None and t >= self.cd_t0 and self.state == 2:
            self.state, self.t_launch, self.cd_t0 = 3, t, None
        in_flight = self.t_launch is not None and self.state not in (7, 8, 9)
        if not in_flight and (t >= self._next_s or self._ack_due):
            self._ack_due = False
            self._next_s = t + (0.5 if self.cd_t0 is not None else 1.0)
            rem = int((self.cd_t0 - t) * 1000) if self.cd_t0 is not None else -1
            nxt = int((10.0 - t % 10.0) * 1000)
            final = 0
            if rem > 1150 and rem - 1150 < nxt:
                nxt, final = rem - 1150, 1
            rs = 3 if self.cd_t0 is not None else 2 if self.launch_soon else 1
            if self.t_launch is not None:
                rs = 5
            rf = 0x83 if rs in (2, 3) else 0x03
            sf = 0x20 | (0x08 if self.launch_soon else 0) | (0x10 if self.cd_t0 is not None else 0) | (0x02 if final else 0)
            self.line.emit(f"$AS,-58,9.5,{int(t * 1000)},-1,{nxt},300,{rem},{self.last_seq},{self.last_rc},"
                           f"{rs},{rf},0,{sf},12400")
        if t < self._next_t:
            return
        self._next_t = t + 0.1
        alt = vz = 0.0
        phase, events, tev, burn = 0, 0, -32768, False
        tilt, az = 2.0 + 0.5 * math.sin(t), 40.0
        tf = 0.0
        if self.t_launch is not None and self.state != 9:
            tf = t - self.t_launch
            if tf < 5:
                self.state, phase, alt, vz = 3, 1, 32 * tf - 1.5 * tf * tf, 32 - 3 * tf
            elif tf < 6:
                self.state, phase, alt, vz = 4, 1, 122.5, 0.0
            elif tf < 10:
                c = tf - 6
                self.state, phase, alt, vz = 5, 2, 122.5 - 16 * c, -16.0
                tev, events = int((10 - tf) * 10), 0x01 | (0x02 if tf > 9.4 else 0)
            elif tf < 12.6:
                c = tf - 10
                self.state, phase, burn = 6, 4, True
                alt = max(0.0, 58.5 * (1 - c / 2.6) ** 2)
                vz = -16 * (1 - c / 2.6)
                tev, events = int((12.6 - tf) * 10), 0x0F | (0x20 if tf > 11 else 0) | (0x40 if alt < 7 else 0)
            else:
                self.state, phase, events = 7, 5, 0xEF
            if phase >= 2:
                tilt = 180 - (3 + 2 * math.sin(t * 2))
            self.max_alt = max(self.max_alt, alt)
        # body->ENU quaternion for a nose (board -Y) tilted `tilt` toward azimuth az
        a = math.radians(tilt)
        z = math.radians(az)
        w0, s0 = math.cos(-math.pi / 4), math.sin(-math.pi / 4)          # board -> upright
        qt = (math.cos(a / 2), -math.cos(z) * math.sin(a / 2), math.sin(z) * math.sin(a / 2), 0.0)
        q = (qt[0] * w0 - qt[1] * s0, qt[0] * s0 + qt[1] * w0, qt[2] * w0 + qt[3] * s0, qt[3] * w0 - qt[2] * s0)
        sg = 1 if q[0] >= 0 else -1
        qx, qy, qz = (int(sg * c * 32767) for c in q[1:])
        cont = 0x07 if phase < 4 else 0x00
        fired = (0x05 if events & 0x20 else 0x01) if phase >= 4 else 0
        flags = (0x03 if self.state >= 2 and self.state != 9 else 0) | (0x04 if self.state >= 3 else 0) | (0x10 if self.state == 9 else 0)
        lat = 49.7985411 + math.sin(tf * 0.3) * 0.00003
        lon = 16.6912772 + math.sin(tf * 0.2) * 0.00004
        vals = [-55 - int(alt / 20), "9.5", int(t * 1000), f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}", f"{vz:.1f}",
                f"{2 + math.sin(t):.1f}", f"{4.2 if burn else 1.0:.2f}", qx, qy, qz,
                f"{math.sin(t * 6) * 3.5 if burn else 0:.2f}", f"{math.cos(t * 5) * 2.5 if burn else 0:.2f}",
                int(95 + math.sin(t * 4) * 20) if burn else 0, tev, 4, self.state, phase,
                int(55 + math.sin(t * 3) * 40) if burn else 100, f"{8.1 - (t % 600) * 0.001:.2f}",
                f"{5.5 if burn else 1.2:.1f}", "0.9", 13, (fired << 4) | cont, 0xFF, flags, events, 0xF4,
                min(255, int(tf)), int(math.sin(tf * 0.3) * 3), int(math.sin(tf * 0.2) * 4)]
        self.line.emit("$AT," + ",".join(str(v) for v in vals))
