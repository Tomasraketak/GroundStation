"""A simulated ground station + rocket, for trying the GUI without hardware.

It speaks exactly the station's USB text protocol (docs/USB_PROTOCOL.md) in
either mode, so the window runs through its real parsing and command paths:
commands wait for the next 10 s RX window, the rocket answers with an ACK,
PREPARE / COUNTDOWN / ABORT change its state, and a countdown ends in a
simulated flight — a supersonic Zenith flight with drogue and main, or a
Daidalos hop with apogee, a nose-first fall and a propulsive landing.
In STAND mode it plays the rampa as a thrust stand: ARM -> erase -> ARMED,
FIRE -> countdown -> igniter -> a ~2.5 s motor burn with chamber pressure and
case heating, results ($SR) and the 100 Hz curve on request ($SC).
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
        self._stand_reset()

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
            if len(p) > 1 and p[1].upper() in ("ZENITH", "ARIADNA", "STAND") and p[1].upper() != self.mode:
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
        elif c == "CMD" and len(p) >= 2 and self.mode == "STAND":
            self._stand_cmd(p[1].upper(), p[2:])
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
        if self.mode == "STAND":
            self._stand(t)
            if t >= self._next_g:
                self._next_g = t + 1.0
                self.line.emit(f"$G,{int(t * 7)},0,90,0,-1,{1 if self.sp else 0},0,-1,STAND,0,1,1")
            return
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

    # ---------------------------------------------------------------- thrust stand (rampa)
    ST_NAMES = ["PING", "ARM", "DISARM", "FIRE", "ABORT", "TARE", "RECORD", "STOP", "RESULT", "CURVE", "BUZZER",
                "RECTIME", "CAL", "PZERO"]
    ALIAS = {"SAFE": "DISARM", "COUNTDOWN": "FIRE", "CD": "FIRE", "REC": "RECORD", "LOCATE": "BUZZER", "NOP": "PING"}

    def _stand_reset(self):
        self.ss = 0                 # sl_state_t
        self.sp = None              # pending [seq, name, args, sent]
        self.s_seq = 0
        self.s_ack = (0, 0)
        self.s_t = 0.0              # time of entering the state
        self.s_t0 = None
        self.s_fire_ms, self.s_ch, self.s_rec = 1000, 1, 8
        self.s_cold = 0.0
        self.s_test = 4
        self.s_res = None
        self.s_curve: list = []
        self.s_peak = 0.0
        self.s_hang = False
        self.s_faults = 0
        self.s_next = 0.0
        self.s_reply: list = []
        self.s_buzz = False
        self.s_tared = False

    def _stand_cmd(self, name: str, args: list):
        name = self.ALIAS.get(name, name)
        if name not in self.ST_NAMES:
            self.line.emit(f"#ERR unknown STAND command {name}")
            return
        if self.sp and self.sp[1] == "ABORT" and name != "ABORT":
            self.line.emit("#ERR abort pending")
            return
        self.s_seq = self.s_seq % 255 + 1
        self.sp = [self.s_seq, name, args, 0]
        self.line.emit(f"$U,{self.s_seq},{name},2,QUEUED,0,0")
        self.line.emit(f"#OK queued {name} seq {self.s_seq}")

    def _stand_force(self, tb: float) -> float:
        """Motor model: 0.25 s ignition delay, spike, regressive 2.4 s burn."""
        if tb < 0.25 or tb > 2.9:
            return 0.0
        x = tb - 0.25
        return 210 * (x / 0.08) if x < 0.08 else 210 - 55 * (x - 0.08) / 2.5 if x < 2.5 else max(0.0, 155 * (1 - (x - 2.5) / 0.15))

    def _stand_exec(self, name: str, args: list, t: float) -> int:
        a0 = float(args[0]) if args else 0.0
        if name == "PING":
            return 1
        if name == "ABORT":
            if self.ss in (4, 5):
                self.ss = 5
            else:
                self.ss, self.s_t, self.s_t0 = 6, t, None
            return 5
        if name == "DISARM":
            if self.ss in (4, 5, 7):
                return 2
            self.ss, self.s_t0, self.s_faults = 0, None, 0
            return 1
        if name == "ARM":
            if self.ss != 0:
                return 2
            self.s_ch = int(a0) or 1
            self.s_fire_ms = int(float(args[1])) if len(args) > 1 else 1000
            if self.s_ch != 1:          # the demo igniter is on output 1
                self.s_faults = 0x04
                return 3
            self.ss, self.s_t, self.s_hang, self.s_faults = 1, t, False, 0
            return 1
        if name == "FIRE":
            if self.ss != 2:
                return 2
            self.ss, self.s_t, self.s_t0 = 3, t, t + a0
            return 1
        if name in ("TARE", "PZERO", "CAL"):
            if name == "TARE":
                self.s_tared = True
            return 1 if self.ss in (0, 2) else 2
        if name == "RECORD":
            if self.ss != 0:
                return 2
            self.ss, self.s_t, self.s_t0, self.s_cold = 7, t, t, a0
            self.s_curve, self.s_peak = [], 0.0
            return 1
        if name == "STOP":
            if self.ss in (5, 7):
                self.s_cold = 0.0
                self.s_rec = 0
                return 1
            return 2
        if name == "RECTIME":
            self.s_rec = int(a0)
            return 1
        if name == "BUZZER":
            self.s_buzz = a0 != 0
            return 1
        if name == "RESULT":
            if not self.s_res:
                return 2
            self.s_reply.append(("SR", 0))
            return 1
        if name == "CURVE":
            if not self.s_res:
                return 2
            self.s_reply.append(("SC", int(a0)))
            return 1
        return 4

    def _stand_finish(self, t: float, cold: bool):
        fired = not cold
        thr = max(2.0, 0.05 * self.s_peak)
        t0i = 0 if cold else 100
        above = [i for i, (f, _p) in enumerate(self.s_curve) if f > thr and i >= t0i]
        ign = bool(above) and self.s_peak > 2.0
        burn = (above[-1] - above[0] + 1) / 100 if ign else 0.0
        imp = sum(self.s_curve[i][0] for i in range(above[0], above[-1] + 1)) / 100 if ign else 0.0
        self.s_test += 1
        flags = (1 if fired else 0) | (2 if ign else 0) | 8 | 16 | (32 if cold else 0) | 64
        tpk = (max(range(len(self.s_curve)), key=lambda i: self.s_curve[i][0]) - t0i) / 100 if self.s_curve else 0
        pmax = max((p for _f, p in self.s_curve), default=0.0)
        self.s_res = [self.s_test, flags, self.s_ch, f"{self.s_peak:.3f}", f"{imp:.4f}", f"{burn:.3f}",
                      f"{imp / burn if burn else 0:.3f}", f"{tpk:.3f}", f"{(above[0] - t0i) / 100 if ign else 0:.3f}",
                      f"{pmax:.2f}", "61.5", "24.0", len(self.s_curve), 100, len(self.s_curve) * 20, 2000,
                      -1 if cold else t0i]
        self.s_hang = fired and not ign
        self.ss, self.s_t, self.s_t0 = 0, t, None
        self.s_reply += [("SR", 0), ("SR", 0)]

    def _stand(self, t: float):
        # recording at 100 Hz (the curve) while a test runs
        if self.ss in (3, 4, 5, 7) and self.s_t0 is not None:
            rel = t - self.s_t0
            if self.ss == 3 and rel >= -1.0 and not self.s_curve:
                self.s_curve = []
            if rel >= -1.0 or self.ss == 7:
                n_should = int((rel + (0 if self.ss == 7 else 1.0)) * 100)
                while len(self.s_curve) < n_should:
                    tb = len(self.s_curve) / 100 - (0 if self.ss == 7 else 1.0)
                    f = self._stand_force(tb) if self.ss != 7 else 0.0
                    self.s_curve.append((f, f * 1.6))
                    self.s_peak = max(self.s_peak, f)
        if self.ss == 1 and t - self.s_t > 1.2:
            self.ss, self.s_t = 2, t
        if self.ss == 3 and t >= self.s_t0:
            self.ss, self.s_t, self.s_curve, self.s_peak = 4, t, self.s_curve, 0.0
        if self.ss == 4 and t - self.s_t0 > self.s_fire_ms / 1000:
            self.ss = 5
        if self.ss == 5 and t - self.s_t0 > self.s_rec:
            self._stand_finish(t, False)
        if self.ss == 7 and t - self.s_t0 > self.s_cold:
            self._stand_finish(t, True)

        # station -> rampa: the pending command goes right after a TLM
        if self.sp and self.sp[3] == 0 and t >= self.s_next - 0.15:
            self.sp[3] = 1
            seq, name, args, _ = self.sp
            self.line.emit(f"$U,{seq},{name},2,SENT,0,1")
            rc = self._stand_exec(name, args, t)
            self.s_ack = (seq, rc)
            if name == "CURVE" and rc == 1:
                self.line.emit(f"$U,{seq},{name},2,ACK,1,1")
                self.sp = None
        for kind, arg in self.s_reply:
            if kind == "SR" and self.s_res:
                self.line.emit("$SR," + ",".join(str(v) for v in self.s_res))
            elif kind == "SC" and self.s_res:
                pts = [(f, p) for f, p in self.s_curve[arg:arg + 16]]
                vals = [f"{f:.2f}" for f, _p in pts] + [f"{p:.1f}" for _f, p in pts]
                self.line.emit(f"$SC,{self.s_test},{arg},{len(pts)}," + ",".join(vals))
        self.s_reply = []

        if t < self.s_next:
            return
        fast = self.ss not in (0, 6)
        self.s_next = t + (0.2 if fast else 0.5)
        rel = int((t - self.s_t0) * 1000) if self.s_t0 is not None and self.ss in (3, 4, 5, 7) else 0
        tb = t - self.s_t0 if self.s_t0 is not None and self.ss in (4, 5) else -1.0
        dt = 0.2 if fast else 0.5
        fs = [self._stand_force(tb - dt * (9 - i) / 10) if self.ss in (4, 5) else 0.0 for i in range(10)]
        fs = [f + (0.15 * math.sin(t * 13 + i)) for i, f in enumerate(fs)]
        flags = 0x01 | 0x02 | 0x20 | 0x40 | 0x80 | 0x100 | 0x200 | 0x400
        flags |= (0x04 | 0x08) if self.ss == 4 else 0
        flags |= 0x10 if self.ss in (3, 4, 5, 7) else 0
        flags |= 0x800 if self.s_res else 0
        flags |= 0x1000 if self.s_hang else 0
        flags |= 0x4000 if self.s_buzz else 0
        flags |= 0x8000 if self.s_tared else 0
        p = fs[-1] * 1.6
        t1 = 22.0 + (min(40.0, (t - self.s_t0) * 14) if self.s_t0 is not None and self.ss in (4, 5) else 0)
        prog = int(min(100, (t - self.s_t) / 1.2 * 100)) if self.ss == 1 else (int(min(100, rel / 300)) if rel > 0 else 0)
        seq, rc = self.s_ack
        vals = [-61, "9.5", self.ss, self.s_faults, flags, seq, rc, "8.02", "4.99", rel] + [f"{f:.2f}" for f in fs] + \
               [f"{max(fs):.2f}", f"{p:.1f}", f"{p:.1f}", f"{t1:.1f}", "24.0", self.s_test, 91, prog, 2000, self.s_ch, 1]
        self.line.emit("$ST," + ",".join(str(v) for v in vals))
        if self.sp and self.sp[3]:
            s2, name, _a, _ = self.sp
            self.line.emit(f"$U,{s2},{name},2,ACK,{rc},1")
            self.sp = None
