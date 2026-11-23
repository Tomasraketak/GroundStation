"""Ground station protocol and the two rocket profiles — no Qt in here.

The station (firmware/Ground_Station) speaks text lines, docs/USB_PROTOCOL.md:

    common    $ID  $MODE  $G (1 Hz)  $U (uplink)  #OK / #ERR
    ZENITH    $T telemetry, $B rampa beacon          (zenith_link.h)
    ARIADNA   $AT telemetry, $AS sync / ACK / rampa   (ariadna_link.h)
    STAND     $ST telemetry, $SR results, $SC curve   (stand_link.h, rampa = thrust stand)

A `RocketLink` per rocket keeps the newest raw lines and turns them into one
normalised snapshot (`Snap`) the window draws without knowing which rocket
it is: state, altitude, speeds, attitude, GPS, battery, pyro channels, the
launch-pad (rampa), the RX window, the last ACK, the pre-launch checks and a
few rocket-specific tiles. Everything a rocket does differently lives in
its profile class (ZenithLink / AriadnaLink).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

MODES = ("ZENITH", "ARIADNA", "STAND")

# zl_ul_rc_t / ariadna_ul_rc_t (identical 0..6, Zenith adds 7)
UL_RC = ["–", "OK", "odmítnuto: raketa letí", "odmítnuto: špatný stav", "odmítnuto: rampa",
         "odmítnuto: token / argument", "PŘERUŠENO (ABORT)", "odmítnuto: raketa NO-GO"]
UL_WHAT = {"QUEUED": "ve frontě – čeká na RX okno", "SENT": "odesláno", "SENT2": "odesláno 2×",
           "FAILED": "NEPOTVRZENO", "CANCELLED": "zrušeno"}
RAMPA_STATES = ["OFFLINE", "IDLE", "READY", "COUNTDOWN", "COMMITTED", "FIRED", "ABORT", "FAULT"]
RAMPA_FAULTS = ["nízká baterie", "klíč vypnut", "rozbuška bez kontinuity", "timeout rakety", "raketa NO-GO",
                "HW rampy", "tlačítko ABORT", "chyba časování"]


def to_float(s, default: float = float("nan")) -> float:
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def to_int(s, default: int = 0) -> int:
    try:
        return int(float(s))
    except (TypeError, ValueError):
        return default


def bits(value: int, names: list[str]) -> list[str]:
    return [n for i, n in enumerate(names) if value & (1 << i)]


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(min(1.0, a)))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


# ----------------------------------------------------------------------------
#  Snapshot
# ----------------------------------------------------------------------------
@dataclass
class Check:
    name: str
    level: str          # ok / warn / bad / info / idle
    detail: str


@dataclass
class Snap:
    """What the window shows. None = not known for this rocket / not yet."""
    rocket: str = ""
    have: bool = False              # any telemetry yet
    age: float = 1e9                # s since the last telemetry packet
    rssi: float | None = None
    snr: float | None = None
    state: int = -1
    state_name: str = "BEZ DAT"
    state_cz: str = ""
    state_level: str = "idle"       # banner colour
    in_flight: bool = False
    on_pad: bool = False            # states where the Go/No-Go check applies
    alt: float | None = None
    vz: float | None = None
    vh: float | None = None
    acc_g: float | None = None
    max_alt: float | None = None
    tilt: float | None = None
    tilt_az: float | None = None
    nose_up: bool | None = None
    lat: float | None = None
    lon: float | None = None
    sats: int | None = None
    vbat: float | None = None
    t_rem: float | None = None      # s to T-0 (synchronised countdown)
    t_flight: float | None = None   # s since liftoff
    go: bool | None = None
    go_text: str = ""
    prepared: bool = False          # PREPARE ("launch soon") in effect on the rocket
    armed: bool | None = None
    pyro: list = field(default_factory=list)      # [(label, continuity, fired)]
    rampa_text: str = "–"
    rampa_level: str = "idle"
    rampa_ready: bool = False
    rampa_alone: bool = False       # ZENITH: the pad's own beacon, no rocket on the pogo
    win_in: float | None = None     # s to the next RX window (from the packet)
    win_final: bool = False
    ack_seq: int | None = None
    ack_rc: int | None = None
    checks: list = field(default_factory=list)    # [Check]
    tiles: list = field(default_factory=list)     # [(key, title, value, sub, level)]
    strips: list = field(default_factory=list)    # [(title, [(label, on, warn)])]
    events: int = 0                 # rocket event bits (narrator)
    event_names: list = field(default_factory=list)
    pad_hint: tuple | None = None   # (lat, lon) of the pad, from the rocket


# ----------------------------------------------------------------------------
#  Profiles
# ----------------------------------------------------------------------------
class RocketLink:
    KEY = ""
    TITLE = ""
    SUBTITLE = ""
    TLM_HZ = 5
    CD_MIN, CD_MAX = 10, 3600
    PYRO_CHANNEL_SELECT = False     # COUNTDOWN carries the rampa igniter channel
    STATE_NAMES: list[str] = []
    STATE_CZ: dict[str, str] = {}
    LIFTOFF_STATE = -1
    ABORT_STATE = -1
    # (label, command, confirm question or None, kind)
    COMMANDS: list[tuple] = []
    EVENT_SPEECH: dict[str, str] = {}
    STATE_SPEECH: dict[int, str] = {}

    def __init__(self):
        self.t: dict = {}
        self.t_rx = 0.0
        self.max_alt = 0.0
        self.track_reset()

    def track_reset(self):
        self.max_alt = 0.0

    def feed(self, tag: str, p: list[str], now: float) -> bool:
        """Takes one parsed line; True when it was this rocket's telemetry."""
        raise NotImplementedError

    def snapshot(self, now: float) -> Snap:
        raise NotImplementedError

    def state_name(self, i: int) -> str:
        return self.STATE_NAMES[i] if 0 <= i < len(self.STATE_NAMES) else f"?{i}"

    def countdown_cmd(self, seconds: int, channel: int = 1) -> str:
        return f"COUNTDOWN {seconds}"

    def ack_text(self, rc: int) -> str:
        return UL_RC[rc] if 0 <= rc < len(UL_RC) else str(rc)

    OK_RCS = (1, 6)                 # result codes that mean "done as asked"


# ================================================================ ZENITH
class ZenithLink(RocketLink):
    KEY = "ZENITH"
    TITLE = "ZENITH"
    SUBTITLE = "Sapphire · Zenith v2.4"
    TLM_HZ = 5
    CD_MIN, CD_MAX = 10, 3600
    STATE_NAMES = ["BOOT", "IDLE", "READY", "COUNTDOWN", "BOOST", "COAST", "DROGUE", "MAIN", "LANDED", "ABORT", "DIAG"]
    STATE_CZ = {"BOOT": "start počítače", "IDLE": "na rampě – bezpečno", "READY": "připraveno ke startu",
                "COUNTDOWN": "odpočet", "BOOST": "motor hoří", "COAST": "setrvačný let", "DROGUE": "brzdicí padák",
                "MAIN": "hlavní padák", "LANDED": "přistáno", "ABORT": "PŘERUŠENO", "DIAG": "diagnostika (USB)"}
    FLIGHT = {4, 5, 6, 7}
    PAD = {1, 2, 3}
    LIFTOFF_STATE = 4
    ABORT_STATE = 9
    T_FIELDS = ["rssi", "snr", "state", "flags", "t_ms", "lat", "lon", "alt", "t_rem_ms", "vz", "vh", "acc_g", "max_alt",
                "vbat", "next_rx", "main_alt", "tilt", "mach", "sats", "pyro", "health", "rampa_state", "rampa_flags",
                "ul_seq", "ul_rc", "err", "win_final"]
    B_FIELDS = ["rssi", "snr", "state", "flags", "t_ms", "vbat", "umb_ma", "c1", "c2", "c3", "next_rx", "faults",
                "ul_seq", "ul_rc", "fw"]
    ERRORS = ["OK", "nekomunikuje Position MCU", "navigace neplatná", "rampa nekomunikuje", "rampa není připravena",
              "nízké napětí baterie", "PYRO1 (drogue) bez kontinuity", "PYRO2 (main) bez kontinuity",
              "PYRO3 (záloha) bez kontinuity", "RBF zasunut (SAFE)", "chybí flash", "I2C periferie (INA/ADS/PCA)",
              "přepnutí na baterii selhalo", "pyro lišta se nezapnula", "rampa nepotvrdila COMMIT",
              "HANGFIRE – motor nezažehl", "přerušeno ze země", "přerušeno rampou", "raketa příliš nakloněná"]
    HEALTH = ["Position MCU", "Navigace", "Baro důvěra", "GPS fix", "Log flash", "I2C HW", "Kamera", "Bzučák"]
    COMMANDS = [
        ("SAFE / zpět do IDLE", "SAFE", None, ""),
        ("Pad reference", "PADREF", None, ""),
        ("Bzučák ZAP", "LOCATE 1", None, ""),
        ("Bzučák VYP", "LOCATE 0", None, ""),
        ("Kamera nahrávat", "CAMERA 1", None, ""),
        ("Kamera stop", "CAMERA 0", None, ""),
        ("Restart rakety", "REBOOT", "Restartovat Master MCU rakety? (raketa ho přijme jen na zemi)", "danger"),
        ("Rampa: napájení rakety ZAP", "ROCKETPWR 1 RAMPA", None, ""),
        ("Rampa: napájení rakety VYP", "ROCKETPWR 0 RAMPA", "Vypnout napájení rakety z rampy (pogo 5 V)?", "danger"),
    ]
    STATE_SPEECH = {4: "Liftoff", 5: "Burnout", 6: "Apogee. Drogue", 7: "Main parachute", 8: "Landed", 9: "Abort"}

    def track_reset(self):
        super().track_reset()
        self.b: dict = {}
        self.b_rx = 0.0
        self.pad_acc: list = []

    def feed(self, tag, p, now):
        if tag == "$T" and len(p) >= len(self.T_FIELDS) + 1:
            self.t = {k: p[i + 1] for i, k in enumerate(self.T_FIELDS)}
            self.t_rx = now
            st = to_int(self.t["state"])
            if st in (1, 2) and to_float(self.t["lat"], 0.0) != 0.0:
                self.pad_acc.append((to_float(self.t["lat"]), to_float(self.t["lon"])))
                del self.pad_acc[:-30]
            return True
        if tag == "$B" and len(p) >= len(self.B_FIELDS) + 1:
            self.b = {k: p[i + 1] for i, k in enumerate(self.B_FIELDS)}
            self.b_rx = now
        return False

    def snapshot(self, now):
        s = Snap(rocket=self.KEY)
        t, b = self.t, self.b
        bage = now - self.b_rx if b else 1e9
        s.age = now - self.t_rx if t else 1e9
        if not t:
            if b:
                self._rampa_alone(s, bage)
            return s
        s.have = True
        st = to_int(t["state"])
        name = self.state_name(st)
        s.state, s.state_name, s.state_cz = st, name, self.STATE_CZ.get(name, "")
        s.in_flight, s.on_pad = st in self.FLIGHT, st in self.PAD
        s.state_level = ("bad" if st == 9 else "accent" if st in self.FLIGHT else "ok" if st == 8
                         else "warn" if st == 3 else "plain" if st in (1, 2) else "idle")
        s.rssi, s.snr = to_float(t["rssi"]), to_float(t["snr"])
        s.alt, s.vz, s.vh = to_float(t["alt"]), to_float(t["vz"]), to_float(t["vh"])
        s.acc_g, s.tilt = to_float(t["acc_g"]), to_float(t["tilt"])
        s.max_alt = to_float(t["max_alt"])
        s.lat, s.lon, s.sats, s.vbat = to_float(t["lat"]), to_float(t["lon"]), to_int(t["sats"]), to_float(t["vbat"])
        rem = to_int(t["t_rem_ms"], -1)
        if name == "COUNTDOWN" and rem >= 0:
            s.t_rem = rem / 1000 - s.age
        elif rem >= 0 and st in self.FLIGHT:
            s.t_flight = rem / 1000 + s.age
        elif rem >= 0 and st == 8:
            s.t_flight = rem / 1000
        flags, health, pyro = to_int(t["flags"]), to_int(t["health"]), to_int(t["pyro"])
        err = to_int(t["err"])
        s.go = bool(flags & 0x80) if s.on_pad else None
        s.prepared = st in (2, 3)
        err_txt = self.ERRORS[err] if 0 <= err < len(self.ERRORS) else f"chyba {err}"
        if s.on_pad:
            s.go_text = "GO" if s.go else "NO-GO: " + err_txt
        elif st == 9:
            s.go_text = "Důvod přerušení: " + err_txt
        s.armed = bool(flags & 0x01)
        names = ["PYRO1 drogue", "PYRO2 main", "PYRO3 záloha"]
        s.pyro = [(names[i], bool(pyro & (1 << i)), bool(pyro & (8 << i))) for i in range(3)]
        s.win_in = to_int(t["next_rx"]) / 1000 - s.age
        s.win_final = to_int(t["win_final"]) == 1
        s.ack_seq, s.ack_rc = to_int(t["ul_seq"]), to_int(t["ul_rc"])
        if st in (1, 2) and len(self.pad_acc) >= 5:
            s.pad_hint = (sum(a for a, _ in self.pad_acc) / len(self.pad_acc),
                          sum(b_ for _, b_ in self.pad_acc) / len(self.pad_acc))

        # rampa as the rocket sees it over the pogo
        rs, rf = to_int(t["rampa_state"]), to_int(t["rampa_flags"])
        if rs == 0:
            s.rampa_text = "OFFLINE (pogo odpojeno – po startu normální)"
            s.rampa_level = "idle" if st in self.FLIGHT or st == 8 else "warn"
        else:
            cont = "".join(str(i + 1) if rf & (2 << i) else "-" for i in range(3))
            s.rampa_ready = bool(rf & 0x80)
            s.rampa_text = (f"{RAMPA_STATES[rs] if rs < len(RAMPA_STATES) else rs} · klíč {'ZAP' if rf & 1 else 'vyp'}"
                            f" · rozbuška {cont} · {'připravena' if rf & 0x80 else 'NEPŘIPRAVENA'}"
                            f" · napájení rakety {'ZAP' if rf & 0x40 else 'vyp'}")
            s.rampa_level = "bad" if rs in (6, 7) else "ok" if s.rampa_ready else "warn"
        if bage < 5:
            s.rampa_text += f" · baterie rampy {to_float(b.get('vbat')):.2f} V"

        # checks (the rocket's own GO flag is the verdict; these say why)
        def ok(c):
            return "ok" if c else "bad"
        mach = to_float(t["mach"])
        s.checks = [
            Check("Position MCU / navigace", ok(health & 1 and health & 2),
                  ("spojení " + ("OK" if health & 1 else "CHYBÍ")) + " · řešení " + ("platné" if health & 2 else "NEPLATNÉ")),
            Check("GPS", "ok" if health & 8 else "warn", f"{'fix' if health & 8 else 'bez fixu'} · {s.sats} sat"),
            Check("Barometr", "ok" if health & 4 else "warn", "důvěryhodný" if health & 4 else "nedůvěryhodný (v letu normální)"),
            Check("Log flash / I2C", ok(health & 16 and health & 32),
                  ("flash OK" if health & 16 else "flash CHYBÍ") + " · " + ("INA/ADS/PCA OK" if health & 32 else "I2C CHYBA")),
            Check("Baterie", "bad" if err == 5 else "ok", f"{s.vbat:.2f} V" + (" · na baterii" if flags & 0x20 else "")),
            Check("PYRO1 · drogue", "info" if pyro & 8 else ok(pyro & 1), "vypáleno" if pyro & 8 else "kontinuita OK" if pyro & 1 else "BEZ KONTINUITY"),
            Check("PYRO2 · main", "info" if pyro & 16 else ok(pyro & 2), "vypáleno" if pyro & 16 else "kontinuita OK" if pyro & 2 else "BEZ KONTINUITY"),
            Check("PYRO3 · záloha", "info" if pyro & 32 else ("ok" if pyro & 4 else "warn"),
                  "vypáleno" if pyro & 32 else "kontinuita OK" if pyro & 4 else "bez kontinuity (vyžaduje-li ji raketa → NO-GO)"),
            Check("RBF / pyro lišta", "ok" if flags & 0x02 else "bad",
                  ("RBF venku" if flags & 0x02 else "RBF ZASUNUT") + " · lišta " + ("ZAP" if pyro & 0x40 else "vyp")),
            Check("Svislost na rampě", "bad" if err == 18 else "ok" if s.tilt < 5 else "warn", f"{s.tilt:.1f}°"),
            Check("Rampa", "ok" if s.rampa_ready else s.rampa_level if s.rampa_level != "idle" else "warn",
                  s.rampa_text.split(" · baterie")[0]),
        ]
        bw = ("1" if flags & 4 else "-") + ("2" if flags & 8 else "-")
        s.tiles = [
            ("mach", "Mach", f"{mach:.2f}", "baro " + ("věří" if health & 4 else "NEVĚŘÍ"), "warn" if 0.6 < mach < 1.4 else "ok"),
            ("main", "Hlavní padák", f"{to_int(t['main_alt'])} m", "výška vypuštění", "info"),
            ("batt", "Baterie", f"{s.vbat:.2f} V", "na baterii" if flags & 0x20 else "z poga", "bad" if err == 5 else "ok"),
            ("gps", "GPS", f"{s.sats} sat", "fix" if health & 8 else "bez fixu", "ok" if health & 8 else "warn"),
            ("bw", "Breakwire / pogo", bw, "pogo 5 V" if flags & 0x10 else "pogo odpojeno", "ok"),
            ("err", "Důvod NO-GO", err_txt if err else "žádný", f"kód {err}" if err else " ", "bad" if err else "ok"),
            ("cam", "Kamera / bzučák", ("REC" if health & 64 else "vyp") + " / " + ("ZAP" if health & 128 else "vyp"), " ", "info"),
            ("vh", "Vodorovná rychlost", f"{s.vh:.1f} m/s", " ", "ok"),
        ]
        s.strips = [
            ("Zdraví", [(n, bool(health & (1 << i)), i in (2, 3)) for i, n in enumerate(self.HEALTH[:6])]),
            ("Pyro · C = kontinuita, F = vypáleno", [("C1", bool(pyro & 1), True), ("C2", bool(pyro & 2), True),
                                                     ("C3", bool(pyro & 4), True), ("F1", bool(pyro & 8), False),
                                                     ("F2", bool(pyro & 16), False), ("F3", bool(pyro & 32), False),
                                                     ("LIŠTA", bool(pyro & 64), False), ("ARM", bool(flags & 1), False),
                                                     ("RBF", bool(flags & 2), False), ("GO", bool(flags & 0x80), False)]),
        ]
        return s

    def _rampa_alone(self, s: Snap, bage: float):
        b = self.b
        st = to_int(b["state"])
        flt = bits(to_int(b["faults"]), RAMPA_FAULTS)
        s.rampa_alone = True
        s.state_name, s.state_cz, s.state_level = "RAMPA BEZ RAKETY", "raketa není na pogu – rampa vysílá sama", "plain"
        s.rssi, s.snr = to_float(b["rssi"]), to_float(b["snr"])
        s.rampa_text = (f"{RAMPA_STATES[st] if st < len(RAMPA_STATES) else st} · baterie {to_float(b['vbat']):.2f} V"
                        f" · pogo {b['umb_ma']} mA · kontinuita {b['c1']}/{b['c2']}/{b['c3']} mV · napájení rakety "
                        f"{'ZAP' if to_int(b['flags']) & 0x40 else 'VYP'}" + (" · chyby: " + ", ".join(flt) if flt else ""))
        s.rampa_level = "bad" if flt else "ok"
        s.win_in = to_int(b["next_rx"]) / 1000 - bage
        s.ack_seq, s.ack_rc = to_int(b["ul_seq"]), to_int(b["ul_rc"])
        s.age = bage


# ================================================================ ARIADNA
# ariadna_link.h / daidalos_widgets.py: TriSense board -Y is the nose.
BODY_NOSE = (0.0, -1.0, 0.0)


def quat_from_smallest_three(qx: int, qy: int, qz: int):
    x, y, z = qx / 32767.0, qy / 32767.0, qz / 32767.0
    return math.sqrt(max(0.0, 1.0 - x * x - y * y - z * z)), x, y, z


def nose_vector(w, x, y, z, body=BODY_NOSE):
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    return tuple(sum(r[i][j] * body[j] for j in range(3)) for i in range(3))


def attitude(w, x, y, z):
    """tilt of the long axis from vertical, the compass direction the top leans to, nose up?"""
    e, n, u = nose_vector(w, x, y, z)
    u = max(-1.0, min(1.0, u))
    tilt = math.degrees(math.acos(abs(u)))
    if u < 0:
        e, n = -e, -n
    az = (math.degrees(math.atan2(e, n)) + 360) % 360
    return tilt, az, u >= 0


class AriadnaLink(RocketLink):
    KEY = "ARIADNA"
    TITLE = "ARIADNA"
    SUBTITLE = "Daidalos · Ariadna V2"
    TLM_HZ = 10
    CD_MIN, CD_MAX = 5, 3600
    PYRO_CHANNEL_SELECT = True
    STATE_NAMES = ["BOOT", "IDLE", "ARMED", "ASCENT", "APOGEE", "DESCENT", "GUIDANCE", "LANDED", "RECOVERY",
                   "ABORT", "TEST", "ERROR"]
    STATE_CZ = {"BOOT": "start, samotesty", "IDLE": "bezpečno, RBF zasunut", "ARMED": "na rampě, připraveno",
                "ASCENT": "stoupání", "APOGEE": "apogeum", "DESCENT": "pád k zemi", "GUIDANCE": "navedení – přistávací motor",
                "LANDED": "přistáno", "RECOVERY": "dohledání, beacon", "ABORT": "PŘERUŠENO", "TEST": "testovací režim",
                "ERROR": "CHYBA"}
    FLIGHT = {3, 4, 5, 6}
    PAD = {1, 2}
    LIFTOFF_STATE = 3
    ABORT_STATE = 9
    AT_FIELDS = ["rssi", "snr", "t_ms", "lat", "lon", "alt", "vz", "vh", "acc_g", "qx", "qy", "qz", "tvc_x", "tvc_y",
                 "thrust", "t_event_ds", "version", "state", "phase", "clamp", "vbat", "curr", "hdop", "sats", "pyro",
                 "health", "flags", "events", "uwb", "t_flight", "px", "py"]
    AS_FIELDS = ["rssi", "snr", "rocket_t_ms", "utc_ms", "next_rx", "rx_dur", "t_launch_rem_ms", "last_seq", "last_rc",
                 "rampa_state", "rampa_flags", "rampa_fault", "sync_flags", "rampa_vbat_mv"]
    PHASES = ["IDLE", "ASCENT", "DESCENT", "IGNITING", "BURN", "DONE"]
    EVENTS = ["APOGEE", "COVER", "IGN 1", "THRUST", "IGN 2", "LEGS", "LIDAR", "DOWN"]
    HEALTH = ["POS", "NAV", "SOLN", "FLASH", "LOG", "SERVO", "GPS", "POST"]
    ERRORS = ["žádná", "POST selhal", "spojení Position MCU", "spojení Navigation MCU", "flash", "servo sběrnice",
              "nízká baterie", "parametry nesynchronní"]
    UWB_MODES = {0: "--", 1: "LOS", 2: "2D", 3: "3D"}
    COMMANDS = [
        ("DISARM – zrušit přípravu", "SAFE", None, ""),
        ("PADREF – změřit rampu", "PADREF", None, ""),
        ("PING (synchronizace času)", "NOP", None, ""),
        ("Restart celé rakety", "REBOOT 0", "Restartovat všechny MCU rakety? (jen na zemi)", "danger"),
        ("Restart Position MCU", "REBOOT 1", "Restartovat Position MCU? (jen na zemi)", "danger"),
        ("Restart Navigation MCU", "REBOOT 2", "Restartovat Navigation MCU? (jen na zemi)", "danger"),
    ]
    EVENT_SPEECH = {"APOGEE": "Apogee", "COVER": "Cover released", "IGN 1": "Ignition", "THRUST": "Thrust confirmed",
                    "IGN 2": "Backup ignition", "LEGS": "Landing legs deployed", "LIDAR": "Lidar lock",
                    "DOWN": "Touchdown"}
    STATE_SPEECH = {3: "Liftoff", 9: "Abort", 11: "Error"}

    def track_reset(self):
        super().track_reset()
        self.s: dict = {}
        self.s_rx = 0.0
        self.pad_acc: list = []

    def countdown_cmd(self, seconds, channel=1):
        return f"COUNTDOWN {seconds} {channel}"

    def feed(self, tag, p, now):
        if tag == "$AT" and len(p) >= len(self.AT_FIELDS) + 1:
            self.t = {k: p[i + 1] for i, k in enumerate(self.AT_FIELDS)}
            self.t_rx = now
            st = to_int(self.t["state"])
            self.max_alt = max(self.max_alt, to_float(self.t["alt"], 0.0)) if st >= 3 else 0.0
            px, py = to_int(self.t["px"]), to_int(self.t["py"])
            lat, lon = to_float(self.t["lat"]), to_float(self.t["lon"])
            if st in (1, 2) and lat and abs(px) < 120 and abs(py) < 120:
                self.pad_acc.append((lat - py / 111320.0, lon - px / (111320.0 * math.cos(math.radians(lat)))))
                del self.pad_acc[:-50]
            return True
        if tag == "$AS" and len(p) >= len(self.AS_FIELDS) + 1:
            self.s = {k: p[i + 1] for i, k in enumerate(self.AS_FIELDS)}
            self.s_rx = now
        return False

    def snapshot(self, now):
        s = Snap(rocket=self.KEY)
        t, y = self.t, self.s
        yage = now - self.s_rx if y else 1e9
        s.age = now - self.t_rx if t else 1e9
        self._sync_part(s, y, yage)
        if not t:
            return s
        s.have = True
        st = to_int(t["state"])
        name = self.state_name(st)
        s.state, s.state_name, s.state_cz = st, name, self.STATE_CZ.get(name, "")
        s.in_flight, s.on_pad = st in self.FLIGHT, st in self.PAD
        s.state_level = {2: "plain", 3: "accent", 4: "accent", 5: "accent", 6: "accent", 7: "ok", 8: "info", 9: "bad",
                         10: "warn", 11: "bad"}.get(st, "idle")
        s.rssi, s.snr = to_float(t["rssi"]), to_float(t["snr"])
        s.alt, s.vz, s.vh, s.acc_g = to_float(t["alt"]), to_float(t["vz"]), to_float(t["vh"]), to_float(t["acc_g"])
        s.max_alt = self.max_alt if st >= 3 else s.alt
        s.tilt, s.tilt_az, s.nose_up = attitude(*quat_from_smallest_three(to_int(t["qx"]), to_int(t["qy"]), to_int(t["qz"])))
        s.lat, s.lon, s.sats, s.vbat = to_float(t["lat"]), to_float(t["lon"]), to_int(t["sats"]), to_float(t["vbat"])
        if st >= 3:
            s.t_flight = to_int(t["t_flight"]) + (s.age if st in self.FLIGHT else 0)
        flags, health, pyro, events, uwb = (to_int(t[k]) for k in ("flags", "health", "pyro", "events", "uwb"))
        err = (flags >> 5) & 7
        s.events, s.event_names = events, self.EVENTS
        s.armed = bool(flags & 1)
        cont, fired = pyro & 0x0F, (pyro >> 4) & 0x0F
        names = ["C1 přistávací motor", "C2 záložní zapalovač", "C3 nohy"]
        s.pyro = [(names[i], bool(cont & (1 << i)), bool(fired & (1 << i))) for i in range(3)]
        if len(self.pad_acc) >= 20:
            s.pad_hint = (sum(a for a, _ in self.pad_acc) / len(self.pad_acc),
                          sum(b for _, b in self.pad_acc) / len(self.pad_acc))

        hdop, curr = to_float(t["hdop"]), to_float(t["curr"])
        batt_lvl = "bad" if s.vbat < 7.0 else "warn" if s.vbat < 7.4 else "ok"
        gps_lvl = "bad" if s.sats < 5 or hdop > 2.5 else "warn" if hdop > 1.5 else "ok"
        uwb_n, uwb_mode, uwb_aided, uwb_alive = uwb & 0x0F, (uwb >> 4) & 3, bool(uwb & 0x40), bool(uwb & 0x80)
        fix = "OFFLINE" if not uwb_alive else f"FUSED {self.UWB_MODES[uwb_mode]}" if uwb_aided else "GATED OUT"
        uwb_lvl = ("idle" if not uwb_alive else "bad" if uwb_n < 2 else "warn" if uwb_n == 2 or not uwb_aided else "ok")
        failed = [n for i, n in enumerate(self.HEALTH) if not health & (1 << i)]
        err_txt = self.ERRORS[err]
        tilt_lvl = "ok" if s.tilt < 4 else "warn" if s.tilt < 10 else "bad"

        def pyro_check(i, need):
            c, f = bool(cont & (1 << i)), bool(fired & (1 << i))
            if f:
                return "info", "odpáleno"
            return ("ok" if c else ("bad" if need else "warn")), ("kontinuita OK" if c else "BEZ KONTINUITY")

        p1, p2 = pyro_check(0, True), pyro_check(1, False)
        s.checks = [
            Check("Subsystémy", "bad" if failed else "ok", "chybí: " + ", ".join(failed) if failed else "všech 8 v pořádku"),
            Check("Chyba", "bad" if err else "ok", err_txt),
            Check("Baterie", batt_lvl, f"{s.vbat:.2f} V · {curr:.1f} A (2S: < 7,0 V NO-GO)"),
            Check("GPS", gps_lvl, f"{s.sats} satelitů · HDOP {hdop:.1f}"),
            Check("UWB", "bad" if uwb_lvl == "idle" else uwb_lvl, f"{uwb_n} / 4 kotvy · {fix}"),
            Check("Pyro C1 · přistávací motor", *p1),
            Check("Pyro C2 · záložní zapalovač", *p2),
            Check("ARM / RBF", "info", f"ARM {'ANO' if flags & 1 else 'ne'} · RBF {'venku' if flags & 2 else 'zasunut'}"),
            Check("Svislost na rampě", tilt_lvl, f"{s.tilt:.1f}° · směr {s.tilt_az:.0f}°"),
            Check("Rampa", "ok" if s.rampa_ready else s.rampa_level if s.rampa_level not in ("idle",) else "warn",
                  s.rampa_text),
        ]
        bad = [c.name for c in s.checks if c.level == "bad"]
        rocket_ready = (to_int(y.get("sync_flags")) & 0x20) if y and yage < 15 else None
        if s.on_pad:
            s.go = not bad and (rocket_ready is None or bool(rocket_ready))
            s.go_text = "GO" if s.go else "NO-GO: " + (", ".join(bad) if bad else "raketa hlásí nepřipravena")
        elif st == 9:
            s.go_text = "PŘERUŠENO" + (f" · {err_txt}" if err else "")

        phase = to_int(t["phase"])
        tev = to_int(t["t_event_ds"], -32768)
        if tev == -32768:
            nxt = ("--", " ", "idle")
        elif phase == 2:
            nxt = (f"{tev / 10:.1f} s", "do ZÁŽEHU", "warn")
        else:
            nxt = (f"{tev / 10:.1f} s", "do PŘISTÁNÍ", "accent")
        burn = phase == 4
        s.tiles = [
            ("phase", "Fáze navedení", self.PHASES[phase] if 0 <= phase < len(self.PHASES) else "?", " ",
             "accent" if phase in (3, 4) else "info" if phase == 2 else "idle"),
            ("next", "Další událost", *nxt),
            ("tvc", "TVC X / Y", f"{to_float(t['tvc_x']):+.1f}° / {to_float(t['tvc_y']):+.1f}°", f"clamp {to_int(t['clamp'])} %",
             "accent" if burn else "idle"),
            ("thrust", "Tah", f"{to_int(t['thrust'])} N", f"clamp {to_int(t['clamp'])} %", "accent" if burn else "idle"),
            ("batt", "Baterie · proud", f"{s.vbat:.2f} V", f"{curr:.1f} A", batt_lvl),
            ("gps", "GPS", f"{s.sats} sat", f"HDOP {hdop:.1f}", gps_lvl),
            ("uwb", "UWB kotvy · fúze", f"{uwb_n} / 4", fix, uwb_lvl),
            ("pad", "Poloha vůči rampě", f"E{to_int(t['px']):+d} N{to_int(t['py']):+d} m", " ", "ok"),
            ("err", "Chyba", err_txt, f"kód {err}" if err else " ", "bad" if err else "ok"),
            ("vh", "Vodorovná rychlost", f"{s.vh:.1f} m/s", " ", "ok"),
        ]
        s.strips = [
            ("Události letu", [(n, bool(events & (1 << i)), False) for i, n in enumerate(self.EVENTS)]),
            ("Subsystémy", [(n, bool(health & (1 << i)), False) for i, n in enumerate(self.HEALTH)]),
            ("Pyro · C = kontinuita, F = odpáleno", [("C1", bool(cont & 1), True), ("C2", bool(cont & 2), True),
                                                     ("C3", bool(cont & 4), True), ("F1", bool(fired & 1), False),
                                                     ("F2", bool(fired & 2), False), ("F3", bool(fired & 4), False),
                                                     ("ARM", bool(flags & 1), False), ("RBF", bool(flags & 2), False)]),
        ]
        return s

    def _sync_part(self, s: Snap, y: dict, yage: float):
        if not y or yage > 30:
            s.rampa_text, s.rampa_level = "bez spojení (sync paket)", "idle"
            return
        rs, rf, flt = to_int(y["rampa_state"]), to_int(y["rampa_flags"]), to_int(y["rampa_fault"])
        sf = to_int(y["sync_flags"])
        if rs == 0:
            s.rampa_text, s.rampa_level = "OFFLINE", "warn"
        else:
            cont = "".join(str(i + 1) if rf & (2 << i) else "-" for i in range(3))
            faults = bits(flt, RAMPA_FAULTS)
            s.rampa_ready = rs == 2 and bool(rf & 1) and bool(rf & 2) and not flt
            s.rampa_text = (f"{RAMPA_STATES[rs] if rs < len(RAMPA_STATES) else rs} · klíč {'OK' if rf & 1 else 'VYP'}"
                            f" · C {cont} · {to_int(y['rampa_vbat_mv']) / 1000:.2f} V"
                            + (" · ODPOJENO" if sf & 0x04 else "") + (" · chyby: " + ", ".join(faults) if faults else ""))
            s.rampa_level = ("ok" if rs in (2, 3, 4) and not flt else "bad" if rs in (6, 7) or flt else "warn")
        s.prepared = bool(sf & 0x08) and yage < 15
        if yage < 3:
            rem = to_int(y["t_launch_rem_ms"], -1)
            if rem >= 0:
                s.t_rem = rem / 1000 - yage
        s.win_in = to_int(y["next_rx"]) / 1000 - yage
        s.win_final = bool(sf & 0x02)
        s.ack_seq, s.ack_rc = to_int(y["last_seq"]), to_int(y["last_rc"])


# ================================================================ STAND (rampa = thrust stand)
SL_STATES = ["SAFE", "ARMING", "ARMED", "COUNTDOWN", "FIRING", "BURN", "ABORT", "RECORD"]
SL_STATE_CZ = {"SAFE": "zajištěno – pyro vypnuto", "ARMING": "odjišťuji – mažu místo pro záznam",
               "ARMED": "ODJIŠTĚNO – čeká na povel k zážehu", "COUNTDOWN": "odpočet na rampě, záznam",
               "FIRING": "ZÁŽEH – proud do rozbušky", "BURN": "motor hoří – záznam běží", "ABORT": "PŘERUŠENO",
               "RECORD": "zkušební záznam bez zážehu"}
SL_STATE_TITLE = {"SAFE": "ZAJIŠTĚNO", "ARMING": "ODJIŠŤUJI…", "ARMED": "ODJIŠTĚNO", "COUNTDOWN": "ODPOČET",
                  "FIRING": "ZÁŽEH", "BURN": "MOTOR HOŘÍ", "ABORT": "PŘERUŠENO", "RECORD": "ZÁZNAM"}
SL_FAULTS = ["nízká baterie", "klíč vypnut", "rozbuška bez kontinuity", "ztráta spojení se stanicí",
             "ADS1220 / tenzometr", "flash (chybí / plná)", "tlačítko ABORT", "měřicí jádro / I2C"]
SL_FLAG = dict(KEY=1 << 0, CONT=1 << 1, ARM_OUT=1 << 2, FIRE_OUT=1 << 3, REC=1 << 4, ADS=1 << 5, FLASH=1 << 6,
               LC_CAL=1 << 7, TH1=1 << 8, TH2=1 << 9, PRESS=1 << 10, RESULT=1 << 11, HANGFIRE=1 << 12, BTN=1 << 13,
               BUZZER=1 << 14, TARED=1 << 15)
SL_T_NOBURN = -2147483648
SL_RF = dict(FIRED=1, IGNITION=2, ABORTED=4, CURVE=8, SAVED=16, COLD=32, LC_CAL=64)
SL_RC = ["–", "OK", "odmítnuto: špatný stav", "odmítnuto: blokování (viz chyby)", "odmítnuto: argument",
         "PŘERUŠENO (ABORT)", "zaneprázdněno – zkus znovu"]
MOTOR_CLASSES = "ABCDEFGHIJKLMNO"


def motor_class(impulse_ns: float, avg_n: float) -> str:
    """NAR/CAR designation from total impulse, e.g. 'G85' (A = 1.26-2.5 N.s, each letter doubles)."""
    if not impulse_ns or impulse_ns < 0.3125:
        return "–"
    if impulse_ns <= 0.625:
        letter = "1/4A"
    elif impulse_ns <= 1.25:
        letter = "1/2A"
    else:
        k = max(0, math.ceil(math.log2(impulse_ns / 2.5)))
        letter = MOTOR_CLASSES[k] if k < len(MOTOR_CLASSES) else "O+"
    return f"{letter}{max(0, round(avg_n or 0))}"


class StandLink(RocketLink):
    """The rampa alone as a static-fire thrust stand (MODE STAND)."""
    KEY = "STAND"
    TITLE = "TEST MOTORU"
    SUBTITLE = "rampa · tah, tlak, teploty"
    TLM_HZ = 5
    CD_MIN, CD_MAX = 3, 60
    STATE_NAMES = SL_STATES
    PAD = {0, 1, 2}
    FLIGHT = {4, 5}
    LIFTOFF_STATE = 4
    ABORT_STATE = 6
    COMMANDS: list = []
    STATE_SPEECH = {4: "Ignition", 6: "Abort"}
    OK_RCS = (1, 5)
    ST_FIELDS = (["rssi", "snr", "state", "faults", "flags", "ack_seq", "ack_rc", "vbat", "v5", "t_rel_ms"] +
                 [f"f{i}" for i in range(10)] +
                 ["fpeak", "p", "ppeak", "t1", "t2", "test_no", "flash_free", "progress", "sps", "ch", "cont", "servo"])
    SR_FIELDS = ["test_no", "flags", "ch", "peak", "impulse", "burn", "avg", "t_peak", "ign_delay", "pmax", "t1max",
                 "t2max", "curve_n", "curve_hz", "n_frames", "sps", "t0_idx"]

    def track_reset(self):
        super().track_reset()
        self.res: dict = {}
        self.res_rx = 0.0
        self.b: dict = {}
        self.b_rx = 0.0
        self.curve: dict = {}          # test_no -> {offset: (forces, pressures)}
        self.burn: dict = {}           # test_no -> {"n": blocks, "rate", "t_first", "a", "b", "blocks": {idx: (f, p, t1, t2)}}
        self.samples: list = []        # [(pc time, force)] of the latest TLM, oldest first
        self.prev_rx = 0.0

    def ack_text(self, rc: int) -> str:
        return SL_RC[rc] if 0 <= rc < len(SL_RC) else str(rc)

    def countdown_cmd(self, seconds, channel=1):
        return f"FIRE {seconds}"

    def feed(self, tag, p, now):
        if tag == "$ST" and len(p) >= len(self.ST_FIELDS) + 1:
            self.t = {k: p[i + 1] for i, k in enumerate(self.ST_FIELDS)}
            dt = min(0.6, max(0.05, now - self.prev_rx)) if self.prev_rx else 0.2
            self.prev_rx = self.t_rx = now
            fs = [to_float(self.t[f"f{i}"]) for i in range(10)]
            self.samples = [(now - dt * (9 - i) / 10.0, f) for i, f in enumerate(fs)]
            return True
        if tag == "$SR" and len(p) >= len(self.SR_FIELDS) + 1:
            self.res = {k: p[i + 1] for i, k in enumerate(self.SR_FIELDS)}
            self.res_rx = now
            return False
        if tag == "$SC" and len(p) >= 4:
            test, off, n = to_int(p[1]), to_int(p[2]), to_int(p[3])
            if len(p) >= 4 + 2 * n:
                fs = [to_float(x) for x in p[4:4 + n]]
                ps = [to_float(x) for x in p[4 + n:4 + 2 * n]]
                self.curve.setdefault(test, {})[off] = (fs, ps)
            return False
        if tag == "$SB" and len(p) >= 12:
            # $SB,test,idx,n_blocks,n,rate,flags,t_first_ms,burn_a_ms,burn_b_ms,t1,t2,f1..fn,p1..pn
            test, idx, nb, n = (to_int(x) for x in p[1:5])
            if len(p) >= 12 + 2 * n:
                d = self.burn.setdefault(test, {"blocks": {}})
                d.update(n=nb, rate=max(1, to_int(p[5], 50)), flags=to_int(p[6]), t_first=to_int(p[7]),
                         a=None if to_int(p[8]) == SL_T_NOBURN else to_int(p[8]),
                         b=None if to_int(p[9]) == SL_T_NOBURN else to_int(p[9]))
                d["blocks"][idx] = ([to_float(x) for x in p[12:12 + n]], [to_float(x) for x in p[12 + n:12 + 2 * n]],
                                    to_float(p[10]), to_float(p[11]))
            return False
        if tag == "$B" and len(p) >= len(ZenithLink.B_FIELDS) + 1:
            self.b = {k: p[i + 1] for i, k in enumerate(ZenithLink.B_FIELDS)}
            self.b_rx = now
        return False

    # ---- helpers for the stand panel
    def flag(self, name: str) -> bool:
        return bool(to_int(self.t.get("flags")) & SL_FLAG[name]) if self.t else False

    def faults(self) -> list[str]:
        return bits(to_int(self.t.get("faults")), SL_FAULTS) if self.t else []

    def servo_arming(self) -> bool:
        """The rampa closes the second arming switch with a servo at ARM (key sense open until then)."""
        return bool(to_int(self.t.get("servo")) & 1) if self.t else False

    def force_unit(self) -> str:
        return "N" if self.flag("LC_CAL") else "µV/V"

    def curve_points(self, test: int):
        """(t_s from T-0, force, pressure) lists of a downloaded curve, or None while incomplete."""
        r = self.res
        if not r or to_int(r.get("test_no")) != test:
            return None
        n, hz, t0 = to_int(r.get("curve_n")), max(1, to_int(r.get("curve_hz"), 100)), to_int(r.get("t0_idx"), -1)
        chunks = self.curve.get(test, {})
        f, pr = [None] * n, [None] * n
        for off, (fs, ps) in chunks.items():
            for i, (a, b) in enumerate(zip(fs, ps)):
                if 0 <= off + i < n:
                    f[off + i], pr[off + i] = a, b
        if n == 0 or any(v is None for v in f):
            return None
        base = t0 if t0 >= 0 else 0
        return [(i - base) / hz for i in range(n)], f, pr

    def burn_missing(self, test: int) -> list[int] | None:
        """Blocks of the cropped burn still missing (None = no block yet, size unknown)."""
        d = self.burn.get(test)
        if not d or "n" not in d:
            return None
        return [i for i in range(d["n"]) if i not in d["blocks"]]

    def burn_points(self, test: int):
        """(t_s from T-0, force, pressure, t1, t2) of the complete crop, or None."""
        d = self.burn.get(test)
        if not d or self.burn_missing(test) != []:
            return None
        t, f, pr, t1, t2 = [], [], [], [], []
        dt = 1.0 / d["rate"]
        for k in range(d["n"]):
            fs, ps, a, b = d["blocks"][k]
            for i, (x, y) in enumerate(zip(fs, ps)):
                t.append(d["t_first"] / 1000.0 + (k * 40 + i) * dt)
                f.append(x)
                pr.append(y)
                t1.append(a)
                t2.append(b)
        return t, f, pr, t1, t2

    def next_missing(self, test: int) -> int | None:
        n = to_int(self.res.get("curve_n")) if self.res else 0
        have = set()
        for off, (fs, _ps) in self.curve.get(test, {}).items():
            have.update(range(off, off + len(fs)))
        for i in range(0, n, 16):
            if i not in have:
                return i
        return None

    def snapshot(self, now):
        s = Snap(rocket=self.KEY)
        t = self.t
        s.age = now - self.t_rx if t else 1e9
        if not t:
            if self.b and now - self.b_rx < 15:
                s.state_name, s.state_cz, s.state_level = "RAMPA ČEKÁ", "rampa slyšet, přepínám ji do režimu test motoru…", "warn"
                s.rssi, s.snr = to_float(self.b.get("rssi")), to_float(self.b.get("snr"))
                s.age = now - self.b_rx
            return s
        s.have = True
        st = to_int(t["state"])
        name = self.state_name(st)
        s.state = st
        s.state_name = SL_STATE_TITLE.get(name, name)
        s.state_cz = SL_STATE_CZ.get(name, "")
        if self.flag("HANGFIRE") and st == 0:
            s.state_name, s.state_cz = "NEZAŽEHL!", "rozbuška odpálena, tah nezaznamenán – NEPŘIBLIŽUJ SE (5 min)"
        s.state_level = ("bad" if st == 6 or (st == 0 and self.flag("HANGFIRE")) else "accent" if st in (4, 5)
                         else "warn" if st in (1, 2, 3) else "info" if st == 7 else "plain")
        s.on_pad, s.in_flight = st in self.PAD, st in self.FLIGHT
        s.rssi, s.snr = to_float(t["rssi"]), to_float(t["snr"])
        s.vbat = to_float(t["vbat"])
        rel = to_int(t["t_rel_ms"])
        if st == 3:
            s.t_rem = -rel / 1000 - s.age
        elif st in (4, 5, 7) and rel:
            s.t_flight = rel / 1000 + s.age
        s.ack_seq, s.ack_rc = to_int(t["ack_seq"]), to_int(t["ack_rc"])
        s.armed = st in (2, 3, 4)
        cont = to_int(t["cont"])
        s.pyro = [(f"Rozbuška {i + 1}", bool(cont & (1 << i)), False) for i in range(3)]
        s.rampa_text = f"test motoru · {name}"
        s.rampa_level = "ok"
        return s


PROFILES = {"ZENITH": ZenithLink, "ARIADNA": AriadnaLink, "STAND": StandLink}


# ----------------------------------------------------------------------------
#  Station status
# ----------------------------------------------------------------------------
@dataclass
class Station:
    mode: str | None = None
    fw: str = ""
    rx_ok: int = 0
    rx_bad: int = 0
    win_locked: bool = False
    win_in_ms: int = -1
    pending: bool = False
    win_final: bool = False
    utc_valid: bool = False
    lora_ok: bool = True
    g_rx: float = 0.0

    def feed(self, tag: str, p: list[str], now: float):
        if tag == "$G" and len(p) >= 9:
            self.rx_ok, self.rx_bad = to_int(p[1]), to_int(p[2])
            self.win_locked, self.win_in_ms, self.pending = to_int(p[4]) == 1, to_int(p[5], -1), to_int(p[6]) == 1
            if len(p) >= 13:
                self.mode = p[9] if p[9] in MODES else self.mode
                self.win_final, self.utc_valid, self.lora_ok = to_int(p[10]) == 1, to_int(p[11]) == 1, to_int(p[12]) == 1
            self.g_rx = now
        elif tag == "$MODE" and len(p) >= 2 and p[1] in MODES:
            self.mode = p[1]
        elif tag == "$ID" and len(p) >= 3:
            self.fw = p[2]
            if len(p) >= 4 and p[3] in MODES:
                self.mode = p[3]


def parse_uplink(p: list[str], link: RocketLink) -> tuple[str, str, str] | None:
    """$U,seq,NAME,dest,WHAT,rc,tries -> (text, level, what)."""
    if len(p) < 7:
        return None
    seq, name, what, rc, tries = p[1], p[2], p[4], to_int(p[5]), to_int(p[6])
    txt = UL_WHAT.get(what, what)
    if what in ("SENT", "SENT2") and tries > 1:
        txt += f" (okno {tries})"
    if what == "ACK":
        txt = "POTVRZENO: " + link.ack_text(rc)
    level = "ok" if what == "ACK" and rc in link.OK_RCS else "bad" if what in ("ACK", "FAILED") else "warn"
    return f"#{seq} {name}: {txt}", level, what


def now() -> float:
    return time.time()
