#!/usr/bin/env python3
"""Offscreen smoke test of the ground station GUI against the demo station.

Flies both rockets through the window's real command and parsing paths:

  ZENITH   PREPARE -> READY, COUNTDOWN -> T-, liftoff, apogee/drogue, main
  ARIADNA  PREPARE -> ARMED (+ sync "launch soon"), COUNTDOWN on rampa
           channel 2, hop, nose-first descent, landing burn, touchdown

plus switching the rocket from the window and from the station (button),
a Zenith-only command refused in Ariadna mode, ABORT, and a check that the
GUI's field lists match the firmware's printf formats.

    QT_QPA_PLATFORM=offscreen python3 test/gui_smoke.py [screenshot_dir]
"""
import os
import re
import sys
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["GROUNDSTATION_DATA"] = tempfile.mkdtemp()
os.environ["GROUNDSTATION_NO_WEBENGINE"] = "1"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gui"))

from PyQt6 import QtWidgets  # noqa: E402
import gs_protocol as gp  # noqa: E402
import gs_widgets as gw  # noqa: E402
import ground_station as gs  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else None
fails = 0


def check(cond, msg):
    global fails
    print(("   ok:   " if cond else "   FAIL: ") + msg)
    if not cond:
        fails += 1


def spin(app, seconds):
    end = time.time() + seconds
    while time.time() < end:
        app.processEvents()
        time.sleep(0.004)


def wait_for(app, cond, timeout):
    end = time.time() + timeout
    while time.time() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.004)
    return False


def shot(w, name):
    if OUT:
        for i in range(w.tabs.count()):
            w.tabs.setCurrentIndex(i)
            spin(app, 0.05)
            w.grab().save(os.path.join(OUT, f"{name}_{i + 1}.png"))
        w.tabs.setCurrentIndex(0)


# ------------------------------------------------------------- firmware <-> GUI
print("== field lists match the firmware printf formats")
ino = open(os.path.join(HERE, "..", "firmware", "Ground_Station", "Ground_Station.ino"), encoding="utf-8").read()


def n_fields(tag):
    m = re.search(r'out\("\$' + tag + r',([^"]*)"', ino)
    return len(re.findall(r"%[-+ 0-9.]*l?[duf]", m.group(1))) if m else -1


check(n_fields("T") == len(gp.ZenithLink.T_FIELDS), f"$T {n_fields('T')} == {len(gp.ZenithLink.T_FIELDS)}")
check(n_fields("B") == len(gp.ZenithLink.B_FIELDS), f"$B {n_fields('B')} == {len(gp.ZenithLink.B_FIELDS)}")
check(n_fields("AT") == len(gp.AriadnaLink.AT_FIELDS), f"$AT {n_fields('AT')} == {len(gp.AriadnaLink.AT_FIELDS)}")
check(n_fields("AS") == len(gp.AriadnaLink.AS_FIELDS), f"$AS {n_fields('AS')} == {len(gp.AriadnaLink.AS_FIELDS)}")

print("== attitude of the Ariadna quaternion (board -Y = nose)")
q = gp.quat_from_smallest_three(0, 0, 0)
check(abs(gp.attitude(*q)[0] - 90) < 0.1, "identity: board flat -> nose horizontal (90 deg)")
w0 = 0.70710678
check(gp.attitude(w0, -w0, 0, 0)[0] < 0.1 and gp.attitude(w0, -w0, 0, 0)[2], "mount quaternion -> nose up, tilt 0")

app = QtWidgets.QApplication(sys.argv)
gw.apply_theme(app, "light")
QtWidgets.QMessageBox.question = staticmethod(lambda *a, **k: QtWidgets.QMessageBox.StandardButton.Yes)
QtWidgets.QDialog.exec = lambda self: QtWidgets.QDialog.DialogCode.Accepted      # the typed START dialog

# ------------------------------------------------------------- ZENITH
print("== ZENITH: simulated mission (x10)")
w = gs.GroundStation(None, True, demo_speed=10.0, rocket="ZENITH")
w.resize(1536, 864)
w.show()
check(wait_for(app, lambda: w.link.t.get("state") == "1", 3), "telemetry parsed, rocket IDLE")
check(w.station.mode == "ZENITH", "station reports ZENITH")
w.send_prepare()
check(wait_for(app, lambda: w.link.t.get("state") == "2", 3), "PREPARE acknowledged -> READY")
check("POTVRZENO" in w.ul_lb.text(), "uplink ACK shown: " + w.ul_lb.text())
spin(app, 0.3)
check(w.link.snapshot(time.time()).prepared, "procedure: prepared")
for _l, chk in w.proc.values():
    if chk is not None:
        chk.setChecked(True)
spin(app, 0.3)
check(w.launch_btn.isEnabled(), "launch enabled once every step is OK")
w.cd_spin.setValue(30)
w.send_launch()
check(wait_for(app, lambda: w.link.t.get("state") == "3", 3), "COUNTDOWN running")
spin(app, 0.3)
check(w.clock.text().startswith("T−"), "countdown clock " + w.clock.text())
check("POTVRZENO" in w.launch_lb.text(), "launch confirmed: " + w.launch_lb.text())
shot(w, "zenith_countdown")
check(wait_for(app, lambda: w.link.t.get("state") in ("4", "5"), 5), "liftoff")
check(w.tabs.currentIndex() == 0, "page 1 shown at liftoff")
check(wait_for(app, lambda: w.link.t.get("state") == "6", 4), "apogee -> DROGUE")
snap = w.link.snapshot(time.time())
check((snap.max_alt or 0) > 2500, f"max altitude {snap.max_alt:.0f} m")
check(snap.pyro[0][2], "drogue fired flag")
shot(w, "zenith_flight")
check(wait_for(app, lambda: w.link.t.get("state") == "7", 12), "MAIN")

# ------------------------------------------------------------- switch from the window
print("== switch to ARIADNA from the window")
w.select_rocket("ARIADNA")
check(wait_for(app, lambda: w.station.mode == "ARIADNA", 2), "station switched to ARIADNA")
check(wait_for(app, lambda: w.link.KEY == "ARIADNA" and w.link.t.get("state") == "1", 3), "Ariadna telemetry, IDLE")
check(not w.ch_lb.isHidden() and w.main_row.isHidden(), "Ariadna controls (rampa channel, no main altitude)")
check(wait_for(app, lambda: bool(w.link.s), 3), "sync packet parsed")
w.cmd("MAINALT 300")
spin(app, 0.3)
check("Ariadna does not have this command" in w.log.toPlainText(), "Zenith-only command refused")

print("== ARIADNA: simulated hop (x10)")
w.send_prepare()
check(wait_for(app, lambda: w.link.t.get("state") == "2", 3), "PREPARE -> ARMED")
check(wait_for(app, lambda: w.link.snapshot(time.time()).prepared, 2), "sync says launch soon")
snap = w.link.snapshot(time.time())
check(snap.rampa_ready, "rampa READY: " + snap.rampa_text)
check(snap.go, "GO: " + snap.go_text)
for _l, chk in w.proc.values():
    if chk is not None:
        chk.setChecked(True)
spin(app, 0.3)
w.cd_spin.setValue(12)
w.ch_cb.setCurrentText("2")
w.send_launch()
check(wait_for(app, lambda: "POTVRZENO" in w.launch_lb.text(), 3), "countdown confirmed: " + w.launch_lb.text())
check(wait_for(app, lambda: w.link.snapshot(time.time()).t_rem is not None, 2), "T- from the sync packet")
shot(w, "ariadna_countdown")
seen, nose_down, burn_shot = set(), [], False
end = time.time() + 8
while time.time() < end and w.link.t.get("state") != "7":
    app.processEvents()
    st = w.link.t.get("state")
    seen.add(st)
    sn = w.link.snapshot(time.time())
    if st in ("5", "6") and sn.tilt is not None:
        nose_down.append(not sn.nose_up and sn.tilt < 10)
    if st == "6" and not burn_shot:
        burn_shot = True
        shot(w, "ariadna_burn")
    time.sleep(0.004)
seen.add(w.link.t.get("state"))
check({"3", "5", "6", "7"} <= seen, f"ASCENT, DESCENT, GUIDANCE, LANDED seen ({sorted(seen)})")
check(nose_down and all(nose_down), f"nose-first descent in {len(nose_down)} samples")
snap = w.link.snapshot(time.time())
check(snap.events & 0x80 and snap.events & 0x20, "events: legs + touchdown")
check(110 < (snap.max_alt or 0) < 130, f"apogee {snap.max_alt:.0f} m")
check("UDÁLOST: APOGEE" in w.log.toPlainText(), "event log")

# ------------------------------------------------------------- station button
print("== station switched by its button -> the window follows")
w.reader.send("MODE ZENITH")       # what the station reports after BL held 3 s
check(wait_for(app, lambda: w.key == "ZENITH" and w.rocket_btns["ZENITH"].isChecked(), 4), "window followed to ZENITH")
check(wait_for(app, lambda: w.link.t.get("state") == "1", 3), "Zenith telemetry again")
w.send_abort()
check("ABORT" in w.ul_lb.text(), "ABORT queued: " + w.ul_lb.text())
check(wait_for(app, lambda: "POTVRZENO" in w.ul_lb.text() or "odmítnuto" in w.ul_lb.text(), 3), "ABORT answered: " + w.ul_lb.text())

w.toggle_theme()
spin(app, 0.3)
if OUT:
    w.grab().save(os.path.join(OUT, "zenith_dark.png"))
w.toggle_theme()
log = w.log_path
w.close()
check(os.path.getsize(log) > 1000, "log file written")

print("\n%s (%d failures)" % ("GUI SMOKE PASSED" if not fails else "GUI SMOKE FAILED", fails))
sys.exit(1 if fails else 0)
