#!/usr/bin/env python3
"""Keep the protocol headers in step.

The two wire formats are owned by the rocket repositories:

    zenith_link.h   Zenith-flight-computer/firmware/common/
    ariadna_link.h  Daidalos_Landing_Rocket_Code/Ariadna_V2/common/
    stand_link.h    RampaCode/Rampa_MCU/            (rampa as a thrust stand)

firmware/common holds this repository's copy and firmware/Ground_Station a
second one (the Arduino builder does not follow "../common").

    python3 tools/sync_common.py          check: sketch copies == common, and
                                          common == the rocket repos when they
                                          are cloned next to this one
    python3 tools/sync_common.py --fix    copy common -> sketch
    python3 tools/sync_common.py --pull   copy the rocket repos -> common -> sketch
"""
from __future__ import annotations

import filecmp
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
COMMON = os.path.join(ROOT, "firmware", "common")
SKETCH = os.path.join(ROOT, "firmware", "Ground_Station")
UPSTREAM = {
    "zenith_link.h": os.path.join(ROOT, "..", "Zenith-flight-computer", "firmware", "common", "zenith_link.h"),
    "ariadna_link.h": os.path.join(ROOT, "..", "Daidalos_Landing_Rocket_Code", "Ariadna_V2", "common", "ariadna_link.h"),
    "stand_link.h": os.path.join(ROOT, "..", "RampaCode", "Rampa_MCU", "stand_link.h"),
}


def main() -> int:
    fix = "--fix" in sys.argv
    pull = "--pull" in sys.argv
    bad = 0
    for name, up in UPSTREAM.items():
        common = os.path.join(COMMON, name)
        sketch = os.path.join(SKETCH, name)
        if os.path.exists(up):
            if not filecmp.cmp(up, common, shallow=False):
                if pull:
                    shutil.copyfile(up, common)
                    print(f"pulled  {name} from {os.path.relpath(up, ROOT)}")
                else:
                    print(f"DIFFERS {name}: firmware/common vs {os.path.relpath(up, ROOT)} (--pull to take the rocket's)")
                    bad += 1
            else:
                print(f"ok      {name} == {os.path.relpath(up, ROOT)}")
        else:
            print(f"skip    {name}: {os.path.relpath(up, ROOT)} not cloned next to this repo")
        if not os.path.exists(sketch) or not filecmp.cmp(common, sketch, shallow=False):
            if fix or pull:
                shutil.copyfile(common, sketch)
                print(f"fixed   firmware/Ground_Station/{name}")
            else:
                print(f"DIFFERS firmware/Ground_Station/{name} vs firmware/common (--fix)")
                bad += 1
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
