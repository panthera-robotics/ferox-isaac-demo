#!/usr/bin/env python3
"""C-39 A/B verdict: did the robot stand, from the bridge's own report line.

Judged exactly the way every earlier C-39 run was judged -- base height and base
pitch off `[lowlevel-sim] t=... base_z=... pitch=...` -- so this number is comparable
to the whole existing evidence base rather than being a new instrument.

STANDS is deliberately strict: upright to within 15 deg of pitch AND roll, and the
base still above 0.55 m, sustained to the end of the run. A robot that is going over
slowly still reads FALLS.
"""
from __future__ import annotations

import math
import re
import sys

REPORT = re.compile(
    r"\[lowlevel-sim\] t=\s*([\d.]+).*?base_z=([+-][\d.]+)\s+pitch=([+-][\d.]+)\s+roll=([+-][\d.]+)")
RELEASE = re.compile(r"TEST RIG RELEASED at t=([\d.]+)")

UPRIGHT_RAD = math.radians(15.0)
MIN_Z = 0.55


def main() -> int:
    path, label = sys.argv[1], sys.argv[2]
    rows, released = [], None
    for line in open(path, errors="replace"):
        m = REPORT.search(line)
        if m:
            rows.append(tuple(float(g) for g in m.groups()))
        r = RELEASE.search(line)
        if r:
            released = float(r.group(1))

    print(f"# C-39 A/B verdict -- asset: {label}")
    print(f"source: {path}")
    if not rows:
        print("RESULT: NO DATA (no [lowlevel-sim] report lines -- the run did not reach PD)")
        return 2
    print(f"rig released at: {released if released is not None else 'NEVER'}")
    print(f"report samples: {len(rows)}  t {rows[0][0]:.2f} .. {rows[-1][0]:.2f}")

    if released is None:
        print("RESULT: INVALID (the rig never released, so the body was never free)")
        return 2

    after = [r for r in rows if r[0] >= released]
    if not after:
        print("RESULT: INVALID (no samples after release)")
        return 2

    print("\n t        base_z   pitch    roll")
    for t, z, p, ro in after[:: max(1, len(after) // 25)]:
        print(f" {t:7.2f} {z:+7.3f} {p:+7.3f} {ro:+7.3f}")
    t_end, z_end, p_end, r_end = after[-1]
    print(f"\nfinal: t={t_end:.2f} base_z={z_end:+.3f} "
          f"pitch={p_end:+.3f} ({math.degrees(p_end):+.1f} deg) "
          f"roll={r_end:+.3f} ({math.degrees(r_end):+.1f} deg)")

    upright = abs(p_end) <= UPRIGHT_RAD and abs(r_end) <= UPRIGHT_RAD
    tall = z_end >= MIN_Z
    held = min(z for _, z, _, _ in after[len(after) // 2:]) >= MIN_Z
    print(f"criteria: |pitch|<=15deg {abs(p_end)<=UPRIGHT_RAD}  "
          f"|roll|<=15deg {abs(r_end)<=UPRIGHT_RAD}  "
          f"base_z>={MIN_Z} {tall}  held through second half {held}")
    print(f"\nRESULT: {'STANDS' if (upright and tall and held) else 'FALLS'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
