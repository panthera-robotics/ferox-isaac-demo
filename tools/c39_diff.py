"""C-39 offline diff: twin runtime articulation vs the reference MJCF.

No GPU, no Isaac. Inputs are both CSVs written by the two capture paths:
  twin      isaac/twin/c39_capture.py  -> links_<variant>.csv   (needs the sim, once)
  reference tools/c39_mjcf_mass.py     -> links_mujoco_ref.csv  (anywhere)

Matches links by NAME and reports what is only on one side, because the twin carries
sensor and hand links the 29-DoF MJCF has no equivalent for and those are exactly the
rows that must not be silently dropped from a mass total.
"""
from __future__ import annotations

import argparse
import csv
import json


def load(p):
    out = {}
    with open(p) as fh:
        for r in csv.DictReader(fh):
            nm = r["link"]
            try:
                m = float(r["mass_kg"])
            except (TypeError, ValueError):
                continue
            def g(k):
                try:
                    return float(r[k])
                except (TypeError, ValueError, KeyError):
                    return None
            out[nm] = {"mass": m, "cx": g("com_local_x"), "cy": g("com_local_y"),
                       "cz": g("com_local_z"), "wx": g("world_x")}
    return out


ap = argparse.ArgumentParser()
ap.add_argument("twin_csv")
ap.add_argument("ref_csv")
ap.add_argument("--out", default="")
ap.add_argument("--top", type=int, default=15)
a = ap.parse_args()

T, R = load(a.twin_csv), load(a.ref_csv)
both = sorted(set(T) & set(R))
only_t = sorted(set(T) - set(R))
only_r = sorted(set(R) - set(T))

rows = []
for nm in both:
    dm = T[nm]["mass"] - R[nm]["mass"]
    dcx = (None if T[nm]["cx"] is None or R[nm]["cx"] is None
           else T[nm]["cx"] - R[nm]["cx"])
    rows.append({"link": nm, "twin_kg": round(T[nm]["mass"], 6),
                 "ref_kg": round(R[nm]["mass"], 6), "d_mass_kg": round(dm, 6),
                 "twin_com_x": T[nm]["cx"], "ref_com_x": R[nm]["cx"],
                 "d_com_x": None if dcx is None else round(dcx, 6)})

tm, rm = sum(v["mass"] for v in T.values()), sum(v["mass"] for v in R.values())
only_t_m = sum(T[n]["mass"] for n in only_t)
only_r_m = sum(R[n]["mass"] for n in only_r)
matched_d = sum(r["d_mass_kg"] for r in rows)

res = {
    "twin_total_kg": round(tm, 6), "ref_total_kg": round(rm, 6),
    "total_delta_kg": round(tm - rm, 6),
    "matched_links": len(both),
    "matched_mass_delta_kg": round(matched_d, 6),
    "twin_only_links": len(only_t), "twin_only_mass_kg": round(only_t_m, 6),
    "ref_only_links": len(only_r), "ref_only_mass_kg": round(only_r_m, 6),
    "by_abs_d_mass": sorted(rows, key=lambda r: -abs(r["d_mass_kg"]))[:a.top],
    "by_abs_d_com_x": sorted([r for r in rows if r["d_com_x"] is not None],
                             key=lambda r: -abs(r["d_com_x"]))[:a.top],
    "twin_only": [{"link": n, "mass_kg": round(T[n]["mass"], 6)} for n in
                  sorted(only_t, key=lambda n: -T[n]["mass"])][:a.top],
    "ref_only": [{"link": n, "mass_kg": round(R[n]["mass"], 6)} for n in
                 sorted(only_r, key=lambda n: -R[n]["mass"])][:a.top],
}
print(json.dumps(res, indent=2))
if a.out:
    open(a.out, "w").write(json.dumps({**res, "all_matched": rows}, indent=2))
