"""C-39 item 1: does the twin's IMU wire TRACK a known tilt?

The torque-decay signature -- SONIC's commanded torque falling monotonically as the
robot goes over, never saturating -- is what an OPEN-LOOP controller looks like. Before
any mass/inertia work, prove the state SONIC reads is live and correctly framed.

The rig pitches the base by a commanded angle. Both `rt/lowstate.imu_state` and
`rt/secondary_imu` must follow it. Frozen, identity, wrong-sign, wrong-frame or
rate-decimated values on either is C-39.

Reports the pitch each source implies, so it can be differenced against the sim's own
printed ground truth rather than eyeballed.
"""
import argparse, json, math, time
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, IMUState_

ap = argparse.ArgumentParser()
ap.add_argument("--domain", type=int, default=0)
ap.add_argument("--iface", default="lo")
ap.add_argument("--seconds", type=float, default=24.0)
ap.add_argument("--hz", type=float, default=4.0)
ap.add_argument("--out", default="")
a = ap.parse_args()


def pitch_of(q):
    w, x, y, z = (float(v) for v in q)
    return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))


ChannelFactoryInitialize(a.domain, a.iface)
state = {"ls": None, "si": None, "n_ls": 0, "n_si": 0}


def on_ls(m):
    state["ls"] = m; state["n_ls"] += 1


def on_si(m):
    state["si"] = m; state["n_si"] += 1


ChannelSubscriber("rt/lowstate", LowState_).Init(on_ls, 32)
ChannelSubscriber("rt/secondary_imu", IMUState_).Init(on_si, 32)

rows = []
t0 = time.monotonic()
while time.monotonic() - t0 < a.seconds:
    time.sleep(1.0 / a.hz)
    ls, si = state["ls"], state["si"]
    if ls is None or si is None:
        continue
    lq = list(ls.imu_state.quaternion)
    sq = list(si.quaternion)
    rows.append({
        "t": round(time.monotonic() - t0, 3),
        "n_ls": state["n_ls"], "n_si": state["n_si"],
        "ls_quat": [round(float(v), 6) for v in lq],
        "ls_pitch_deg": round(pitch_of(lq), 4),
        "ls_gyro": [round(float(v), 5) for v in ls.imu_state.gyroscope],
        "ls_rpy_deg": [round(math.degrees(float(v)), 4) for v in ls.imu_state.rpy],
        "si_quat": [round(float(v), 6) for v in sq],
        "si_pitch_deg": round(pitch_of(sq), 4),
        "si_gyro": [round(float(v), 5) for v in si.gyroscope],
        "si_rpy_deg": [round(math.degrees(float(v)), 4) for v in si.rpy],
    })
    r = rows[-1]
    print(f"t={r['t']:6.2f}  lowstate.imu pitch={r['ls_pitch_deg']:+8.3f} deg "
          f"gyro={r['ls_gyro']}   secondary_imu pitch={r['si_pitch_deg']:+8.3f} deg "
          f"gyro={r['si_gyro']}", flush=True)

if not rows:
    print("NO IMU DATA RECEIVED", flush=True)
    raise SystemExit(1)

lsp = np.array([r["ls_pitch_deg"] for r in rows])
sip = np.array([r["si_pitch_deg"] for r in rows])
summary = {
    "samples": len(rows),
    "lowstate_pitch_range_deg": [round(float(lsp.min()), 4), round(float(lsp.max()), 4)],
    "lowstate_pitch_span_deg": round(float(lsp.max() - lsp.min()), 4),
    "secondary_pitch_range_deg": [round(float(sip.min()), 4), round(float(sip.max()), 4)],
    "secondary_pitch_span_deg": round(float(sip.max() - sip.min()), 4),
    "max_abs_diff_deg": round(float(np.abs(lsp - sip).max()), 4),
    "rows": rows,
}
print(json.dumps({k: v for k, v in summary.items() if k != "rows"}, indent=2), flush=True)
if a.out:
    open(a.out, "w").write(json.dumps(summary, indent=2))
