"""Subscribe to rt/lowcmd and report what the controller is ACTUALLY commanding.

C-39, 1b(4) PD parity.  The twin's fall trace shows |tau| falling monotonically as
the robot goes over -- 42 Nm holding, 31 the instant SONIC takes authority, 16 by
the time it is past 80 degrees, with sat=0/29 throughout.  A balancer losing a fight
commands MORE torque, not less.  So the question is whether SONIC sends the twin a
weaker command than it sends MuJoCo, or whether the two sims apply the same command
differently.  This is the instrument for the first half, and it is deliberately the
SAME instrument on both sims: one DDS subscriber, one code path, one output format.
"""
import argparse, json, sys, time
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

N = 29

ap = argparse.ArgumentParser()
ap.add_argument("--domain", type=int, default=0)
ap.add_argument("--iface", default="lo")
ap.add_argument("--seconds", type=float, default=6.0)
ap.add_argument("--label", default="TWIN")
ap.add_argument("--out", default="")
a = ap.parse_args()

ChannelFactoryInitialize(a.domain, a.iface)
kp = []; kd = []; qd = []; dqd = []; tff = []
n = 0

def _on(m):
    global n
    n += 1
    kp.append([m.motor_cmd[i].kp for i in range(N)])
    kd.append([m.motor_cmd[i].kd for i in range(N)])
    qd.append([m.motor_cmd[i].q for i in range(N)])
    dqd.append([m.motor_cmd[i].dq for i in range(N)])
    tff.append([m.motor_cmd[i].tau for i in range(N)])

sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
sub.Init(_on, 32)
time.sleep(a.seconds)

if not n:
    print(f"[{a.label}] NO rt/lowcmd RECEIVED in {a.seconds}s", flush=True)
    sys.exit(1)

kp = np.array(kp); kd = np.array(kd); qd = np.array(qd)
dqd = np.array(dqd); tff = np.array(tff)
out = {
    "label": a.label, "n_lowcmd": n, "hz": round(n / a.seconds, 2),
    # kp/kd are what the controller asks the actuator to be.  If these differ between
    # the two sims the controller is in a different internal state; if they match, the
    # divergence is downstream, in how each sim turns the command into a torque.
    "kp_mean": [round(float(v), 3) for v in kp.mean(0)],
    "kd_mean": [round(float(v), 3) for v in kd.mean(0)],
    "kp_distinct": sorted({round(float(v), 3) for v in kp.reshape(-1)}),
    "kd_distinct": sorted({round(float(v), 3) for v in kd.reshape(-1)}),
    "q_d_mean": [round(float(v), 4) for v in qd.mean(0)],
    "q_d_absmax": round(float(np.abs(qd).max()), 4),
    "dq_d_absmax": round(float(np.abs(dqd).max()), 4),
    "tau_ff_absmax": round(float(np.abs(tff).max()), 4),
    "tau_ff_nonzero": int((np.abs(tff) > 1e-9).sum()),
}
print(json.dumps(out, indent=2), flush=True)
if a.out:
    open(a.out, "w").write(json.dumps(out, indent=2))
