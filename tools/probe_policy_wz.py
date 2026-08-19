import torch, numpy as np, json
p="/workspace/ferox_isaac/checkpoints/g1/exported/policy.pt"
m=torch.jit.load(p, map_location="cpu"); m.eval()
H=5; D=96
# term layout per step: ang_vel(3) grav(3) cmd(3) qpos(29) qvel(29) act(29)
def build(wz, vx=0.0, vy=0.0):
    # per-term history blocks, oldest-first, matching run.py
    ang=np.zeros((H,3),np.float32)
    grav=np.tile(np.array([0,0,-1],np.float32),(H,1))
    cmd=np.tile(np.array([vx,vy,wz],np.float32),(H,1))
    qp=np.zeros((H,29),np.float32); qv=np.zeros((H,29),np.float32); la=np.zeros((H,29),np.float32)
    return np.concatenate([ang.reshape(-1),grav.reshape(-1),cmd.reshape(-1),
                           qp.reshape(-1),qv.reshape(-1),la.reshape(-1)]).astype(np.float32)
out={}
base=None
for wz in (0.0,0.2,0.5,1.0,-1.0):
    o=build(wz)
    with torch.no_grad(): a=m(torch.from_numpy(o).unsqueeze(0)).squeeze(0).numpy()
    if base is None: base=a.copy()
    d=np.abs(a-base)
    out[str(wz)]={"action_l2":float(np.linalg.norm(a)),"max_abs_delta_vs_wz0":float(d.max()),
                  "mean_abs_delta":float(d.mean())}
    print(f"wz={wz:+.2f}  |a|={np.linalg.norm(a):.4f}  max|a-a(wz=0)|={d.max():.6f}  mean={d.mean():.6f}",flush=True)
# sanity: does vx move the action?
o=build(0.0,vx=0.5)
with torch.no_grad(): a=m(torch.from_numpy(o).unsqueeze(0)).squeeze(0).numpy()
d=np.abs(a-base)
print(f"vx=+0.50 (control) max|a-a(0)|={d.max():.6f}  mean={d.mean():.6f}")
print("obs dim used:", D*H)
json.dump(out,open("/tmp/policy_probe.json","w"),indent=2)
