# C-39 diagnostic fork — both branches negative

SONIC alone from t=0 (`G1_CONTROL=lowcmd`, no hand-off), rig holding the twin at
**SONIC's own nominal stance** (knee 0.30, arms 0.0, base at the contract standing height
0.731), settled, then released with SONIC already commanding.

## Branch 1 — stance transition? NO

Real hand mass (L 1.025045 / R 0.978570 kg). Released at t=24.52 s.

```
t=42.50  mode=cmd  base_z=+0.060  pitch=-1.549  |tau|max= 6.56  sat=0/29
t=60.00  mode=cmd  base_z=+0.060  pitch=-1.549  |tau|max= 6.55  sat=0/29
```

Flat, and SONIC is not straining: 6.5 Nm across 29 joints with nothing saturated. So
C-39 is **not** the omni policy's stance being wrong — handed a robot already in its own
nominal pose, SONIC still puts it on the floor.

## Branch 2 — hand mass? NO

Same run with the palms zeroed — `TWIN_HAND_KG=0.34`, palm 0.685702 → 0.001 kg, i.e.
**0.69 kg/hand lighter than reality**, which is the mass the W campaign proved SONIC's
balance against.

```
[G1] DIAGNOSTIC hand mass -> 0.340 kg/hand (palm 0.0010, was [0.7322, 0.6857])
t=45.00  mode=cmd  base_z=+0.060  pitch=-1.547  |tau|max= 6.53  sat=0/29
```

**Identical.** Base height, pitch and torque all match the real-mass run to three
figures. The payload bisect (0.34 → 0.6 → 0.8 → 1.0) is therefore moot: there is no
payload margin to report, because the failure does not depend on payload at all.

## What it actually is: SONIC's commanded targets are not a stand

Measured while the twin is **upright on the rig**, so this is not divergence after a
fall, and with the observation conventions already verified against both the reference
MuJoCo bridge and the DT bag (`CONVENTION_TABLE.md`):

```
q_d legs : [-0.859, 0.128, -0.051, 1.648, -0.320, 0.115]
q   legs : [-0.413, 0.315, -0.109, 0.284,  0.124, -0.262]
|q_d| max : 1.986 (idx 22)
q_d beyond URDF |limit| : 3 of 29
|q_d - q| mean 0.692, max 2.198
```

Its gains are sane — kp 99.1 on the legs, 14.3–99.1 overall, kd 0.9–6.3, `tau_ff` zero —
so this is not a disabled or limp controller. It is asking for a **knee of 1.648 rad
against its own nominal 0.30**, a hip of −0.859 against −0.1, and **three joints outside
the URDF's own limits**. That is not a standing posture, and no amount of stance
matching or mass reduction will make a robot stand from it.

## Conclusion

C-39 is upstream of both hypotheses. SONIC is not producing valid standing control for
this twin, its observations are convention-correct, its gains are correct, and its
targets are out of range. Whatever is wrong sits between its observations and its policy
output — most likely in the token/encoder state or the reference-motion set it was
started with (`reference/example/` holds 13 dance/kick/lunge/jump motions and it opens on
`tired_one_leg_jumping_R_001__A359`; there is no plain stand among them).

**Next probe for whoever picks this up**, and it is cheap: run this same fork against the
reference MuJoCo sim rather than the twin. If SONIC also commands out-of-range targets
there, the twin is exonerated entirely and the fault is in how this deploy is being
driven; if it stands there, the difference is in the twin and the diff is now a very
short list, because the wire is already proven identical.
