# C-23 — the five boots, and what each one ruled out

Every G1 twin boot attempted on this box with a live ROS 2 image writer, in order.
All five segfaulted at the same place: `run.py:1201`, the first
`world.step(render=True)` after the camera render product exists, with a native
backtrace through `libomni.syntheticdata.plugin.so` and
`libomni.graph.image.core.plugin.so`, immediately after Isaac's own warning

```
OgnSdPostRenderVarToHost : rendervar copy from texture directly to host buffer
is counter-performant. Please use copy from texture to device buffer first.
```

| # | configuration | world | outcome | what it ruled out |
|---|---|---|---|---|
| 1 | stock, committed asset | `dso_block_a` | segfault | — (the baseline) |
| 2 | stock, repeat | `dso_block_a` | segfault | a transient |
| 3 | stock + VRAM/RSS sampling at 1 Hz | `dso_block_a` | segfault | **memory**: peak VRAM **3776 MiB of 16376**, peak host python RSS **5695 MiB of 47 GiB**, no OOM, no Xid, `/dev/shm` 24 G |
| 4 | `annotator_device="cuda"` | `hospital` | **no crash, and worse** | that this is a fix — both render-product topics went silent, the camera image *and* `/livox/lidar`, while `/livox/imu` (94 Hz) and `/odom` (46 Hz) kept publishing so the sim looked alive |
| 5 | `want_depth_frame=False` (python-side depth annotator removed) | `hospital` | segfault | **the depth annotator** — the default `rgb` annotator does it too |

## The control

`ROBOT=go2 TWIN=1 SIM_WORLD=hospital` — same box, same Isaac image, same RTX lidar
render product, same ROS 2 bridge, same worlds, **no camera in its contract** —
reached the main loop on **every** attempt (40 s, 190 s, 35 s, 35 s, 40 s).

Camera present ⇒ crash. Camera absent ⇒ no crash.

## The other control: offscreen rendering is fine

`scripts/14_capture_views.sh`, `scripts/13_capture_hands.sh` and two
`tools/media/render_orbit.py` passes all create their own `Camera` in a headless
Isaac and all completed, writing real frames (3 views, 4 hand poses, 90 + 60 orbit
frames). So C-23 is **not** "any camera" — it is the **ROS 2 image writer** path
specifically. The orbit renders do segfault on shutdown, after every frame is
written; that is separate and harmless.

## The box

| | this box | the box the campaign was validated on (RESUME §1) |
|---|---|---|
| GPU | **RTX 4080 SUPER, 16376 MiB** | RTX 4090, 49140 MiB |
| RAM | 47 GiB | 98 GB |
| driver | 580.105.08 | 580.105.08 |
| Isaac | 5.1.0 | 5.1.0 |

Driver and Isaac are the documented ones. The GPU is not.

## Disposition

**Mohammed, 2026-08-19: it is the GPU. Do not work around it on this box.**
`TWIN_CAMERA=0` (default off — i.e. camera on) exists only so the lidar/nav half
stays workable here; it skips the camera *device* and touches nothing the audit
checks. Item C, E-1, C-21's live re-proof and the montage's camera clip all wait
for a 4090 box.
