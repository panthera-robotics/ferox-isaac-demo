# C-23 — **RESOLVED. It was `headless: False`.**

2026-08-22. Not the GPU, not the ROS 2 image writer, not memory. `run.py` created its
`SimulationApp` with `headless: False` unconditionally, which asks Kit for a **windowed**
renderer; on a box with no logged-in X session GLFW init fails and the camera render path
segfaults on the second `world.step(render=True)`.

## The bisect that found it — one variable per probe

| probe | configuration | result |
|---|---|---|
| A | camera + render product, no ROS 2 bridge | SURVIVED |
| B | + `isaacsim.ros2.bridge` | SURVIVED |
| C | + G1-Dex5 asset, camera under the articulation | SURVIVED |
| D | + hospital world | SURVIVED |
| E | + the twin's own `twin_sensors.create_camera` (contract K, `fx=908.0`) | SURVIVED |
| F | + two rclpy executor threads spinning during render | SURVIVED |
| G | + TorchScript policy on CUDA via `G1VelocityPolicy` | SURVIVED |
| **H** | **G with `headless: False` — the only change** | **SEGFAULT on render step #2** |

Probe G is, structurally, the entire twin. It survives. Flip one boolean and it dies in
`libomni.syntheticdata` + `libomni.graph.image` — C-23's signature on the 4080 and the
4090 alike.

## The fix

`TWIN_HEADLESS=1` (`run.py`). Default remains `False`, so an operator with a real desktop
still gets a viewport; the flag is for boxes without a logged-in X session, which is every
cloud box this campaign has ever run on.

## What was wrong in the record, and what it cost

* *"It is the GPU. Do not work around it."* — wrong. Reproduced on an RTX 4090 with the
  exact driver `RESUME.md` specified.
* *"It is the ROS 2 image writer specifically."* — wrong. A capture route with no
  OmniGraph writer in it crashes identically, and the writer-ful stack survives headless.
* *"Camera path verified only on RTX 4090."* — unverifiable; nobody had evidence of the
  camera working anywhere.

Cost: five boots on the 4080, a second GPU, and every camera item deferred since
2026-08-19 — MM0.2's aligned-depth check, the C-21 clip, E-1/`ferox_vision`, MM6, MM7 and
the montage's PiP track. **None of it needed hardware.**

The lesson worth keeping is the shape of the error, not the boolean: the original
diagnosis reasoned from *what the crash touched* (`libomni.syntheticdata`, so "the image
writer") instead of bisecting *what the process differed in*. Eight probes at two minutes
each settled what two GPUs could not.

## Aligned-depth check — MM0.2, now green

`evidence/C23/aligned_depth_20260822.json`, via the annotator route, headless:

    K            fx 908.0  fy 908.0  cx 640  cy 360   HFOV 70.36 deg  VFOV 43.25 deg
    rgb          (720, 1280, 3)          depth  (720, 1280)      aligned: true
    depth_mm     min 306   median 1668   max 2574
    zero_fraction 0.0      finite_fraction 1.0

Colour and depth share a shape and a render product, so a pixel in one indexes the same
ray in the other. Zero fraction is 0.0 — every pixel carries a return, as it should
looking into a room at 0.3–2.6 m.
