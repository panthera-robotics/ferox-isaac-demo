# Progress montage — 2026-08-18

**`twin_progress_20260818.mp4`** — 1920x1080, 30 fps, **93 s**, 6.49 MB.
**`twin_progress_20260818.png`** — 2x4 contact sheet, one still per clip.

Every frame is a real offscreen render or real recorded data. Nothing is mocked. Two
clips could not be produced as asked inside the time-box; both say so **on their own
title card**, and the substitution is named there as well as here.

Built with no GUI and no ffmpeg: this box has no logged-in desktop X session, so all
renders are offscreen (`tools/media/render_orbit.py`), and the video is stitched with
OpenCV's `avc1` writer (`tools/media/stitch.py`) because there is no ffmpeg binary
anywhere on the machine.

| # | Clip | Shows | Gate / tag |
|---|---|---|---|
| 1 | G1 twin body | Offscreen orbit front-side-top: Unitree meshes, Dex5-1P hands attached, head shell, no floating sensors | DT2 + DT3 / `twin-DT2`, `twin-DT3` |
| 2 | Walks with hands on | The DT3 `validate_motion` table revealed row by row over an orbit still - measured body velocities and zmin per motion | DT3 / `twin-DT3` |
| 3 | Sensors where the real ones are | Same orbit with RGB axis triads drawn as **real geometry** at the authored prim paths: `livox_frame`, `camera_link`, `dog_imu_link` | DT2 / `twin-DT2` |
| 4 | Mid-360 twin | Live `/livox/lidar` sweeps transformed into `base_link` by the twin's own TF; floor plane 0.0039 deg vs world vertical | DT2 / `twin-DT2` |
| 5 | Camera twin | `rgb8` 1280x720 beside `16UC1` mm aligned depth, chair + mustard bottle in view; then C-21 before/after | DT2 / C-21 closed |
| 6 | `/scan` - twin vs robot | The 723-ray polar plot, twin on the left, the **real G1 from the ground-truth bag** on the right | DT1 + `twin-gt-g1` |
| 7 | Nav2 + SLAM | The live SLAM Toolbox map revealed as it builds, from the twin's `/scan` | DT2 / `twin-DT2` |
| 8 | Dex5-1P hand poses | rest / open / fist / thumb opposition, each reached to <= 0.001 rad, commanded by joint **name** | DT3 / `twin-DT3` |
| 9 | Go2 twin | Orbit with the Mid-360 on its top mount and axis triads, the 20 Hz cloud, and the `/scan` polar with the **C-17 self-hit run in red** | DT5 / `twin-DT5` |
| 10 | Closing card | Audit status both robots, gates + tags, open items | - |

## The two substitutions

* **Clip 2 is not a rendered walk.** Adding an offscreen camera to the *running* sim
  was out of time-box, and the twin was standing still during the media capture - so a
  z-vs-time plot would have been a flat line dressed up as motion. It shows the actual
  DT3 `validate_motion` numbers instead (`evidence/DT3/validate_motion_dex5.txt`):
  forward +0.506 against a commanded +0.50, rotate-in-place still failing, max
  base-height delta 0.02 m against the 0.03 allowance.
* **Clip 7 has no Nav2 path.** The map is real and live, but no plan is drawn because
  the planner published none: G1 goals reach the tolerance boundary and time out, which
  DT2 recorded as PARTIAL and deliberately did not tune.

Clip 9's self-hit ray count is measured live at render time, not typed in - it read
**12 rays** in 0.29-0.32 m for this capture, against the 13-14 the DT5 audit recorded.
The cluster is body-fixed; the exact count varies by a ray or two between sweeps.

## Rebuild

    # 1. offscreen renders (needs the sim container; copy frames OUT immediately --
    #    01_start_sim.sh recreates the container and wipes /tmp)
    docker exec -e ROBOT=g1 -e FRAMES=72 -e AXES=0 -e OUT=/tmp/orbit_g1 \
      ferox_isaac_sim /isaac-sim/python.sh /tmp/isaacrun/render_orbit.py

    # 2. real data -> .npz (nav container; live twin, or MODE=bag against a capture)
    MODE=live python3 tools/media/capture_data.py
    MODE=bag BAG=/tmp/gt python3 tools/media/capture_data.py

    # 3. assemble + stitch
    python3 tools/media/build_montage.py                          # host, PIL only
    python3 tools/media/stitch.py /tmp/montage/plan.json out.mp4   # nav container, cv2

sha256(`twin_progress_20260818.mp4`) = `ce387ea7d11b3ed020d9a20bdec61c65ff99e020764c63028f16c185d546ff93`

Also attached to the **`twin-gt-g1`** release - see [`../CAPTURES.md`](../CAPTURES.md).
