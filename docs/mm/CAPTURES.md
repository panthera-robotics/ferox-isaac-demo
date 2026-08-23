
## MM8 montage — 2026-08-22

| file | sha256 | duration |
|---|---|---|
| `docs/mm/media/ferox_g1_motion_manip_20260822.mp4` | `9fe3af220e27b6faa1f5022f13ff0d3c75575120e86b2510842cc4bc21c8e4de` | 52 s |

Contents, each captioned on screen for what it is:
1. title card
2. **omni policy walk** — 6.159 m measured vs 6.0 m commanded, chase + fixed front
3. **film pipeline proof** — scripted joint sweep, captioned "NOT a capability demo"
4. scorecard — C-39 parked, C-23 fixed, grasp 0/20 lifts with 4/20 closures

**Not in it, and why:** no SONIC stand/walk (parked, C-39), no pick-place (0/20 lifts),
no PiP camera track. C-23 is fixed and the camera now runs, but the PiP track was not
shot this session — the montage would rather be short than imply a capability.

Ghost gate PASS 0.00095/0.01 on the source clips. **Every clip also had a rendered still
checked by eye** — the gate passed a legless torso earlier in this session, so it is
necessary and not sufficient.

## Release `mm-persist-12` — media assets (2026-08-23)

**Every file below is a RELEASE ASSET, not just a tree file.** This box is disposable;
a working-tree video dies with it. Download from the release, verify the sha256.

| file | MB | sha256 | honest caption |
|---|---|---|---|
| `ferox_g1_mm_reel_20260823.mp4` | 17.3 | `e34f01454d9819fb1b19de05d052a19ddbdf75f83a13bc74b93416c15a497b17` | THE REEL (74 s). Real omni walk + live D435i camera + a clearly-labelled scripted placement; nothing here is a real grasp. |
| `ferox_g1_motion_manip_20260822.mp4` | 10.7 | `9fe3af220e27b6faa1f5022f13ff0d3c75575120e86b2510842cc4bc21c8e4de` | Earlier 52 s cut from 2026-08-22, superseded by the reel above. Kept for provenance. |
| `mm_choreo_pick_20260823.mp4` | 1.4 | `a1fb5193f174c0d1d4a9f089d45e186badf2319d9f3fd48774f69bb2eccea3f7` | CHOREOGRAPHY, NOT A GRASP. Scripted joint trajectory with a cheat-attach (CAMPAIGN 0.6), banner burned into every frame. The real grasp is descent/IK-limited and in validation. |
| `mm_filmtool_chase_20260822.mp4` | 1.8 | `303291677d9eebddb9d6ebc4d61eb79157f3d655a758d5a72a3d00b3f6985ac2` | PIPELINE PROOF ONLY. film.py's scripted joint sweep -- proves convergence/framing/ghost gate. The robot is not walking. |
| `mm_filmtool_front_20260822.mp4` | 1.9 | `91ba0754ebe4d40565c5330f51182acc9b77196447c3eeff6cb4641b15f464a2` | PIPELINE PROOF ONLY. Same scripted sweep, fixed camera. |
| `mm_omni_walk_chase_20260822.mp4` | 7.3 | `1caf7a8895159f663d7b96a049ae49ccbcec2e2b12fb38c0ad1655200b086805` | REAL. Omni locomotion policy walking 6.159 m against 6.0 m commanded, measured from base pose. Chase camera. |
| `mm_omni_walk_front_20260822.mp4` | 7.3 | `6fce8feb56d591664cac4b0194767d6c7d313e42f4c5c1e8ed6f806669073797` | REAL. The same 6.159 m walk from a fixed front camera -- the view that shows travel rather than following it. |
| `mm_walk_with_pip_20260823.mp4` | 5.8 | `ad0de00dcc95765e620eb25f52141a3c1081fa99b30bd425fed4777db21a9300` | REAL. Live D435i colour + colourised depth (captured in the hospital world) composited over the walk clip -- two separate captures, not one simultaneous shot. |

**Camera capture:** `docs/mm/media/camera_capture/` holds 6 representative RGB + 6 depth
frames and `MANIFEST_sha256.txt` covering all 296 captured frames.


## Release `mm-persist-13` — media assets (2026-08-23, ALL REAL)

**No cheat-attach and no choreography in this release.** Task 1 closed as
`reachable-but-pinches` (0 finger links around the object), so the reel is walk + camera
only and says so on a closing card.

| file | MB | sha256 | honest caption |
|---|---|---|---|
| `ferox_g1_real_reel_20260823.mp4` | 25.6 | `49f53eaa9301ada59427812b979cc80249e7677243a77481081417b2b417cdb6` | THE REEL (60 s), ALL REAL. Omni walk in the lit hospital (hero arc + chase) and live D435i colour+depth. No cheat-attach, no choreography; a closing card states manipulation is excluded and why. |
| `mm_lit_chase_20260823.mp4` | 13.2 | `2175b442b8330649873c8f72af56aa7ec32eecb786770737b082612c3f298330` | REAL. The same walk, chase camera with lag, lit hospital -- textured floor and shadows, so travel reads as travel. |
| `mm_lit_hero_20260823.mp4` | 8.6 | `9da41bd7e1b4b7db71cfa0b73c2aebf817edc298f7649c70ff8b6e838e1e533d` | REAL. Omni policy walking 6.159 m vs 6.0 commanded, hero camera on an eased arc-and-dolly in the lit hospital. |
| `mm_lit_pip_20260823.mp4` | 11.0 | `1669abe700af9d4db826bf3bfc94b7f8d2f45c074eeac40e7300248e20fe2ec1` | REAL. Lit-hospital chase with live D435i colour + colourised depth insets (insets captured separately and composited). |
