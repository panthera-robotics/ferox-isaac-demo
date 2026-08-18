# Captures — robot-side ground truth

Large binaries live on **GitHub Releases**, not in git. The repo stays cloneable in
about 117 MB; captures are fetched on demand.

**Release:** <https://github.com/panthera-robotics/ferox-isaac-demo/releases/tag/twin-gt-g1>

> **This repository is PUBLIC**, so anything listed here is publicly downloadable
> without authentication. That is what makes a fresh instance resumable from GitHub
> alone. It also means these captures are published robot telemetry — flagged here so
> the choice is a deliberate one rather than a side effect.

---

## g1_twin_gt.tgz — G1 #1 ground truth

| | |
|---|---|
| **sha256** | `946ae35238827f8ef77187d4f9719ce38e76f8d42c594b67878ce2ff96e1d17c` |
| **size** | 67 200 539 bytes (64 MiB) |
| **robot** | Unitree G1 #1, standing still |
| **recorded** | 2026-08-18 14:02:46 → 14:03:20 UTC (34.571 s) |
| **conditions** | driver + livox sidecar up, **no camera container** |
| **contents** | `captures/g1_twin_gt/` (rosbag2, sqlite3) + three layout side-files |
| **messages** | 52 210 across 10 topics |
| **closed** | OQ-2, OQ-3, and four provenance upgrades to `captured` |
| **opened** | C-18, C-19, C-20; and OQ-6 (the robot ran `lidar_tf_mode=static`) |
| **report** | [`RESULTS_GT_G1.md`](RESULTS_GT_G1.md) |

### Topics

| topic | type | count | rate |
|---|---|---|---|
| `/lowstate` | `unitree_hg/msg/LowState` | 35 998 | ~1041 Hz |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 6 911 | 200.0 Hz |
| `/ferox/g1_01/imu/data` | `sensor_msgs/msg/Imu` | 3 240 | 93.7 Hz |
| `/ferox/g1_01/odom` | `nav_msgs/msg/Odometry` | 1 778 | 51.46 Hz |
| `/state_estimator/odom_pelvis` | `nav_msgs/msg/Odometry` | 1 777 | 51.4 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1 777 | 51.4 Hz |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 346 | 10.0 Hz |
| `/ferox/g1_01/scan` | `sensor_msgs/msg/LaserScan` | 346 | 10.0 Hz |
| `/ferox/g1_01/clock_offset` | `diagnostic_msgs/msg/DiagnosticStatus` | 34 | ~1 Hz |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | latched |

**`/lowstate` needs `unitree_hg`**, which the Ferox nav image does not have. 35 998 of
the bag's messages are therefore not decodable with the tooling in this repo. The
`GT_G1` analysis used the TF edge for the waist pose instead — see RESULTS_GT_G1 §4.

### Inner file hashes (after extraction)

| file | sha256 |
|---|---|
| `captures/g1_twin_gt/g1_twin_gt_0.db3` | `4d3f5165181c447b9a65c5b3c3d36c8cddd244523fba1fe698de7e52e0b017d4` |
| `captures/livox_lidar_layout.txt` | `90ac902a570bc56c0778b1f31e5103f4daf26e59ce382923c887dc95cf10da55` |
| `captures/livox_imu_layout.txt` | `36f3daae1b86906baed945078cce2d7e3d7838de259a6832cd2c324c25bf6b36` |
| `captures/livox_lidar_qos.txt` | `186f2bd5ad44cea0a5e22a288052f0b07972eb7b45925be0833e5dac11509a6d` |

### How to fetch

```bash
mkdir -p ~/panthera/ref/captures/g1
cd ~/panthera
curl -fL -o g1_twin_gt.tgz \
  https://github.com/panthera-robotics/ferox-isaac-demo/releases/download/twin-gt-g1/g1_twin_gt.tgz

# verify BEFORE extracting -- a truncated release download is a silent 0-byte topic
echo "946ae35238827f8ef77187d4f9719ce38e76f8d42c594b67878ce2ff96e1d17c  g1_twin_gt.tgz" \
  | sha256sum -c -

tar -xzf g1_twin_gt.tgz -C ref/captures/g1/
```

### How to audit against it

```bash
docker cp ~/panthera/ref/captures/g1/captures/g1_twin_gt ferox_nav:/tmp/gt
cd ~/panthera/ferox-isaac-demo
ROBOT=g1 ./scripts/07_twin_audit.sh --bag /tmp/gt
```

Expect **36 pass, 0 Class-B fail, 11 Class-A fail** — of which **nine are the camera
the capture did not have** (5 topics + 4 TF edges) and two are the single OQ-6 item
(`base_link → livox_frame` static on the robot, dynamic in the contract), counted from
both sides. Anything else is a regression.

---

## twin_progress_20260818.mp4 — progress montage

| | |
|---|---|
| **sha256** | `ce387ea7d11b3ed020d9a20bdec61c65ff99e020764c63028f16c185d546ff93` |
| **size** | 6 491 813 bytes (6.19 MiB) |
| **format** | 1920x1080, 30 fps, 93 s, H.264 (`avc1`) |
| **built** | 2026-08-18, offscreen — no GUI, no ffmpeg on the box |
| **contents** | 10 clips, DT0-DT5 + fastpath + gt-g1 + persist |
| **notes** | [`media/README.md`](media/README.md) — one line per clip, and the two substitutions |

Committed in the repo at `docs/twin/media/` as well as attached to this release, since
6 MB is small enough to keep in git and a montage that only exists on a release is one
`git clone` away from being lost.

```bash
curl -fL -o twin_progress_20260818.mp4 \
  https://github.com/panthera-robotics/ferox-isaac-demo/releases/download/twin-gt-g1/twin_progress_20260818.mp4
echo "ce387ea7d11b3ed020d9a20bdec61c65ff99e020764c63028f16c185d546ff93  twin_progress_20260818.mp4" | sha256sum -c -
```

---

## Pending — Go2 ground truth

Not captured yet. When it lands, extract to `~/panthera/ref/captures/go2/`, upload to
this same release (or a `twin-gt-go2` tag), add a row here with its sha256, and run:

```bash
ROBOT=go2 ./scripts/07_twin_audit.sh --bag /tmp/gt_go2
```

The row that decides **C-17** is `laserscan/self_hit`. The sim reports
`14 rays 0.300-0.313 m in 1 run(s)` at `-6.5..-0.1 deg`. If the robot reports a
comparable run, C-17 is faithful and the fix belongs in the driver; if it reports
`none under 0.35 m`, C-17 is a sim artefact. The Go2's `pointcloud/fields` is still
`assumed` on the retired Unitree layout, and this capture is what closes it — as the
G1's did for OQ-2.

---

## What is NOT here, and why

* **The robot USD assets** (`isaac/assets/**`, ~103 MB) are committed to git, not to a
  release. They are what the twin loads, and a clone that cannot boot the sim is not
  resumable. They are also regenerable — see RESUME.md step 5 — so if they ever need to
  leave git, nothing is lost but time.
* **`cache/`** (4.9 GB of Isaac shader and texture cache) is gitignored and must stay
  that way. It rebuilds itself on first run, slowly.
* **The reference driver clones** under `~/panthera/ref/` are other repositories; RESUME.md
  lists the exact clone commands and the commits this campaign read.
