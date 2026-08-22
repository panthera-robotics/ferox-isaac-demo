# PING — two agent instances are running this brief on one box, in one working tree

**Raised 2026-08-22 ~04:45 UTC. Class: coordination / destructive-risk. Not a task failure.**

## The fact

A second instance of this campaign's agent is executing the same brief on this machine,
against **the same working tree** (`/root/panthera/ferox-isaac-demo`), the same branch
(`mohammed/mm-campaign`), the same Docker stack, and the same GPU. It is not a stale
process from an earlier session — it is live and committing.

Evidence, all of it checkable:

| observation | detail |
|---|---|
| commits I did not author | `bda1992` 04:38:30 and `b8fdfa5` 04:40:40, both stamped with **this session's own git identity** (`Mohammed (MM campaign) <mohameddbardana@gmail.com>`, which exists only in the `GIT_CONFIG_*` I set in-process) |
| a process running my script with arguments I never issued | `bash scripts/c39_ab_asset.sh twin_bare 1500` (PID 262832), and earlier `mm4_sonic_drive.py --hold-s 1200`. I used `twin`/`ref` and `--hold-s 900` |
| files I never wrote | `scripts/c39_ab_harvest.sh`, `docs/mm/evidence/C39/SONIC_ABORT.md` |
| **my own edits committed by someone else** | the robust sim-clock wait loop I wrote into `c39_ab_asset.sh` at 04:43 appears inside commit `bda1992`'s version of that file |
| my run destroyed mid-experiment | my SONIC container was stopped (`Stop`, `Program exiting normally`) and all four containers recreated under me, twice |

Same tree + same branch is the sharp edge: a `git add -A` from either instance sweeps
the other's half-finished work into a commit, and either instance editing a script the
other is *currently executing* corrupts the running shell (bash reads scripts
incrementally — this already produced one spurious `line 81: is: command not found`).

## Why this stops me rather than being logged and passed

The brief's stop list includes destructive/safety. Two agents mutating one container
stack and one branch is that: every A/B run either of us starts tears down the other's
containers mid-measurement, so **neither instance can produce a trustworthy C-39
result** while both are live, and the shared index makes commit corruption a matter of
timing rather than of care.

## What I did, and did not do

* **Did not** kill the other instance's containers, processes, or commits.
* **Yielded** `scripts/c39_ab_asset.sh` to its committed version — and that version is
  **more correct than mine**: it removes the `G1_LL_RIG_YAW=0` I had added. Its
  reasoning is right and my change was the bug: `sim_side` captures the real spawn pose
  and the override then *replaces* the quaternion, so the rig pins the base to a yaw the
  robot was never spawned at and twists it against its own planted feet. That is what
  drove the wrist past 35 rad/s and tripped SONIC's guard in my run.
* **Restored** the tracked evidence files under `evidence/C39/ab/` that my cleanup had
  staged for deletion.
* **Stopped** contending for the GPU, the containers and the branch.

## What the other instance has already established (its evidence, read and checked)

`docs/mm/evidence/C39/SONIC_ABORT.md`: with `HAND=dex5_1p`, SONIC does not fail to
balance the twin — **it aborts**, before the rig releases, on its own one-sided velocity
guard `body_dq[i] > 35` at mujoco index 24 = SDK 26 = `right_wrist_roll_joint`, the
joint this campaign already recorded as saturating (5 Nm limit carrying a 1 kg Dex5
hand). The command channel then freezes all-zero, the bridge correctly falls to
`mode=hold`, and the auto-release never fires: `rig released at: NEVER`,
`RESULT: INVALID`. **The robot never became free-standing, so nothing about balance was
tested.** I reproduced the same abort independently before finding their write-up.

This retro-invalidates any earlier row that read "SONIC will not balance the twin" as a
balance result, and it is a bigger correction than the one task 0 recorded.

## What I need from Mohammed (the decision is yours, not mine)

1. **Which instance should own this box?** Kill the other, or kill me. Running both
   wastes the 4090 and produces evidence neither of us can defend.
2. If both must run, they need **separate working trees, separate container name
   prefixes (`SIM_CONTAINER`), and separate branches** — none of which I will create
   unilaterally while another process is mid-run in this tree.

Until that is answered I am not starting any further container work. The GPU gate
itself is green: **RTX 4090, 24564 MiB, driver 580.105.08.**
