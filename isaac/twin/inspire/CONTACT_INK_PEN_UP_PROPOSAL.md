# Proposed pen-up ink specification v2 — OWNER_REVIEW_REQUIRED (17 Sep 2026)

**Status:** proposal only. The authoritative gate is unchanged (`contact_ink.MarkingRule`, `allowed_pen_up_leakage_m = 0`, `allowed_unintended_bridge_m = 0`). Every report produced by `contact_ink_proposal.evaluate_v2` carries the old score unchanged plus a `proposal` block; no probe or gate imports the proposal.

**Why a proposal is needed (measured, not argued):** with a spring nib compressed 5 mm at nominal pressure, the planner's lift after the pen-up flag starts at rest and ramps at 0.5 m/s², so the nib stays in contact for 0.1–0.3 s after the flag while it unloads (contact-09/11/12: 0.34 s). The old rule adds one ink width (0.5 mm) for the first pen-up mark and every metre of travel after it, so a stationary unloading dot already fails a zero allowance. The rule therefore cannot be met by any compliant marker regardless of tracking, which makes it unable to discriminate a good lift from a bad one.

**Proposed rule (units: m, s):** pen-up ink is allowed only if all three hold: (a) within `unloading_window_s` (0.40 s) after the pen-up flag time (first physics step after the last pen-down sample); (b) inside a capsule of `settle_radius_m` (1.0 mm) around the intended endpoint of the stroke that just ended; (c) the nib is unloading, i.e. `0 < spring_compression ≤ maximum_unloading_compression_m` (6.0 mm; a re-press is not unloading). Pre-pen-down ink is allowed only within `settle_radius_m` of the next stroke's intended start and within `unloading_window_s` before its pen-down flag. All other pen-up ink is leakage (same stroke) or bridge (between strokes) with the OLD zero allowance. The proposal never rescues a coverage/path/force/order failure.

**What it does NOT change:** p95 ≤ 3 mm, max ≤ 10 mm, ≥ 95 % coverage, per-segment rules, force window, bottoming, attachment/support disqualification, scoring frame, intended-stroke geometry.

**Adversarial replay tests:** `tools/tests/test_contact_ink_proposal.py` — stationary unloading at the endpoint (old FAIL, proposal PASS), drifting unloading (both FAIL), late ink after the window (FAIL), re-press without compression decay (FAIL), bridge dragged toward the next stroke (FAIL), geometric failure never rescued, parameter bounds.

**Decision requested from the owner:** adopt v2 as the gate (with these parameters), adjust parameters, or keep the zero rule and accept that spring-nib contact writing can only ever be reported as executed-but-not-qualified.
