#!/usr/bin/env python3
"""twin_audit — does the sim publish what the robot publishes?

The twin campaign exists because sim/hardware drift is invisible until it costs a
training run. This tool makes it loud. It compares an observed ROS 2 interface (the live
sim graph, a rosbag, or the robot's own recorded evidence) against a twin contract, and
exits non-zero if any Class-A item differs.

Parity classes (campaign section 2):
  A  exact     topic names, msg types, encodings, frame_ids, TF edge set + values, QoS
               reliability, PointCloud2 field layout, LaserScan ray geometry
  B  modeled   rates, camera intrinsics, resolutions -- within a stated tolerance
  C  declared  known approximations, listed in docs/twin/TWIN_DEVIATIONS.md

Only Class A sets the exit code. Class B failures are reported and counted; Class C items
are informational. A clean run means the strings and structure are identical -- not that
the physics matches.

Usage
  # live sim graph (default). Must run where the ROS graph is visible:
  #   docker exec ferox_nav ... python3 tools/twin_audit.py --contract <yaml>
  twin_audit.py --contract isaac/twin/g1_contract.yaml [--duration 12]

  # a recorded bag
  twin_audit.py --contract isaac/twin/g1_contract.yaml --bag /path/to/bag

  # no bag available: check the contract against the driver's own evidence capture
  twin_audit.py --contract isaac/twin/g1_contract.yaml \
                --against-evidence ~/panthera/ref/panthera-g1-driver/evidence
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, asdict
from typing import Any, Dict, List

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import twin_contract  # noqa: E402
import twin_sources  # noqa: E402
from twin_sources import Observation, TopicObs  # noqa: E402

# Tolerances -- campaign section 6. Named, not inlined, so a change is reviewable.
TOL_TRANSLATION_M = 1e-4
TOL_ROTATION_RAD = 1e-4
TOL_RATE_FRACTION = 0.10          # lidar / odom / imu: +-10 %
# Camera floor. The campaign's 20 Hz was relaxed to 15 at DT2: the converter is
# BANDWIDTH-bound, not compute-bound -- 1280x720 colour plus 1280x720 float depth is
# ~190 MB/s of inbound DDS into one Python process, and optimising its numpy hot path
# changed nothing measurable. Carried as TWIN_DEVIATIONS C-10 with the fix that would
# close it (publish depth at the module's native 848x480 and upsample).
CAMERA_MIN_RATE_HZ = 15.0         # 30 Hz target; >=15 accepted, see C-10
TOL_K_FRACTION = 0.01             # camera_info K within 1 %
MIN_FRAC_IN_RANGE = 0.99          # >=99 % of returns inside [range_min, range_max]

OK, FAIL, SKIP = "PASS", "FAIL", "SKIP"


@dataclass
class Finding:
    parity: str
    check: str
    subject: str
    expected: str
    actual: str
    status: str
    note: str = ""

    @property
    def failed(self) -> bool:
        return self.status == FAIL


def _f(v: Any) -> str:
    """Render a value for the findings table.

    Floats get 9 significant digits, not 6. At 6 the LaserScan angle bounds of the sim
    and the contract both render as "-3.14159" while differing in the 8th digit -- a
    PASS row that looks like a FAIL row, or worse, two visibly identical numbers with
    one marked FAIL. An audit whose output cannot be trusted at a glance is not an audit.
    """
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.9g}"
    return str(v)


# ------------------------------------------------------------------ checks


def check_topics(contract: Dict[str, Any], obs: Observation) -> List[Finding]:
    out: List[Finding] = []
    for spec in contract["topics"]:
        name = spec["name"]
        cls = spec["parity_class"]
        t: TopicObs = obs.topics.get(name, TopicObs())

        if spec["direction"] == "subscribe":
            # A topic the robot SUBSCRIBES to is an input; the sim must also accept it,
            # but its presence in the graph depends on whoever publishes it. Report,
            # never fail on it.
            out.append(Finding("C", "topic/subscribe", name, "accepted by sim",
                               "present" if t.present else "no publisher seen", SKIP,
                               "input topic -- presence depends on the commanding side"))
            continue

        out.append(Finding(cls, "topic/present", name, "published",
                           "published" if t.present else "ABSENT",
                           OK if t.present else FAIL))
        if not t.present:
            continue

        if not t.type:
            out.append(Finding(cls, "topic/type", name, spec["type"],
                               "not recorded by this source", SKIP))
        else:
            out.append(Finding(cls, "topic/type", name, spec["type"], t.type,
                               OK if t.type == spec["type"] else FAIL))

        want_frame = spec.get("frame_id")
        if want_frame:
            if not obs.payloads_observable:
                out.append(Finding(cls, "topic/frame_id", name, want_frame,
                                   "not observable by this source", SKIP))
            elif not t.frame_id:
                out.append(Finding(cls, "topic/frame_id", name, want_frame, "no message received", FAIL))
            else:
                out.append(Finding(cls, "topic/frame_id", name, want_frame, t.frame_id,
                                   OK if t.frame_id == want_frame else FAIL))

        want_enc = (spec.get("expect") or {}).get("encoding")
        if want_enc:
            if not obs.payloads_observable:
                out.append(Finding(cls, "topic/encoding", name, want_enc,
                                   "not observable by this source", SKIP))
            else:
                out.append(Finding(cls, "topic/encoding", name, want_enc, t.encoding or "-",
                                   OK if t.encoding == want_enc else FAIL))

        want_qos = spec.get("qos") or {}
        if want_qos.get("reliability"):
            if not t.qos_reliability:
                out.append(Finding(cls, "topic/qos", name, want_qos["reliability"], "not observable", SKIP))
            else:
                out.append(Finding(cls, "topic/qos", name, want_qos["reliability"], t.qos_reliability,
                                   OK if t.qos_reliability == want_qos["reliability"] else FAIL))

        want_rate = spec.get("rate_hz")
        if want_rate:
            if t.rate_hz is None and not obs.payloads_observable:
                out.append(Finding("B", "topic/rate", name, f"{want_rate} Hz",
                                   "not recorded by this source", SKIP))
            elif t.rate_hz is None:
                out.append(Finding("B", "topic/rate", name, f"{want_rate} Hz", "no messages", FAIL))
            elif spec.get("rate_rule") == "camera":
                out.append(Finding("B", "topic/rate", name, f">={CAMERA_MIN_RATE_HZ} Hz (target {want_rate})",
                                   f"{t.rate_hz:.2f} Hz ({t.rate_basis})",
                                   OK if t.rate_hz >= CAMERA_MIN_RATE_HZ else FAIL))
            else:
                lo, hi = want_rate * (1 - TOL_RATE_FRACTION), want_rate * (1 + TOL_RATE_FRACTION)
                out.append(Finding("B", "topic/rate", name, f"{want_rate} Hz +-10%",
                                   f"{t.rate_hz:.2f} Hz ({t.rate_basis})",
                                   OK if lo <= t.rate_hz <= hi else FAIL))
    return out


def check_tf_static(contract: Dict[str, Any], obs: Observation) -> List[Finding]:
    out: List[Finding] = []
    # A dynamic edge lives on /tf, not /tf_static -- checked separately below.
    want = {(e["parent"], e["child"]): e for e in contract.get("tf_static", [])
            if not e.get("dynamic")}
    if not want:
        return out
    have = obs.tf_static

    for key in sorted(want):
        label = f"{key[0]} -> {key[1]}"
        if key not in have:
            if want[key].get("default_published") is False:
                out.append(Finding("A", "tf_static/edge", label, "published only when enabled",
                                   "absent (gate off)", SKIP,
                                   want[key].get("conditional", "")))
            else:
                out.append(Finding("A", "tf_static/edge", label, "present", "MISSING", FAIL))
            continue
        out.append(Finding("A", "tf_static/edge", label, "present", "present", OK))

        if not obs.tf_static_values_known or have[key].get("xyz") is None:
            out.append(Finding("A", "tf_static/value", label, "match contract",
                               "values not recorded by this source", SKIP))
            continue

        exp = want[key]
        dx = [a - b for a, b in zip(have[key]["xyz"], exp["xyz"])]
        dist = math.sqrt(sum(v * v for v in dx))
        out.append(Finding("A", "tf_static/xyz", label,
                           f"{[round(v,6) for v in exp['xyz']]} (<={TOL_TRANSLATION_M} m)",
                           f"{[round(v,6) for v in have[key]['xyz']]} (d={dist:.2e} m)",
                           OK if dist <= TOL_TRANSLATION_M else FAIL))

        want_q = twin_contract.quat_from_rpy(*exp["rpy"])
        ang = twin_contract.angular_distance(tuple(have[key]["quat"]), want_q)
        out.append(Finding("A", "tf_static/rpy", label,
                           f"{[round(v,6) for v in exp['rpy']]} rad (<={TOL_ROTATION_RAD} rad)",
                           f"d={ang:.2e} rad",
                           OK if ang <= TOL_ROTATION_RAD else FAIL))

    for key in sorted(have):
        if key not in want:
            out.append(Finding("A", "tf_static/extra", f"{key[0]} -> {key[1]}",
                               "not in contract", "PUBLISHED ANYWAY", FAIL,
                               "an edge the robot does not publish"))
    return out


def _deviation_for(spec: Dict[str, Any], check: str) -> str:
    """Return the declared deviation id for a check, or "" if it is not declared.

    A contract topic may list `deviations: {check: "C-n reason"}`. Those checks are
    reported at Class C instead of A, so they no longer gate the exit code -- but
    they are still RUN and still printed with the measured difference. The point is
    that an accepted approximation stays visible every single run; deleting the
    check would make the sim look conformant, which is the failure mode this whole
    campaign exists to prevent.
    """
    return (spec.get("deviations") or {}).get(check, "")


def check_payloads(contract: Dict[str, Any], obs: Observation) -> List[Finding]:
    """LaserScan geometry, PointCloud2 field layout, CameraInfo intrinsics."""
    out: List[Finding] = []
    for spec in contract["topics"]:
        name = spec["name"]
        expect = spec.get("expect") or {}
        t = obs.topics.get(name)
        if not expect or t is None or not t.present:
            continue
        if not t.extras:
            if not obs.payloads_observable:
                out.append(Finding("A", "payload", name, "compared against contract",
                                   "not observable by this source", SKIP))
            continue
        ex = t.extras

        ls = expect.get("laserscan")
        if ls:
            for key, cls in (("ray_count", "A"), ("angle_min", "A"), ("angle_max", "A"),
                             ("angle_increment", "A"), ("scan_time", "B"),
                             ("range_min", "A"), ("range_max", "A")):
                if key not in ls:
                    continue
                got, wanted = ex.get(key), ls[key]
                if isinstance(wanted, float):
                    good = got is not None and abs(got - wanted) <= max(1e-9, abs(wanted) * 1e-6)
                else:
                    good = got == wanted
                out.append(Finding(cls, "laserscan/" + key, name, _f(wanted), _f(got),
                                   OK if good else FAIL))
            frac = ex.get("frac_in_range")
            if frac is not None:
                out.append(Finding("B", "laserscan/in_range", name,
                                   f">={MIN_FRAC_IN_RANGE:.0%} of finite returns",
                                   f"{frac:.1%}", OK if frac >= MIN_FRAC_IN_RANGE else FAIL))
            below = ex.get("frac_below_range_min")
            if below is not None:
                out.append(Finding("C", "laserscan/below_range_min", name,
                                   "reported (self-hit fraction)", f"{below:.2%}", SKIP))

            # SELF-HIT REPORT (C-17). Always emitted, never a pass/fail: this exists
            # so the sim's number and a hardware capture can be read side by side and
            # compared one-to-one. On the Go2 the sim currently reports 13-14 rays at
            # 0.300-0.313 m over a single 6.4-degree run off the robot's own nose; if
            # a robot capture reports the same, C-17 is faithful and the fix belongs
            # in the driver. If it reports nothing, C-17 is a sim artefact.
            sh = ex.get("self_hit")
            if sh is not None:
                if sh["count"] == 0:
                    detail = f"none under {sh['threshold_m']:.2f} m"
                else:
                    detail = (f"{sh['count']} rays "
                              f"{sh['range_min_m']:.3f}-{sh['range_max_m']:.3f} m "
                              f"in {len(sh['runs'])} run(s)")
                out.append(Finding("C", "laserscan/self_hit", name,
                                   f"reported in {sh['frame'] or 'target frame'} "
                                   f"(<{sh['threshold_m']:.2f} m)",
                                   detail, SKIP))
                for run in sh["runs"]:
                    out.append(Finding(
                        "C", "laserscan/self_hit_run", name,
                        "azimuth span of one contiguous run",
                        f"{run['rays']} rays  "
                        f"{run['azimuth_lo_deg']:+.1f}..{run['azimuth_hi_deg']:+.1f} deg",
                        SKIP))

        pc = expect.get("pointcloud_fields")
        if pc:
            got = [(f["name"], f["datatype"], f["count"]) for f in ex.get("fields", [])]
            wanted = [(f["name"], f["datatype"], f.get("count", 1)) for f in pc]
            dev = _deviation_for(spec, "pointcloud/fields")
            ok = got == wanted
            out.append(Finding("C" if (dev and not ok) else "A", "pointcloud/fields", name,
                               ", ".join(f"{n}:{d}" for n, d, _ in wanted),
                               ", ".join(f"{n}:{d}" for n, d, _ in got) or "-",
                               OK if ok else (SKIP if dev else FAIL), dev))
            if expect.get("point_step") is not None:
                ok = ex.get("point_step") == expect["point_step"]
                out.append(Finding("C" if (dev and not ok) else "A", "pointcloud/point_step", name,
                                   _f(expect["point_step"]), _f(ex.get("point_step")),
                                   OK if ok else (SKIP if dev else FAIL), dev))

        ci = expect.get("camera_info")
        if ci:
            for key in ("width", "height", "distortion_model"):
                if key in ci:
                    out.append(Finding("B", "camera_info/" + key, name, _f(ci[key]), _f(ex.get(key)),
                                       OK if ex.get(key) == ci[key] else FAIL))
            if "K" in ci and ex.get("K"):
                worst, worst_i = 0.0, -1
                for i, (a, b) in enumerate(zip(ex["K"], ci["K"])):
                    if b == 0:
                        continue
                    rel = abs(a - b) / abs(b)
                    if rel > worst:
                        worst, worst_i = rel, i
                out.append(Finding("B", "camera_info/K", name, f"within {TOL_K_FRACTION:.0%}",
                                   f"worst {worst:.2%} at K[{worst_i}]",
                                   OK if worst <= TOL_K_FRACTION else FAIL))
            if "D" in ci and ex.get("D") is not None:
                same = list(ex["D"]) == list(ci["D"])
                out.append(Finding("B", "camera_info/D", name, _f(ci["D"]), _f(ex.get("D")),
                                   OK if same else FAIL))
    return out


def check_tf_dynamic(contract: Dict[str, Any], obs: Observation) -> List[Finding]:
    """Edges the twin must publish on /tf, and fast enough to be usable.

    A dynamic transform that is present but slow is nearly as bad as a missing one:
    tf2 extrapolates between samples, so a 5 Hz waist edge means every cloud
    reprojection is interpolating over 200 ms of body motion.
    """
    out: List[Finding] = []
    for e in contract.get("tf_static", []):
        if not e.get("dynamic"):
            continue
        label = f"{e['parent']} -> {e['child']}"
        seen = obs.tf_dynamic.get((e["parent"], e["child"]))
        want = float(e["rate_hz"])
        floor = want * 0.5
        if not seen:
            out.append(Finding("A", "tf_dynamic/edge", label, f"on /tf at >={floor:.0f} Hz",
                               "ABSENT from /tf", FAIL))
            continue
        out.append(Finding("A", "tf_dynamic/edge", label, "on /tf", "present", OK))
        out.append(Finding("B", "tf_dynamic/rate", label, f">={floor:.0f} Hz (target {want:g})",
                           f"{seen:.2f} Hz", OK if seen >= floor else FAIL))
    return out


def check_provenance(contract: Dict[str, Any]) -> List[Finding]:
    """Surface every `assumed` value. An assumption nobody sees becomes a fact."""
    out: List[Finding] = []
    for s in contract.get("sensors", []):
        if (s.get("pose") or {}).get("provenance") == "assumed":
            out.append(Finding("C", "provenance/assumed", f"sensor {s['name']} pose",
                               "calibrated or datasheet", "assumed", SKIP,
                               s["pose"].get("source", "")))
    for e in contract.get("tf_static", []):
        if e.get("provenance") == "assumed":
            out.append(Finding("C", "provenance/assumed", f"tf {e['parent']}->{e['child']}",
                               "calibrated", "assumed", SKIP, e.get("source", "")))
    for t in contract.get("topics", []):
        if t.get("provenance") == "assumed":
            out.append(Finding("C", "provenance/assumed", f"topic {t['name']}",
                               "measured", "assumed", SKIP, t.get("source", "")))
    return out


# ------------------------------------------------------------------ reporting


def render(findings: List[Finding], obs: Observation, contract_path: str, quiet: bool) -> None:
    w_check = max([len(f.check) for f in findings] + [12])
    w_subj = min(44, max([len(f.subject) for f in findings] + [10]))
    print(f"twin_audit  contract={contract_path}")
    print(f"            source={obs.label}")
    print()
    header = f"{'CLS':<4}{'STATUS':<7}{'CHECK':<{w_check + 2}}{'SUBJECT':<{w_subj + 2}}EXPECTED -> ACTUAL"
    print(header)
    print("-" * min(len(header) + 40, 150))
    for f in findings:
        if quiet and f.status == OK:
            continue
        line = (f"{f.parity:<4}{f.status:<7}{f.check:<{w_check + 2}}{f.subject[:w_subj]:<{w_subj + 2}}"
                f"{f.expected}  ->  {f.actual}")
        print(line)
        if f.note:
            print(" " * (4 + 7 + w_check + 2) + f"note: {f.note}")
    for n in obs.notes:
        print(f"\n[source note] {n}")


def summarise(findings: List[Finding]) -> Dict[str, int]:
    s = {"A_fail": 0, "B_fail": 0, "pass": 0, "skip": 0, "total": len(findings)}
    for f in findings:
        if f.status == OK:
            s["pass"] += 1
        elif f.status == SKIP:
            s["skip"] += 1
        elif f.parity == "A":
            s["A_fail"] += 1
        else:
            s["B_fail"] += 1
    return s


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Audit a ROS 2 interface against a twin contract.")
    ap.add_argument("--contract", required=True)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--bag", help="audit a rosbag2 directory instead of the live graph")
    src.add_argument("--against-evidence", dest="evidence",
                     help="audit the contract against a driver evidence/ directory")
    ap.add_argument("--duration", type=float, default=12.0, help="live capture seconds (default 12)")
    ap.add_argument("--json", help="also write findings as JSON here")
    ap.add_argument("--quiet", action="store_true", help="print only non-PASS rows")
    args = ap.parse_args(argv)

    try:
        contract = twin_contract.load(args.contract)
    except twin_contract.ContractError as exc:
        print(f"twin_audit: contract invalid: {exc}", file=sys.stderr)
        return 2

    names = [t["name"] for t in contract["topics"]]
    try:
        if args.evidence:
            obs = twin_sources.observe_evidence(args.evidence)
        elif args.bag:
            obs = twin_sources.observe_bag(args.bag, names)
        else:
            obs = twin_sources.observe_live(names, args.duration)
    except Exception as exc:  # a source that cannot run must not look like a pass
        print(f"twin_audit: could not observe: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    findings = (check_topics(contract, obs) + check_tf_static(contract, obs)
                + check_tf_dynamic(contract, obs) + check_payloads(contract, obs)
                + check_provenance(contract))
    render(findings, obs, args.contract, args.quiet)

    s = summarise(findings)
    print()
    print(f"summary: {s['pass']} pass, {s['A_fail']} Class-A FAIL, "
          f"{s['B_fail']} Class-B fail, {s['skip']} skipped, {s['total']} checks")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"contract": args.contract, "source": obs.label,
                       "summary": s, "findings": [asdict(f) for f in findings],
                       "notes": obs.notes}, fh, indent=2)
        print(f"wrote {args.json}")

    if s["A_fail"]:
        print(f"\nRESULT: NON-CONFORMANT -- {s['A_fail']} Class-A difference(s) from the contract.")
        return 1
    print("\nRESULT: conformant on Class A.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
