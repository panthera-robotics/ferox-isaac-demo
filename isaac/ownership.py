# PANTHERA (Sprint N, wb-cand-N01): body/arm/hand OWNERSHIP ARBITER for the deployment runtime. Off unless G1_OWNERSHIP=1.
#
# One writer per joint per physics step, always the runtime itself; the arbiter only decides WHICH SOURCE each joint
# group's targets come from and journals every transition:
#   WALK       locomotion+arms <- policy (walk owner); /cmd_vel honoured; hands untouched (USD-authored drives)
#   SETTLING   /cmd_vel forced to zero; policy still writes locomotion+arms; waits until |v_xy| < v_settle for settle_s
#   MANIP      locomotion <- policy (balance only, command zero); arms (+hands if given) <- manipulation source
#   RETURNING  arms ramp from the last manipulation targets back to the policy's arm targets over return_s; then WALK
# Manipulation source = the scripted reach schedule (G1_OWNERSHIP_SCRIPT, tonight's test) and/or ROS 2 topics:
#   /wbc/ownership   std_msgs/String  "request_manip" | "release"
#   /wbc/arm_targets sensor_msgs/JointState  name[] + position[] (arm and/or hand joints only; refused outside MANIP)
#   /wbc/owner_state std_msgs/String  JSON {state, owner_by_group, t, refused, ...} at ~10 Hz
# Guard rules: a request is only granted after the settle criterion; targets from a source that does not own the joint
# are refused and counted; no joint STATE is ever written (targets only); every transition and refusal is journaled.
import json
import math
import os
import threading
import time

LOCOMOTION = [
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint", "left_hip_roll_joint", "right_hip_roll_joint",
    "waist_roll_joint", "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint", "left_knee_joint",
    "right_knee_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint", "left_ankle_roll_joint", "right_ankle_roll_joint",
]
ARMS = [
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint", "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint", "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
]


def enabled() -> bool:
    return os.environ.get("G1_OWNERSHIP", "").strip().lower() in ("1", "true", "yes", "on")


class StationKeeper:
    """Sprint P M1 — locomotion-owner station keeper on the walking policy's OWN velocity-command interface.
    A station frame (pelvis x, y, yaw in the world) is latched at the manipulation-owner grant, at an explicit lock, or (mode
    'idle') whenever the walk owner's command has been zero and the base settled; every control tick the keeper commands
        v_world = -K_xy * softdead(e_xy),  wz = -K_yaw * softdead(e_yaw),  e = current - target,
    rotated into the body frame, clamped to the bounds and slew-limited. Pure P (no integral -> nothing to wind up); no forces,
    no constraints, no policy change: the numbers go where /cmd_vel goes. The measurement source is the simulator's pelvis
    (articulation root) pose, the same source the /odom publisher uses. Pure Python (CPU-testable)."""

    def __init__(self, *, k=(1.0, 1.0, 1.0), vmax=(0.3, 0.3, 0.5), deadband=(0.015, math.radians(2.0)), slew=(0.5, 1.0),
                 journal=None, trace_path=None, every=10):
        self.k = [float(v) for v in k]; self.vmax = [abs(float(v)) for v in vmax]
        self.db_xy = float(deadband[0]); self.db_yaw = float(deadband[1]); self.slew_xy = float(slew[0]); self.slew_yaw = float(slew[1])
        self.target = None                      # (x, y, yaw)
        self.locked_at = None; self.lock_source = None
        self.cmd = [0.0, 0.0, 0.0]              # last commanded (vx, vy, wz) in the body frame
        self.err = [0.0, 0.0, 0.0]              # last (ex, ey) world m, eyaw rad
        self._journal = journal; self._trace = open(trace_path, "a", encoding="utf-8") if trace_path else None
        self.every = int(every); self._n = 0; self.locks = 0

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _softdead(e, db):
        # continuous deadband: zero inside +-db, then linear from zero (no jump at the edge)
        if abs(e) <= db:
            return 0.0
        return e - db if e > 0 else e + db

    def lock(self, x, y, yaw, t, source="explicit"):
        self.target = (float(x), float(y), float(yaw)); self.locked_at = float(t); self.lock_source = source; self.locks += 1
        self.cmd = [0.0, 0.0, 0.0]
        if self._journal: self._journal("station_lock", {"x": round(float(x), 4), "y": round(float(y), 4), "yaw_deg": round(math.degrees(yaw), 2), "source": source, "n": self.locks})

    def unlock(self, t, source="explicit"):
        if self.target is None:
            return
        if self._journal: self._journal("station_unlock", {"source": source, "held_s": round(float(t) - self.locked_at, 2), "last_err_xy_m": round(math.hypot(self.err[0], self.err[1]), 4), "last_err_yaw_deg": round(math.degrees(self.err[2]), 2)})
        self.target = None; self.locked_at = None; self.lock_source = None; self.cmd = [0.0, 0.0, 0.0]

    @property
    def active(self):
        return self.target is not None

    def step(self, dt, x, y, yaw, t, state=""):
        """Returns [vx, vy, wz] (body frame) while locked, else None."""
        self._n += 1
        if self.target is None:
            return None
        ex = float(x) - self.target[0]; ey = float(y) - self.target[1]; eyaw = self._wrap(float(yaw) - self.target[2])
        self.err = [ex, ey, eyaw]
        # world-frame velocity demand, then into the body (yaw) frame: v_body = R(-yaw) v_world
        vwx = -self.k[0] * self._softdead(ex, self.db_xy); vwy = -self.k[1] * self._softdead(ey, self.db_xy)
        c, s_ = math.cos(yaw), math.sin(yaw)
        want = [c * vwx + s_ * vwy, -s_ * vwx + c * vwy, -self.k[2] * self._softdead(eyaw, self.db_yaw)]
        for i in range(3):
            want[i] = max(-self.vmax[i], min(self.vmax[i], want[i]))
        # slew limit (command acceleration bound) against the previous command
        dmax = [self.slew_xy * dt, self.slew_xy * dt, self.slew_yaw * dt]
        for i in range(3):
            d = want[i] - self.cmd[i]
            self.cmd[i] += max(-dmax[i], min(dmax[i], d))
        if self._trace and self._n % self.every == 0:
            self._trace.write(json.dumps({"t": round(float(t), 4), "state": state, "held_s": round(float(t) - self.locked_at, 3), "pos": [round(float(x), 4), round(float(y), 4)], "yaw_deg": round(math.degrees(yaw), 2),
                                          "err_xy_m": [round(ex, 4), round(ey, 4)], "err_norm_m": round(math.hypot(ex, ey), 4), "err_yaw_deg": round(math.degrees(eyaw), 2),
                                          "cmd": [round(v, 4) for v in self.cmd]}) + "\n")
            if self._n % (self.every * 20) == 0: self._trace.flush()
        return list(self.cmd)


class ArmSwingOverlay:
    """Sprint P M3 — presentation arm swing on the policy's arm targets while the WALK owner is walking. Opposite-phase shoulder
    pitch derived from the measured gait itself (delta_R = +k * (hip_pitch_L - hip_pitch_R), delta_L = -delta_R), so it is
    phase-locked to the legs and vanishes when they stand still; amplitude-clamped, rate-limited, blended in over >= 0.5 s
    once the base is commanded (|cmd| > v_on) in WALK and blended out over >= 0.5 s when the command drops or ownership
    leaves WALK (a manipulation request); zero before the handoff. Pure Python (CPU-testable)."""

    def __init__(self, amplitude=0.15, gain=0.25, blend_s=0.5, v_on=0.05, max_rate=3.0, bound=(-0.6, 0.6)):
        self.A = float(amplitude); self.gain = float(gain); self.blend_s = max(0.5, float(blend_s)); self.v_on = float(v_on); self.max_rate = float(max_rate); self.bound = bound
        self.w = 0.0; self.delta = [0.0, 0.0]; self.active = False

    def step(self, dt, state, cmd, hip_l, hip_r, base_l, base_r):
        """Returns (left_shoulder_pitch_target, right_shoulder_pitch_target) or None when the overlay is fully out."""
        moving = state == "WALK" and math.hypot(float(cmd[0]), float(cmd[1])) > self.v_on
        target_w = 1.0 if moving else 0.0
        step = dt / self.blend_s
        self.w += max(-step, min(step, target_w - self.w)); self.w = max(0.0, min(1.0, self.w))
        raw = self.gain * (float(hip_l) - float(hip_r)); raw = max(-self.A, min(self.A, raw)) * self.w
        want = [-raw, raw]                                   # left, right (opposite phase)
        for i in range(2):
            d = want[i] - self.delta[i]; lim = self.max_rate * dt
            self.delta[i] += max(-lim, min(lim, d))
        self.active = self.w > 1e-6 or any(abs(v) > 1e-6 for v in self.delta)
        if not self.active:
            return None
        lo, hi = self.bound
        return (max(lo, min(hi, float(base_l) + self.delta[0])), max(lo, min(hi, float(base_r) + self.delta[1])))


class OwnershipArbiter:
    """Pure-Python state machine (no Isaac/ROS imports) so it can be unit-tested on CPU."""

    def __init__(self, policy_names, all_dof_names, *, v_settle=0.05, settle_s=1.0, return_s=1.0, journal_path=None):
        self.policy_names = list(policy_names)
        self.all_names = list(all_dof_names)
        self.arm_idx = [self.policy_names.index(n) for n in ARMS if n in self.policy_names]
        self.loco_idx = [self.policy_names.index(n) for n in LOCOMOTION if n in self.policy_names]
        self.hand_names = [n for n in self.all_names if n not in self.policy_names]
        self.hand_idx_all = [self.all_names.index(n) for n in self.hand_names]
        self.v_settle = float(v_settle); self.settle_s = float(settle_s); self.return_s = float(return_s)
        self.state = "WALK"; self.t = 0.0; self._settled_since = None; self._return_t0 = None
        self._pending_request = False; self._pending_release = False
        self._manip_arm = None          # dict arm joint name -> target (rad), last received
        self._manip_hand = None         # dict hand joint name -> target (rad)
        self._return_from = None
        self.refused = {"arm_targets_outside_manip": 0, "unknown_joint": 0, "request_while_moving_queued": 0}
        self.transitions = []
        self._lock = threading.RLock()
        self.hooks = {"grant": [], "walk": []}      # callables(arbiter) run on SETTLING->MANIP and RETURNING->WALK
        self._manip_effort = None                    # dict policy joint name -> extra joint effort (N m) while MANIP/RETURNING
        self._carry_arm = None                       # Sprint O W3: dict arm joint -> target held by the "carry_hold" source outside MANIP
        self._carry_hand = None                      # dict hand joint -> target kept while carrying (grip persists across ownership)
        self.station = None; self.station_mode = "manip"; self.station_cmd = None   # Sprint P M1 (attach_station)
        self._idle_since = None; self._ext_cmd_zero = True
        self._journal = open(journal_path, "a", encoding="utf-8") if journal_path else None
        self._log("init", {"policy_joints": len(self.policy_names), "arms": len(self.arm_idx), "locomotion": len(self.loco_idx),
                           "hands": len(self.hand_names), "v_settle": self.v_settle, "settle_s": self.settle_s, "return_s": self.return_s})

    # ---- journal -------------------------------------------------------------------------------------------------------
    def _log(self, event, detail=None):
        row = {"t": round(self.t, 4), "wall": time.time(), "event": event, "state": self.state, "detail": detail}
        self.transitions.append(row)
        if self._journal:
            self._journal.write(json.dumps(row) + "\n"); self._journal.flush()
        print(f"[ownership] t={self.t:8.3f} {event} state={self.state} {json.dumps(detail) if detail else ''}", flush=True)

    def owner_by_group(self):
        if self.state in ("WALK", "SETTLING"):
            return {"locomotion": "policy", "arms": "carry_hold" if self._carry_arm else "policy", "hands": "carry_hold" if self._carry_hand else "authored"}
        if self.state == "MANIP":
            return {"locomotion": "policy(balance)", "arms": "manipulation", "hands": "manipulation" if self._manip_hand else "authored"}
        return {"locomotion": "policy(balance)", "arms": "return_ramp", "hands": "authored"}

    def status(self):
        with self._lock:
            return {"state": self.state, "t": round(self.t, 3), "owner_by_group": self.owner_by_group(), "refused": dict(self.refused),
                    "pending_request": self._pending_request, "manip_arm_joints": sorted(self._manip_arm) if self._manip_arm else [],
                    "manip_hand_joints": sorted(self._manip_hand) if self._manip_hand else []}

    # ---- external inputs (any thread) -----------------------------------------------------------------------------------
    def request_manip(self, source="topic"):
        with self._lock:
            if self.state == "WALK":
                self._pending_request = True; self._log("request_manip", {"source": source})
            elif self.state in ("MANIP", "SETTLING"):
                self._log("request_manip_ignored", {"source": source, "reason": "already " + self.state})
            else:
                self._pending_request = True; self._log("request_manip_queued", {"source": source, "reason": self.state})

    def release(self, source="topic"):
        with self._lock:
            if self.state == "MANIP":
                self._pending_release = True; self._log("release", {"source": source})
            else:
                self._pending_request = False; self._log("release_ignored", {"source": source, "reason": self.state})

    def set_carry_hold(self, arm_targets, hand_targets, source="manip"):
        """Arm/hand targets kept by the carry_hold source after the manipulation owner releases (WALK_CARRY): the policy keeps
        the locomotion group, the arms listed here stay at these targets, the hand keeps its grip targets."""
        with self._lock:
            self._carry_arm = {n: float(q) for n, q in (arm_targets or {}).items() if n in ARMS}
            self._carry_hand = {n: float(q) for n, q in (hand_targets or {}).items() if n in self.hand_names} or None
            self._log("carry_hold_set", {"source": source, "arms": sorted(self._carry_arm), "hands": sorted(self._carry_hand or {})})

    def clear_carry_hold(self, source="manip"):
        with self._lock:
            had = bool(self._carry_arm); self._carry_arm = None; self._carry_hand = None
            self._log("carry_hold_cleared", {"source": source, "had_hold": had})

    def set_efforts(self, efforts):
        """Extra joint efforts (N m) from the manipulation source for joints it owns; None clears."""
        with self._lock:
            self._manip_effort = dict(efforts) if efforts else None

    def set_targets(self, names, positions, source="topic"):
        """Arm/hand targets from the manipulation source. Refused (counted) unless the arbiter is in MANIP."""
        with self._lock:
            if self.state != "MANIP":
                self.refused["arm_targets_outside_manip"] += 1
                if self.refused["arm_targets_outside_manip"] in (1, 10, 100):
                    self._log("targets_refused", {"source": source, "reason": "state " + self.state, "count": self.refused["arm_targets_outside_manip"]})
                return False
            arm = dict(self._manip_arm or {}); hand = dict(self._manip_hand or {})
            for n, q in zip(names, positions):
                if n in ARMS: arm[n] = float(q)
                elif n in self.hand_names: hand[n] = float(q)
                else:
                    self.refused["unknown_joint"] += 1
                    self._log("target_refused_unknown_joint", {"source": source, "joint": n}); return False
            self._manip_arm = arm; self._manip_hand = hand or None
            return True

    # ---- per physics step (sim thread) ---------------------------------------------------------------------------------
    def attach_station(self, keeper, mode="manip"):
        """mode 'manip': latch at the manipulation grant, unlatch at RETURNING->WALK; 'idle': additionally latch during WALK once
        the external command has been zero for >= 1 s and the base settled, unlatch when a non-zero command arrives (re-latched
        to the grant pose at a manipulation grant, per the packet)."""
        self.station = keeper; self.station_mode = str(mode)
        self._log("station_attached", {"mode": self.station_mode, "k": keeper.k, "vmax": keeper.vmax, "deadband_m_rad": [keeper.db_xy, keeper.db_yaw], "slew": [keeper.slew_xy, keeper.slew_yaw]})

    def note_external_command(self, cmd):
        """The walk owner's external command (vx, vy, wz) this tick, for the 'idle' station mode."""
        self._ext_cmd_zero = all(abs(float(v)) < 1e-6 for v in cmd)

    def _station_tick(self, dt, base_xy, base_yaw, base_speed_xy):
        """Latch/unlatch bookkeeping + the command; called inside step() with the lock held (base pose may be None -> no keeper)."""
        k = self.station
        if k is None or base_xy is None or base_yaw is None:
            self.station_cmd = None; return
        if self.state == "WALK" and self.station_mode == "idle":
            if self._ext_cmd_zero:
                if base_speed_xy < self.v_settle:
                    if self._idle_since is None: self._idle_since = self.t
                    if not k.active and self.t - self._idle_since >= 1.0:
                        k.lock(base_xy[0], base_xy[1], base_yaw, self.t, source="idle")
                else:
                    self._idle_since = None
            else:
                self._idle_since = None
                if k.active and k.lock_source in ("idle", "grant"):
                    k.unlock(self.t, source="external_command")
        self.station_cmd = k.step(dt, base_xy[0], base_xy[1], base_yaw, self.t, state=self.state)

    def step(self, dt, base_speed_xy, base_z, policy_targets, base_xy=None, base_yaw=None):
        """policy_targets: full policy-order target vector (numpy array) as the policy computed it. Returns
        (targets_to_write, hand_targets_dict_or_None, command_allowed, extra_efforts_dict_or_None); with a station keeper
        attached, self.station_cmd carries the body-frame (vx, vy, wz) to command instead of the external command (None = none)."""
        import numpy as np
        with self._lock:
            self.t += float(dt)
            self._station_tick(dt, base_xy, base_yaw, base_speed_xy)
            out = np.array(policy_targets, dtype=np.float32, copy=True)
            hold_hands = dict(self._carry_hand) if self._carry_hand else None
            if self._carry_arm:
                for n, q in self._carry_arm.items():
                    out[self.policy_names.index(n)] = q
            if self.state == "WALK":
                if self._pending_request:
                    self._pending_request = False; self._settled_since = None
                    self.state = "SETTLING"; self._log("WALK->SETTLING", {"speed": round(float(base_speed_xy), 4)})
                else:
                    return out, hold_hands, True, None
            if self.state == "SETTLING":
                if base_speed_xy < self.v_settle:
                    if self._settled_since is None: self._settled_since = self.t
                    if self.t - self._settled_since >= self.settle_s:
                        self.state = "MANIP"; self._manip_arm = None; self._manip_hand = None
                        self._log("SETTLING->MANIP", {"speed": round(float(base_speed_xy), 4), "z": round(float(base_z), 4)})
                        if self.station is not None and base_xy is not None and base_yaw is not None:
                            self.station.lock(base_xy[0], base_xy[1], base_yaw, self.t, source="grant")
                            self.station_cmd = self.station.step(0.0, base_xy[0], base_xy[1], base_yaw, self.t, state=self.state)
                        for h in self.hooks["grant"]:
                            h(self)
                else:
                    self._settled_since = None
                return out, hold_hands, False, None
            if self.state == "MANIP":
                if self._manip_arm:
                    for n, q in self._manip_arm.items():
                        out[self.policy_names.index(n)] = q
                hands = dict(self._manip_hand) if self._manip_hand else hold_hands
                if self._pending_release:
                    self._pending_release = False
                    self._return_from = {n: float(out[self.policy_names.index(n)]) for n in ARMS}
                    self._return_t0 = self.t; self.state = "RETURNING"; self._log("MANIP->RETURNING", {"return_s": self.return_s})
                return out, hands, False, (dict(self._manip_effort) if self._manip_effort else None)
            # RETURNING: linear ramp from the last manipulation arm targets to the live policy arm targets (or to the carry hold)
            a = min(1.0, (self.t - self._return_t0) / max(1e-6, self.return_s))
            for n, q0 in self._return_from.items():
                i = self.policy_names.index(n)
                dest = self._carry_arm[n] if (self._carry_arm and n in self._carry_arm) else float(policy_targets[i])
                out[i] = (1.0 - a) * q0 + a * dest
            eff = {n: (1.0 - a) * v for n, v in self._manip_effort.items()} if self._manip_effort else None
            if a >= 1.0:
                self.state = "WALK"; self._return_from = None; self._manip_effort = None; self._log("RETURNING->WALK", {})
                if self.station is not None and self.station.active and self.station_mode == "manip":
                    self.station.unlock(self.t, source="return_to_walk")
                    self.station_cmd = None
                for h in self.hooks["walk"]:
                    h(self)
                return out, hold_hands, True, None
            return out, hold_hands, False, eff


class ScriptedManipulationSource:
    """Tonight's manipulation owner: repeats {request -> (after grant) reach keyframes -> release} `cycles` times.
    G1_OWNERSHIP_SCRIPT = {"cycles": 3, "first_request_s": 10, "gap_s": 6, "schedule": {"joints": [...], "keyframes": [{"t","q"}...]}}
    Keyframe t is seconds since the MANIP grant; the source releases when the last keyframe has been held for hold_end_s."""

    def __init__(self, spec, arbiter):
        self.spec = spec; self.arb = arbiter
        self.cycles = int(spec.get("cycles", 1)); self.first = float(spec.get("first_request_s", 10.0)); self.gap = float(spec.get("gap_s", 6.0))
        self.hold_end = float(spec.get("hold_end_s", 1.0))
        # Sprint O W2: an optional list of named schedules ("schedules": [{"candidate","joints","keyframes"}...]) is played
        # interleaved, cycle k -> schedules[k % n], so an early fall still leaves every candidate with the same number of
        # completed cycles; each request is journaled as source "script:<candidate>" for per-candidate scoring.
        self.schedules = [dict(sc) for sc in spec.get("schedules", [])] or [dict(spec["schedule"], candidate=spec.get("candidate", "script"))]
        self._select(0)
        self.done = 0; self._grant_t = None; self._next_request_t = self.first; self._released_t = None
        self._last_state = None

    def _select(self, cycle):
        sch = self.schedules[cycle % len(self.schedules)]
        self.candidate = str(sch.get("candidate", "script")); self.joints = list(sch["joints"]); self.keys = sorted(sch["keyframes"], key=lambda k: float(k["t"]))

    def step(self, t, state):
        if state != self._last_state:
            if state == "MANIP": self._grant_t = t
            if state == "WALK" and self._last_state == "RETURNING":
                self.done += 1; self._next_request_t = t + self.gap
                print(f"[ownership script] cycle {self.done}/{self.cycles} ({self.candidate}) complete at t={t:.2f}", flush=True)
                self._select(self.done)
            self._last_state = state
        if state == "WALK" and self.done < self.cycles and t >= self._next_request_t:
            self.arb.request_manip(source="script" if len(self.schedules) == 1 and self.candidate == "script" else f"script:{self.candidate}"); self._next_request_t = float("inf")
        if state == "MANIP" and self._grant_t is not None:
            import numpy as np
            tt = [float(k["t"]) for k in self.keys]; s = t - self._grant_t
            q = [float(np.interp(s, tt, [float(k["q"][j]) for k in self.keys])) for j in range(len(self.joints))]
            self.arb.set_targets(self.joints, q, source="script")
            if s >= tt[-1] + self.hold_end:
                self.arb.release(source="script"); self._grant_t = None


class BodyTrace:
    """Per-step body trace for the arbiter path (Sprint O W2 prefilter): pelvis pose/rpy, base velocities, both feet
    (ankle_roll links) world positions, foot-lift flags and a step counter; one JSONL row every `every` physics steps.
    Read-only; enabled by G1_OWNERSHIP_TRACE=1 (path next to the ownership journal)."""

    def __init__(self, robot, arbiter, path, every=10, lift_m=0.02):
        import numpy as np
        self.robot = robot; self.arb = arbiter; self.every = int(every); self.lift = float(lift_m)
        self.f = open(path, "a", encoding="utf-8"); self.n = 0
        self.view = robot._articulation_view
        self.links = list(self.view.body_names) if hasattr(self.view, "body_names") else []
        self.feet = {k: (self.links.index(k) if k in self.links else None) for k in ("left_ankle_roll_link", "right_ankle_roll_link")}
        self.foot_z0 = {}; self.foot_up = {k: False for k in self.feet}; self.steps = {k: 0 for k in self.feet}
        self._np = np

    def _link(self, i):
        tf = self._np.asarray(self.view._physics_view.get_link_transforms()).reshape(-1, len(self.links), 7)[0][i]
        return [float(v) for v in tf[:3]]

    def step(self):
        self.n += 1
        if self.n % self.every:
            return
        np = self._np
        try:
            pos, quat = self.robot.get_world_pose(); lin = self.robot.get_linear_velocity(); ang = self.robot.get_angular_velocity()
            w, x, y, z = [float(v) for v in quat]
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)); pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))); yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            feet = {}
            for k, i in self.feet.items():
                if i is None: continue
                p = self._link(i); feet[k] = [round(v, 4) for v in p]
                if k not in self.foot_z0: self.foot_z0[k] = p[2]
                up = p[2] - self.foot_z0[k] > self.lift
                if up and not self.foot_up[k]: self.steps[k] += 1
                self.foot_up[k] = up
            row = {"t": round(self.arb.t, 4), "state": self.arb.state, "owners": self.arb.owner_by_group(),
                   "pelvis": {"pos": [round(float(v), 4) for v in pos], "rpy_deg": [round(math.degrees(roll), 2), round(math.degrees(pitch), 2), round(math.degrees(yaw), 2)]},
                   "base_speed_xy": round(float(np.hypot(float(lin[0]), float(lin[1]))), 4), "base_ang_speed": round(float(np.linalg.norm(np.asarray(ang, float))), 4),
                   "feet": feet, "foot_up": dict(self.foot_up), "steps": dict(self.steps)}
            self.f.write(json.dumps(row) + "\n")
            if self.n % (self.every * 20) == 0: self.f.flush()   # every ~1 s sim: a boot-ending fall keeps its last rows
        except Exception as exc:
            if self.n % 2000 == 0: print(f"[ownership trace] error {exc!r}", flush=True)


def setup_ros(arbiter):
    """ROS 2 interface on its own rclpy node (rclpy is already initialised by the cmd_vel subscriber)."""
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from sensor_msgs.msg import JointState
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("wbc_ownership")
        pub = node.create_publisher(String, "/wbc/owner_state", 10)

        def _own(msg):
            cmd = msg.data.strip().lower()
            if cmd in ("request_manip", "manip"): arbiter.request_manip(source="topic")
            elif cmd in ("release", "walk"): arbiter.release(source="topic")
            else: print(f"[ownership] unknown /wbc/ownership command {cmd!r}", flush=True)

        def _targets(msg):
            arbiter.set_targets(list(msg.name), list(msg.position), source="topic")

        node.create_subscription(String, "/wbc/ownership", _own, 10)
        node.create_subscription(JointState, "/wbc/arm_targets", _targets, 10)

        def _tick():
            m = String(); m.data = json.dumps(arbiter.status()); pub.publish(m)

        node.create_timer(0.1, _tick)
        ex = SingleThreadedExecutor(); ex.add_node(node)

        def _spin():
            while rclpy.ok():
                ex.spin_once(timeout_sec=0.1)

        threading.Thread(target=_spin, daemon=True).start()
        print("[ownership] ROS 2 interface up: /wbc/ownership, /wbc/arm_targets -> /wbc/owner_state", flush=True)
        return node
    except Exception as exc:
        print(f"[ownership] ROS 2 interface unavailable ({exc!r}); scripted source only", flush=True)
        return None
