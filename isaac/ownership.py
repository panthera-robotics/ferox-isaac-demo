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
        self._lock = threading.Lock()
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
            return {"locomotion": "policy", "arms": "policy", "hands": "authored"}
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
    def step(self, dt, base_speed_xy, base_z, policy_targets):
        """policy_targets: full policy-order target vector (numpy array) as the policy computed it. Returns
        (targets_to_write, hand_targets_dict_or_None, command_allowed)."""
        import numpy as np
        with self._lock:
            self.t += float(dt)
            out = np.array(policy_targets, dtype=np.float32, copy=True)
            if self.state == "WALK":
                if self._pending_request:
                    self._pending_request = False; self._settled_since = None
                    self.state = "SETTLING"; self._log("WALK->SETTLING", {"speed": round(float(base_speed_xy), 4)})
                else:
                    return out, None, True
            if self.state == "SETTLING":
                if base_speed_xy < self.v_settle:
                    if self._settled_since is None: self._settled_since = self.t
                    if self.t - self._settled_since >= self.settle_s:
                        self.state = "MANIP"; self._manip_arm = None; self._manip_hand = None
                        self._log("SETTLING->MANIP", {"speed": round(float(base_speed_xy), 4), "z": round(float(base_z), 4)})
                else:
                    self._settled_since = None
                return out, None, False
            if self.state == "MANIP":
                if self._manip_arm:
                    for n, q in self._manip_arm.items():
                        out[self.policy_names.index(n)] = q
                hands = dict(self._manip_hand) if self._manip_hand else None
                if self._pending_release:
                    self._pending_release = False
                    self._return_from = {n: float(out[self.policy_names.index(n)]) for n in ARMS}
                    self._return_t0 = self.t; self.state = "RETURNING"; self._log("MANIP->RETURNING", {"return_s": self.return_s})
                return out, hands, False
            # RETURNING: linear ramp from the last manipulation arm targets to the live policy arm targets
            a = min(1.0, (self.t - self._return_t0) / max(1e-6, self.return_s))
            for n, q0 in self._return_from.items():
                i = self.policy_names.index(n); out[i] = (1.0 - a) * q0 + a * float(policy_targets[i])
            if a >= 1.0:
                self.state = "WALK"; self._return_from = None; self._log("RETURNING->WALK", {})
                return out, None, True
            return out, None, False


class ScriptedManipulationSource:
    """Tonight's manipulation owner: repeats {request -> (after grant) reach keyframes -> release} `cycles` times.
    G1_OWNERSHIP_SCRIPT = {"cycles": 3, "first_request_s": 10, "gap_s": 6, "schedule": {"joints": [...], "keyframes": [{"t","q"}...]}}
    Keyframe t is seconds since the MANIP grant; the source releases when the last keyframe has been held for hold_end_s."""

    def __init__(self, spec, arbiter):
        self.spec = spec; self.arb = arbiter
        self.cycles = int(spec.get("cycles", 1)); self.first = float(spec.get("first_request_s", 10.0)); self.gap = float(spec.get("gap_s", 6.0))
        self.hold_end = float(spec.get("hold_end_s", 1.0))
        sch = spec["schedule"]; self.joints = list(sch["joints"]); self.keys = sorted(sch["keyframes"], key=lambda k: float(k["t"]))
        self.done = 0; self._grant_t = None; self._next_request_t = self.first; self._released_t = None
        self._last_state = None

    def step(self, t, state):
        if state != self._last_state:
            if state == "MANIP": self._grant_t = t
            if state == "WALK" and self._last_state == "RETURNING":
                self.done += 1; self._next_request_t = t + self.gap
                print(f"[ownership script] cycle {self.done}/{self.cycles} complete at t={t:.2f}", flush=True)
            self._last_state = state
        if state == "WALK" and self.done < self.cycles and t >= self._next_request_t:
            self.arb.request_manip(source="script"); self._next_request_t = float("inf")
        if state == "MANIP" and self._grant_t is not None:
            import numpy as np
            tt = [float(k["t"]) for k in self.keys]; s = t - self._grant_t
            q = [float(np.interp(s, tt, [float(k["q"][j]) for k in self.keys])) for j in range(len(self.joints))]
            self.arb.set_targets(self.joints, q, source="script")
            if s >= tt[-1] + self.hold_end:
                self.arb.release(source="script"); self._grant_t = None


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
