"""Grasp v7 — finger/object contact via the PhysX contact-report CALLBACK.

Why this route and not the other two:

* `get_net_contact_forces` is **absent** on both the articulation wrapper and the
  physics view in this build (`evidence/C39/VERDICT.md`), so the tensor route cannot
  answer "is the finger touching the can" at all. Every `NO_GRIP` row in v5/v6 carries
  `grip_contacts = -1` for that reason and is *not* evidence about colliders.
* `ContactSensor` prims were tried twice in v6 and would not attach: created on the link
  Xform they report 5/5 and then fail `initialize()` because the Xform carries no
  collision, and re-parented onto the collision child they find 0/5.

The callback route needs neither a sensor prim nor any re-parenting. `PhysxContactReportAPI`
is applied to bodies we already have, and PhysX reports pairs as they happen.

Two rules this module keeps, both learned the hard way in this campaign:

1. **Never construct per-body prim wrappers while physics is running.** Building a
   `SingleRigidPrim` per link invalidated the whole physics view mid-episode and ended
   the run. Everything here is stage reads and one subscription.
2. **Absence is reported, never inferred.** If the API is missing or nothing subscribes,
   `counts()` returns `None` — which the caller must treat as "unknown", not "zero".
   A silent zero is what made the grasp workstream chase colliders for two versions.
"""
from __future__ import annotations


class ContactReporter:
    """Counts right-hand finger links currently touching a named target prim."""

    def __init__(self, log=print):
        self.log = log
        self._sub = None
        self._iface = None
        self._finger_paths = set()
        self._target_prefix = ""
        self._hits = {}          # finger path -> accumulated |impulse| this window
        self._last = {}          # finger path -> LAST substep impulse magnitude (N*s)
        self._n_reports = 0
        self.dt = 1.0 / 200.0    # physics substep; set by the caller
        self.available = False
        self._decode = None

    # ------------------------------------------------------------------ setup
    def attach(self, stage, finger_prim_paths, target_prim_path):
        """Apply the report API to fingers + target and subscribe. Idempotent."""
        try:
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysxSchema, PhysicsSchemaTools
        except Exception as exc:
            self.log(f"[contact] PhysX contact-report API unavailable: {exc!r}")
            return False

        self._decode = PhysicsSchemaTools.intToSdfPath
        self._finger_paths = set(str(p) for p in finger_prim_paths)
        self._target_prefix = str(target_prim_path)

        applied = 0
        for path in list(self._finger_paths) + [self._target_prefix]:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                self.log(f"[contact] prim not on stage, SKIPPED: {path}")
                continue
            try:
                api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                # Report every touch. The default threshold is high enough to hide a
                # fingertip resting on a 349 g can, which is exactly the contact we are
                # trying to see.
                api.CreateThresholdAttr().Set(0.0)
                applied += 1
            except Exception as exc:
                self.log(f"[contact] could not apply report API to {path}: {exc!r}")

        if applied == 0:
            self.log("[contact] report API applied to ZERO prims -- route unusable")
            return False

        try:
            self._iface = get_physx_simulation_interface()
            self._sub = self._iface.subscribe_contact_report_events(self._on_report)
        except Exception as exc:
            self.log(f"[contact] subscribe failed: {exc!r}")
            return False

        self.available = True
        self.log(f"[contact] contact-report route ACTIVE: {applied} prim(s), "
                 f"{len(self._finger_paths)} finger link(s), target {self._target_prefix}")
        return True

    # --------------------------------------------------------------- callback
    def _on_report(self, contact_headers, contact_data):
        for h in contact_headers:
            try:
                a0 = str(self._decode(h.actor0))
                a1 = str(self._decode(h.actor1))
            except Exception:
                continue
            # One side must be the target, the other a finger. Prefix match, because
            # the collision geometry sits on a CHILD of the link prim.
            finger = None
            if self._is_target(a0):
                finger = self._match_finger(a1)
            elif self._is_target(a1):
                finger = self._match_finger(a0)
            if finger is None:
                continue
            imp = 0.0
            try:
                for i in range(h.contact_data_offset,
                               h.contact_data_offset + h.num_contact_data):
                    d = contact_data[i]
                    # VECTOR magnitude. Summing |x|+|y|+|z| is an L1 norm and overstates
                    # a diagonal contact by up to sqrt(3).
                    imp += float((float(d.impulse[0]) ** 2
                                  + float(d.impulse[1]) ** 2
                                  + float(d.impulse[2]) ** 2) ** 0.5)
            except Exception:
                imp = max(imp, 1e-9)      # a pair was reported; count the touch
            # PhysX reports an IMPULSE in N*s for this substep, not a force. Keeping the
            # LAST substep's impulse per finger (rather than a running sum over an
            # unknown number of callbacks) makes `counts()` able to divide by dt and
            # report an instantaneous force. The previous code summed impulses across
            # every callback since the last read and printed the total as "N", so the
            # 0.48 that the first closure reported was never newtons and could not be
            # compared against the ~3.4 N a 0.349 kg can needs.
            # PEAK over the window, not the last substep. "Last" made a contact that
            # happened mid-closure vanish if the final substep was free, which turned a
            # real touch into contacts=0 -- under-reporting is as bad as the impulse/force
            # confusion it replaced.
            self._last[finger] = max(self._last.get(finger, 0.0), imp)
            self._hits[finger] = self._hits.get(finger, 0.0) + imp
            self._n_reports += 1

    def _is_target(self, path):
        if path.startswith(self._target_prefix):
            return True
        return path.rsplit("/", 1)[-1] == self._target_prefix.rsplit("/", 1)[-1]

    def _match_finger(self, path):
        for f in self._finger_paths:
            if path.startswith(f):
                return f
        # Fallback by LINK NAME. URDF import instances link geometry under
        # /Flattened_Prototype_NNN/..., so a reported actor path need not sit under the
        # link prim at all -- that is precisely what made v6's re-parented ContactSensor
        # find 0/5. Matching the trailing name recovers those without loosening the
        # target test, which stays a prefix match.
        leaf = path.rsplit("/", 1)[-1]
        for f in self._finger_paths:
            if f.rsplit("/", 1)[-1] == leaf:
                return f
        return None

    # ----------------------------------------------------------------- output
    def counts(self, reset=True):
        """(n_fingers_touching, total_force_N, per_finger_N) or None if unavailable.

        Force, not impulse: PhysX hands back an impulse in N*s for the substep, so the
        instantaneous normal force is `impulse / dt`. Reported per finger from that
        finger's most recent contact substep.
        """
        if not self.available:
            return None
        last = dict(self._last)
        if reset:
            self._hits = {}
            self._last = {}
        dt = max(float(self.dt), 1e-9)
        mags = [last.get(f, 0.0) / dt for f in sorted(self._finger_paths)]
        return sum(1 for m in mags if m > 0.0), float(sum(mags)), mags

    def close(self):
        self._sub = None
        self.available = False
