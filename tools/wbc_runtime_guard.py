"""Enforced per-step control-ownership boundary for standing/reaching probes (CPU, no Isaac).

The probe calls the guard before every actuator write and after every physics step. The
guard is the runtime form of the whole-body seam rules: one body owner, named 29-joint
commands with no finger names, finite/fresh state, bounded target rate, hands owned
separately and never below the declared margin, support declared before release and
never reattached afterwards, and a terminal fault->damp transition. Every role/support
change is journaled with its physics sequence. A refusal raises GuardRefused; the probe
must abort the run, not continue with the previous command.
"""
import json
import math


class GuardRefused(RuntimeError):
    pass


def _finite(v):
    return type(v) in (int, float) and math.isfinite(v)


class RuntimeOwnershipGuard:
    def __init__(self, *, body_names, hand_names, limits, physics_dt, decimation, hand_margin_rad,
                 max_target_step_rad=0.5, journal=None):
        body = tuple(body_names)
        hands = tuple(hand_names)
        if len(body) != 29 or len(set(body)) != 29:
            raise GuardRefused('exactly 29 unique body joint names required')
        if set(body) & set(hands):
            raise GuardRefused('body and hand joint sets overlap')
        fingers = [n for n in body if any('_' + f + '_' in n for f in ('thumb', 'index', 'middle', 'ring', 'little'))]
        if fingers:
            raise GuardRefused('finger joints in the body set: %s' % fingers[:3])
        if set(body) | set(hands) != set(limits):
            raise GuardRefused('limits must cover exactly the body and independent hand joints')
        if not (_finite(physics_dt) and 0 < physics_dt <= 0.05) or type(decimation) is not int or decimation < 1:
            raise GuardRefused('physics_dt/decimation invalid')
        if not (_finite(hand_margin_rad) and 0 < hand_margin_rad <= 0.1):
            raise GuardRefused('hand margin must be in (0, 0.1]')
        self.body, self.hands, self.limits = body, hands, {n: dict(l) for n, l in limits.items()}
        self.dt, self.decimation, self.margin = float(physics_dt), decimation, float(hand_margin_rad)
        self.max_step = float(max_target_step_rad)
        self.body_owner = None
        self.hand_owner = None
        self.support = 'NONE'
        self.release_sequence = None
        self.state = 'idle'            # idle | supported_settle | unsupported | fault_damp
        self.fault_reason = None
        self.sequence = None
        self.physics_s = None
        self.last_state_sequence = None
        self.last_targets = None
        self.journal = journal
        self.entries = []

    # ------------------------------------------------------------------ journal
    def _log(self, kind, **detail):
        entry = {'sequence': self.sequence, 'physics_s': self.physics_s, 'kind': kind, **detail}
        self.entries.append(entry)
        if self.journal is not None:
            self.journal.write(json.dumps(entry) + '\n')

    def _refuse(self, reason):
        self._log('refused', reason=reason)
        raise GuardRefused(reason)

    # ------------------------------------------------------------------ clocks
    def begin_step(self, sequence, physics_s):
        if self.state == 'fault_damp':
            self._refuse('faulted run: no further steps are admitted')
        if type(sequence) is not int or sequence < 0:
            self._refuse('sequence must be a nonnegative integer')
        if not _finite(physics_s):
            self._refuse('physics time must be finite')
        if self.sequence is not None:
            if sequence != self.sequence + 1:
                self._refuse('non-contiguous sequence %d after %d' % (sequence, self.sequence))
            if abs((physics_s - self.physics_s) - self.dt) > 1e-6:
                self._refuse('physics advanced by %.6f s, admitted dt %.6f s' % (physics_s - self.physics_s, self.dt))
            if self.last_state_sequence != self.sequence:
                self._refuse('no state was observed for step %d before starting step %d' % (self.sequence, sequence))
        self.sequence, self.physics_s = sequence, physics_s

    def observe_state(self, names, q, dq):
        names = tuple(names)
        if set(names) != set(self.limits) or len(q) != len(names) or len(dq) != len(names):
            self._refuse('observed state name set / width mismatch')
        if not all(_finite(v) for v in q) or not all(_finite(v) for v in dq):
            self._refuse('non-finite observed state')
        self.last_state_sequence = self.sequence

    # ------------------------------------------------------------------ ownership
    def claim_body(self, owner):
        """One owner at a time. A hand-over is legal only while supported (settle) or idle;
        while unsupported the owner is frozen (fault -> damp is the only exit)."""
        if self.state == 'fault_damp':
            self._refuse('faulted: only damp holds the body')
        if self.body_owner is not None and self.body_owner != owner:
            if self.state == 'unsupported':
                self._refuse('command-source switching while unsupported is refused')
            if self.state not in ('idle', 'supported_settle'):
                self._refuse('body already owned by %r; %r is a duplicate owner' % (self.body_owner, owner))
            self._log('body_owner_handover', previous=self.body_owner, owner=owner)
        self.body_owner = owner
        self._log('body_owner', owner=owner)

    def claim_hands(self, owner):
        if self.hand_owner is not None and self.hand_owner != owner:
            self._refuse('hands already owned by %r' % self.hand_owner)
        self.hand_owner = owner
        self._log('hand_owner', owner=owner)

    def declare_support(self, kind):
        if kind not in ('FIXED_PELVIS', 'RIG_WRENCH'):
            self._refuse('support kind must be FIXED_PELVIS or RIG_WRENCH')
        if self.release_sequence is not None:
            self._refuse('support declared after the recorded release (hidden support)')
        self.support, self.state = kind, 'supported_settle'
        self._log('support', support_kind=kind)

    def release_support(self):
        if self.state != 'supported_settle':
            self._refuse('release requires an active supported settle')
        if self.body_owner is None:
            self._refuse('release without a body owner')
        self.support, self.state, self.release_sequence = 'NONE', 'unsupported', self.sequence
        self._log('support_release')

    def assert_support_row(self, support):
        """Called with the support record the probe is about to apply this step."""
        kind = support.get('kind')
        if self.state == 'unsupported':
            if kind != 'NONE' or any(v != 0. for v in support.get('force_n', [1])) or any(v != 0. for v in support.get('torque_nm', [1])):
                self.fault('support reattached after release')
                raise GuardRefused('support reattached after release')
        elif self.state == 'supported_settle':
            if kind != self.support:
                self._refuse('support row %r differs from the declared %r' % (kind, self.support))
            if not all(_finite(v) for v in support.get('force_n', [])) or not all(_finite(v) for v in support.get('torque_nm', [])):
                self._refuse('non-finite support wrench')

    # ------------------------------------------------------------------ commands
    def admit_body_command(self, owner, names, targets):
        if self.state == 'fault_damp':
            self._refuse('faulted: body commands are refused')
        if owner != self.body_owner:
            self._refuse('body command from %r but owner is %r' % (owner, self.body_owner))
        if tuple(names) != self.body:
            self._refuse('body command names must be exactly the 29 body joints in the declared order')
        if len(targets) != 29 or not all(_finite(v) for v in targets):
            self._refuse('body targets must be 29 finite values')
        for n, v in zip(self.body, targets):
            lo, hi = self.limits[n]['lower'], self.limits[n]['upper']
            if not lo <= v <= hi:
                self._refuse('target %s=%.4f outside [%.4f, %.4f]' % (n, v, lo, hi))
        if self.last_targets is not None:
            worst = max(abs(a - b) for a, b in zip(targets, self.last_targets))
            if worst > self.max_step:
                self._refuse('target step %.3f rad exceeds the admitted per-step bound %.3f' % (worst, self.max_step))
        if self.last_state_sequence is None and self.sequence is not None and self.sequence > 0:
            self._refuse('no fresh state observed before commanding')
        self.last_targets = list(targets)

    def admit_hand_command(self, owner, names, values):
        if tuple(names) != self.hands:
            self._refuse('hand command names must be exactly the independent hand joints')
        if self.hands and owner != self.hand_owner:
            self._refuse('hand command from %r but owner is %r' % (owner, self.hand_owner))
        if len(values) != len(self.hands) or not all(_finite(v) for v in values):
            self._refuse('hand values must be finite and complete')
        for n, v in zip(self.hands, values):
            if v < self.margin - 1e-9:
                self._refuse('hand target %s=%.4f below the declared margin %.3f (exact-open is a separate diagnostic)' % (n, v, self.margin))
            if v > self.limits[n]['upper']:
                self._refuse('hand target %s above its limit' % n)

    def fault(self, reason):
        self.fault_reason = reason
        self.state = 'fault_damp'
        self.body_owner = 'damp'
        self._log('fault_damp', reason=reason)

    def summary(self):
        return {'state': self.state, 'body_owner': self.body_owner, 'hand_owner': self.hand_owner, 'support': self.support,
                'release_sequence': self.release_sequence, 'fault_reason': self.fault_reason, 'journal_entries': len(self.entries)}
