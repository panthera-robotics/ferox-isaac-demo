"""Closed-loop target advancement for the command-replay probe (sprint L C1).

Pure Python, no simulator. One ``ClosedLoopTargets`` object owns the arm and hand targets that the probe applies while
a model chunk is being consumed:

* ``begin_row`` — called once per model row (every ``model_ticks`` physics ticks): rate-limits the requested arm targets
  against the previous applied arm targets (each limited axis is an intervention), sets the arm ramp for this row, and —
  independently of the arm interpolation — applies the model's hand targets when the hand owner is ``MODEL``. With the
  hand owner ``SCRIPTED`` (hybrid route) the raw model hand values are recorded and never applied. It returns exactly one
  ``model_chunk`` log record (requested arm targets, requested hand joint targets, ownership, ids).
* ``advance`` — called every physics tick: computes the arm targets for this tick (linear interpolation over the
  ``model_ticks`` ticks of the row when enabled, else the stepped target) on the declared interpolation axes only, and
  returns the applied arm+hand targets for the per-tick ``applied`` log record. Hand targets are never touched here.
* ``hold`` — tail hold: keeps the last applied targets (no new row).

Sprint K's probe nested the non-hybrid hand update and the model_chunk log under the *interpolation-disabled* branch, so
with the default ``interpolate_within_step=True`` model hand targets were never applied and no model_chunk rows were logged.
"""
import math


class TargetError(ValueError):
    pass


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class ClosedLoopTargets:
    def __init__(self, *, arm_names, hand_names, initial_arm, initial_hand, model_ticks, max_step_rad=None,
                 interpolate=True, hand_owner='MODEL', hand_decoder=None, interpolation_axes=None):
        if hand_owner not in ('MODEL', 'SCRIPTED'):
            raise TargetError('hand_owner must be MODEL or SCRIPTED')
        if int(model_ticks) < 1:
            raise TargetError('model_ticks must be >= 1')
        if hand_owner == 'MODEL' and hand_decoder is None:
            raise TargetError('hand_owner MODEL needs a hand_decoder(side, values) -> {joint: rad}')
        self.arm_names = list(arm_names); self.hand_names = list(hand_names)
        if len(set(self.arm_names)) != len(self.arm_names) or len(set(self.hand_names)) != len(self.hand_names):
            raise TargetError('duplicate joint names')
        self.model_ticks = int(model_ticks); self.max_step = None if max_step_rad is None else float(max_step_rad)
        self.interpolate = bool(interpolate); self.hand_owner = hand_owner; self.hand_decoder = hand_decoder
        self.interpolation_axes = set(self.arm_names if interpolation_axes is None else interpolation_axes)
        unknown = self.interpolation_axes - set(self.arm_names)
        if unknown:
            raise TargetError('interpolation axes are not arm axes: %s' % sorted(unknown))
        self.arm = {n: float(initial_arm[n]) for n in self.arm_names}
        self.hand = {n: float(initial_hand[n]) for n in self.hand_names}
        self.ramp_from = dict(self.arm); self.ramp_to = dict(self.arm); self.ramp_tick0 = None
        self.rows = 0; self.interventions = []; self.applied_records = 0; self.chunk_records = 0
        self.model_hand_raw_last = None

    # ------------------------------------------------------------------ per model row
    def begin_row(self, *, tick, physics_s, body_q_rad, hands, iteration, chunk_pos, obs_id=None):
        """Consume one decoded model row. Returns the single model_chunk log record for this row."""
        if not isinstance(body_q_rad, dict):
            raise TargetError('body_q_rad must be a dict')
        requested = {}
        for n, v in body_q_rad.items():
            if n not in self.arm:
                continue          # waist/leg outputs are not model-owned in this route
            if not _finite(v):
                raise TargetError('non-finite arm target for %s' % n)
            requested[n] = float(v)
        missing = [n for n in self.arm_names if n not in requested]
        if missing:
            raise TargetError('model row misses arm targets: %s' % missing)
        self.ramp_from = dict(self.arm); self.ramp_to = dict(self.arm); self.ramp_tick0 = int(tick)
        for n, v in requested.items():
            prev = self.arm[n]
            v_lim = v if self.max_step is None else min(max(v, prev - self.max_step), prev + self.max_step)
            if abs(v_lim - v) > 1e-9:
                self.interventions.append({'tick': int(tick), 'physics_s': physics_s, 'axis': n, 'kind': 'rate_limit',
                                           'requested_rad': v, 'applied_rad': v_lim, 'max_step_rad': self.max_step})
            self.ramp_to[n] = v_lim
        hand_requested = None
        if hands is not None and not isinstance(hands, dict):
            raise TargetError('hands must be a dict side -> values')
        self.model_hand_raw_last = hands
        if self.hand_owner == 'MODEL':
            if not hands:
                raise TargetError('hand owner is MODEL but the row carries no hand values')
            hand_requested = {}; clipped = {}
            for side, vals in hands.items():
                decoded = self.hand_decoder(side, vals)
                # a decoder may return {joint: rad} or ({joint: rad}, info) where info['clipped_axes'] lists declared clips
                targets, info = (decoded if isinstance(decoded, tuple) else (decoded, {}))
                clipped[side] = list((info or {}).get('clipped_axes', []))
                for n, v in targets.items():
                    if n not in self.hand:
                        raise TargetError('decoded hand joint %s is not a hand target' % n)
                    if not _finite(v):
                        raise TargetError('non-finite hand target for %s' % n)
                    hand_requested[n] = float(v)
            # applied at the model-row cadence, independent of the arm interpolation
            for n, v in hand_requested.items():
                self.hand[n] = v
        else:
            clipped = None
        self.rows += 1; self.chunk_records += 1
        return {'sequence': int(tick), 'physics_s': physics_s, 'source': 'model_chunk', 'iteration': iteration, 'chunk_pos': chunk_pos,
                'obs_id': obs_id, 'body_targets_rad': requested, 'arm_targets_rate_limited_rad': dict(self.ramp_to),
                'hand_owner': self.hand_owner, 'hand_targets_rad': hand_requested, 'hand_clipped_axes': clipped,
                'model_hand_raw': hands}   # the requested raw vector is recorded for both owners (H3: requested, mapped, applied)

    # ------------------------------------------------------------------ per physics tick
    def advance(self, *, tick, physics_s):
        """Arm targets for this tick (interpolated on the declared axes, stepped elsewhere); hands untouched. Returns the applied record."""
        if self.ramp_tick0 is not None:
            a = min(1.0, (int(tick) - self.ramp_tick0 + 1) / self.model_ticks)
            for n in self.arm_names:
                if self.interpolate and n in self.interpolation_axes:
                    self.arm[n] = (1.0 - a) * self.ramp_from[n] + a * self.ramp_to[n]
                else:
                    self.arm[n] = self.ramp_to[n]
        self.applied_records += 1
        return {'sequence': int(tick), 'physics_s': physics_s, 'source': 'applied', 'arm_targets_rad': dict(self.arm), 'hand_targets_rad': dict(self.hand),
                'hand_owner': self.hand_owner, 'row_index': self.rows}

    def hold(self, *, tick, physics_s):
        """Tail hold: no new row; targets stay where they are (arm at the end of the last ramp)."""
        for n in self.arm_names:
            self.arm[n] = self.ramp_to[n]
        return self.advance(tick=tick, physics_s=physics_s)

    def set_scripted_hand(self, targets):
        """Scripted hand owner writes hand targets here (hybrid route); refused when the model owns the hands."""
        if self.hand_owner != 'SCRIPTED':
            raise TargetError('hand targets are model-owned; a scripted write would silently overwrite them')
        for n, v in targets.items():
            if n not in self.hand:
                raise TargetError('unknown hand joint %s' % n)
            if not _finite(v):
                raise TargetError('non-finite scripted hand target for %s' % n)
            self.hand[n] = float(v)

    def arm_vector(self, names):
        return [self.arm[n] if n in self.arm else None for n in names]

    def hand_vector(self, names):
        return [self.hand[n] for n in names]
