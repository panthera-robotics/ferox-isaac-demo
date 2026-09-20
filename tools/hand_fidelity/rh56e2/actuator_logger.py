#!/usr/bin/env python3
"""Actuator command/readback logger for the installed RH56E2 (six axes, native counts).

Reads angle_actual (1546), actuator_position_actual (1534), force_actual (1582), actuator_current (1594),
actuator_temperature (1618), actuator_error_code (1606) and dof_status (1612) at --rate-hz for --duration-s, and, ONLY with
--allow-writes and a plan file, sends angle_set steps from the plan at their times. The default transport is dry_run:
no port or socket is opened, every read is logged with raw_hex null, and plan steps are logged as planned-not-sent.

Plan file (JSON): {"steps": [{"t_s": 0.0, "angle_set": {"index": 1000}}, {"t_s": 2.0, "angle_set": {"index": 0}}, ...]};
axes omitted from a step are sent as -1 (no action, Table 45). Speeds/force thresholds are never written by this tool.
Output: JSONL per logging_common.Log; summarise with analyze_actuator_log.py.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

from .logging_common import Log, add_common_args, header_for, open_from_args, read_group, write_group
from .register_map import NATIVE_ORDER

READ_GROUPS = ('angle_actual', 'actuator_position_actual', 'force_actual', 'actuator_current', 'actuator_temperature', 'actuator_error_code', 'dof_status')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter); add_common_args(p)
    p.add_argument('--plan', help='JSON step plan (angle_set only); requires --allow-writes to be sent')
    p.add_argument('--groups', default=','.join(READ_GROUPS))
    a = p.parse_args(argv)
    plan = json.load(open(a.plan))['steps'] if a.plan else []
    for s in plan:
        if set(s) - {'t_s', 'angle_set', 'note'} or not isinstance(s.get('angle_set'), dict) or set(s['angle_set']) - set(NATIVE_ORDER):
            raise SystemExit('plan step %r: only t_s, angle_set{axis: -1|0..1000}, note are accepted' % s)
        if any(not (v == -1 or 0 <= int(v) <= 1000) for v in s['angle_set'].values()):
            raise SystemExit('plan step %r: angle_set values must be -1 or 0..1000' % s)
    auth, t = open_from_args(a)
    log = Log(a.out, {**header_for(a, 'actuator_command_readback'), 'plan_file': a.plan, 'plan_steps': len(plan), 'read_groups': a.groups.split(',')})
    groups = a.groups.split(','); period = 1.0 / a.rate_hz; t0 = time.monotonic(); i = 0; pending = sorted(plan, key=lambda s: s['t_s'])
    try:
        while True:
            now = time.monotonic() - t0
            if now > a.duration_s: break
            while pending and pending[0]['t_s'] <= now:
                s = pending.pop(0); values = {ax: int(s['angle_set'].get(ax, -1)) for ax in NATIVE_ORDER}
                if a.allow_writes and t.kind != 'dry_run':
                    write_group(t, log, auth, 'angle_set', values)
                else:
                    log.emit({'kind': 'command_planned_not_sent', 'group': 'angle_set', 'values': values, 't_s': s['t_s'], 'reason': 'dry_run' if t.kind == 'dry_run' else 'writes not allowed'})
            for g in groups:
                read_group(t, log, g)
            i += 1; nxt = t0 + i * period; dt = nxt - time.monotonic()
            if dt > 0: time.sleep(dt)
    finally:
        log.close(); t.close()
    print('wrote', log.path, 'records', log.n, '| dry_run' if t.kind == 'dry_run' else '| transport ' + t.kind)


if __name__ == '__main__':
    sys.exit(main())
