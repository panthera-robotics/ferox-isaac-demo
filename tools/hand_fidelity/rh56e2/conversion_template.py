#!/usr/bin/env python3
"""Native-to-radian conversion map for the installed RH56E2, built ONLY from filled sheets; refuses to emit a map while any
required field is null and lists what is missing.

Per axis, from actuator_endpoints_sheet.json: native readback at command 1000 (open) and 0 (closed), direction, and the
measured physical angle at both endpoints (alpha for fingers, theta for thumb bend, beta for thumb rotation; manual Figure 5).
Map: closure c = (n_open - n) / (n_open - n_closed) in [0, 1] from native counts n; physical angle = open + c * (closed - open)
in the Figure 5 definition; donor joint radian = c * donor_closed_rad (closure-preserving import: closed maps to closed;
the donor datum, URDF zero = open, is NOT the E2 metacarpal-plane datum, so radian identity is never assumed). Each axis carries
its own affine map; the thumb rotation scale is never assumed equal to the fingers'. Output: native_to_radian_map.json plus a
six-axis convention block in the command_semantics format (inspire_e2_installed_<side>_v1).
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys

from .register_map import MANUAL, NATIVE_ORDER

DONOR_CLOSED_RAD = {'index': 1.4381, 'middle': 1.4381, 'ring': 1.4381, 'little': 1.4381, 'thumb_bend': 0.5864, 'thumb_rotation': 1.1641}   # FTP donor URDF limits (command_semantics.FTP_DONOR_LIMITS)
REQUIRED = ('angle_actual_at_command_1000', 'angle_actual_at_command_0', 'direction', 'physical_open_angle_deg', 'physical_closed_angle_deg', 'readback_units', 'command_units')


def build(sheet, geometry=None):
    axes = sheet.get('axes') or {}; missing = []; out_axes = {}
    if sheet.get('evidence_class') != 'installed_measurement': missing.append('sheet.evidence_class must be installed_measurement')
    if not sheet.get('measured_utc') or not sheet.get('operator'): missing.append('sheet.measured_utc / operator')
    for ax in NATIVE_ORDER:
        a = axes.get(ax) or {}
        for k in REQUIRED:
            if a.get(k) in (None, '', []): missing.append('%s.%s' % (ax, k))
        if not a.get('log_files'): missing.append('%s.log_files (evidence)' % ax)
        if missing and any(m.startswith(ax + '.') for m in missing): continue
        n_open, n_closed = float(a['angle_actual_at_command_1000']), float(a['angle_actual_at_command_0'])
        if n_open == n_closed: missing.append('%s: open and closed readbacks are equal (no travel)' % ax); continue
        p_open, p_closed = float(a['physical_open_angle_deg']), float(a['physical_closed_angle_deg'])
        out_axes[ax] = {'native_readback_open': n_open, 'native_readback_closed': n_closed, 'direction': a['direction'], 'readback_units': a['readback_units'], 'command_units': a['command_units'],
                        'physical_angle_definition': a.get('physical_angle_definition'), 'physical_open_deg': p_open, 'physical_closed_deg': p_closed,
                        'closure_from_native': 'c = (%.1f - n) / (%.1f)' % (n_open, n_open - n_closed),
                        'physical_deg_from_closure': 'deg = %.2f + c * (%.2f)' % (p_open, p_closed - p_open),
                        'donor_rad_from_closure': 'q = c * %.4f (closure-preserving; donor URDF zero = open)' % DONOR_CLOSED_RAD[ax],
                        'affine_native_to_donor_rad': {'slope_rad_per_count': -DONOR_CLOSED_RAD[ax] / (n_open - n_closed), 'intercept_rad': DONOR_CLOSED_RAD[ax] * n_open / (n_open - n_closed)},
                        'affine_native_to_physical_deg': {'slope_deg_per_count': -(p_closed - p_open) / (n_open - n_closed), 'intercept_deg': p_open + (p_closed - p_open) * n_open / (n_open - n_closed)},
                        'deadband_counts': a.get('deadband_counts'), 't90_full_stroke_s': a.get('t90_full_stroke_s'), 'evidence': a.get('log_files')}
    if missing:
        return {'status': 'INCOMPLETE', 'label': 'RH56E2_NATIVE_TO_RADIAN_MAP_NOT_ESTABLISHED', 'missing': missing, 'axes_complete': sorted(out_axes)}
    conv = {'name': 'inspire_e2_installed_%s_v1' % sheet['side'], 'axis_order': list(NATIVE_ORDER), 'open_value': 1000, 'closed_value': 0,
            'per_axis_endpoints': {ax: [out_axes[ax]['native_readback_open'], out_axes[ax]['native_readback_closed']] for ax in NATIVE_ORDER},
            'source': 'installed measurement %s %s, operator %s, %s' % (sheet['hardware_model'], sheet['side'], sheet['operator'], sheet['measured_utc']), 'evidence_class_per_axis': {ax: 'MEASURED' for ax in NATIVE_ORDER}}
    return {'status': 'COMPLETE', 'label': 'RH56E2_NATIVE_TO_RADIAN_MAP_ESTABLISHED', 'generated_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'), 'side': sheet['side'], 'hardware_model': sheet['hardware_model'],
            'manual': MANUAL, 'donor_closed_rad': DONOR_CLOSED_RAD, 'import_policy': 'closure_preserving', 'axes': out_axes, 'six_axis_convention': conv}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter); p.add_argument('actuator_endpoints_sheet'); p.add_argument('--out'); a = p.parse_args(argv)
    res = build(json.load(open(a.actuator_endpoints_sheet))); s = json.dumps(res, indent=1)
    if a.out: open(a.out, 'w').write(s)
    print(s); return 0 if res['status'] == 'COMPLETE' else 2


if __name__ == '__main__':
    sys.exit(main())
