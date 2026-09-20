#!/usr/bin/env python3
"""Donor-vs-installed comparison and the calibration-completeness gate for the RH56E2.

Inputs: the filled sheets directory (sheets.py layout), a donor profile JSON (hand_fidelity.donor_profile output for the same
side) and optionally the campaign fidelity ledger JSON (predeclared tolerances). Output: DONOR_VS_INSTALLED.json (+ .md) with
one row per property: donor value (with the proxy used), nominal (manual/product page where one exists), installed value or
NOT_MEASURED, residual and PASS/FAIL/INDETERMINATE under the ledger's uncertainty-aware rule, and the final label:
RH56E2_INSTALLED_CALIBRATION_COMPLETE only when every required item of the owner's list is MEASURED with evidence; otherwise
RH56E2_INSTALLED_CALIBRATION_INCOMPLETE with the missing list. No similarity percentage is ever computed.
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys

from .conversion_template import build as build_map
from .register_map import NATIVE_ORDER

FINGERS = ('index', 'middle', 'ring', 'little')
TOL_DEFAULT = {'contact_geometry_mm': 3.0, 'link_length_mm': 3.0, 'joint_travel_deg': 5.0, 'mass_g': 50.0, 'timing_ms': 100.0, 'com_mm': 5.0, 'hinge_position_mm': 3.0, 'pad_mm': 2.0}
try:
    from .nominal_reference import NOMINAL
except Exception:  # pragma: no cover
    NOMINAL = {}

REQUIRED_FOR_LABEL = {
    'palm_finger_geometry': ['geometry:palm_thickness_at_pocket_mm', 'geometry:palm_width_mcp_row_mm', 'geometry:flange_to_index_tip_open_mm'] + ['geometry:%s_proximal_length_mm' % f for f in FINGERS] + ['geometry:%s_mcp_to_tip_open_mm' % f for f in FINGERS],
    'hinge_positions': ['geometry:%s_mcp_hinge_centre' % f for f in FINGERS] + ['geometry:thumb_rotation_axis_point', 'geometry:thumb_mp_hinge_centre_open'],
    'pad_dimensions': ['geometry:%s_pad_length_mm' % f for f in FINGERS] + ['geometry:%s_pad_width_mm' % f for f in FINGERS] + ['geometry:thumb_pad_length_mm', 'geometry:thumb_pad_width_mm'],
    'open_closed_endpoints': ['geometry:%s_alpha_open_deg' % f for f in FINGERS] + ['geometry:%s_alpha_closed_deg' % f for f in FINGERS] + ['geometry:thumb_theta_open_deg', 'geometry:thumb_theta_closed_deg', 'geometry:thumb_beta_open_deg', 'geometry:thumb_beta_closed_deg'],
    'total_mass': ['mass_com:mass_as_mounted_total_g'], 'approximate_com': ['mass_com:com_as_mounted_hand_plus_adapter'],
    'actuator_native_min_max': ['actuator:%s.native_min_readback' % ax for ax in NATIVE_ORDER] + ['actuator:%s.native_max_readback' % ax for ax in NATIVE_ORDER],
    'direction': ['actuator:%s.direction' % ax for ax in NATIVE_ORDER], 'command_readback_units': ['actuator:%s.readback_units' % ax for ax in NATIVE_ORDER] + ['actuator:%s.command_units' % ax for ax in NATIVE_ORDER],
    'speed_response': ['actuator:%s.t90_full_stroke_s' % ax for ax in NATIVE_ORDER], 'deadband': ['actuator:%s.deadband_counts' % ax for ax in NATIVE_ORDER],
    'firmware_transport': ['serial:firmware_version', 'serial:transport_in_use'], 'thumb_rotation_direction_scale': ['geometry:thumb_rotation_sense', 'actuator:thumb_rotation.direction', 'actuator:thumb_rotation.angle_actual_at_command_1000', 'actuator:thumb_rotation.angle_actual_at_command_0'],
    'tactile': ['tactile:variant', 'tactile:channel_count_total', 'tactile:arrays', 'tactile:units', 'tactile:rate_hz_achieved'],
}


def _sheet(d, name):
    p = os.path.join(d, name + '_sheet.json'); return json.load(open(p)) if os.path.exists(p) else None


def _lookup(sheets, ref):
    kind, key = ref.split(':', 1); s = sheets.get(kind)
    if not s: return None, None
    if kind == 'actuator':
        ax, field = key.split('.'); return (s.get('axes') or {}).get(ax, {}).get(field), (s.get('axes') or {}).get(ax, {}).get('log_files')
    if kind == 'tactile':
        return (s.get('items') or {}).get(key), s.get('source_evidence', {}).get('files')
    for it in s.get('items', []):
        if it.get('key') == key:
            v = it.get('value') if 'value' in it else it.get('value_xyz'); return v, it.get('evidence') or s.get('source_evidence', {}).get('photos')
    return None, None


def _present(v):
    return v is not None and v != '' and v != [] and not (isinstance(v, list) and any(x is None for x in v))


def _status(resid, u, T):
    if abs(resid) + u <= T: return 'PASS'
    if max(0.0, abs(resid) - u) > T: return 'FAIL'
    return 'INDETERMINATE'


def rows_geometry(sheets, donor, tol):
    g = sheets.get('geometry'); me = (donor or {}).get('mesh_extents_root') or {}; fk = (((donor or {}).get('fk_sweep_root') or {}).get('single_actuator') or {}).get('index', {}).get('closure_0.00', {})
    acts = (donor or {}).get('actuators') or {}; rows = []
    def row(key, donor_val, proxy, tkey, nominal=None, unit=None):
        v, ev = _lookup({'geometry': g} if g else {}, 'geometry:' + key); it = next((i for i in (g or {}).get('items', []) if i.get('key') == key), {})
        r = {'property': key, 'unit': unit or it.get('unit'), 'donor': donor_val, 'donor_proxy': proxy, 'nominal': nominal, 'installed': v if _present(v) else 'NOT_MEASURED', 'uncertainty': it.get('uncertainty'), 'tolerance': tol.get(tkey), 'evidence': ev if _present(v) else None}
        if _present(v) and isinstance(donor_val, (int, float)) and isinstance(v, (int, float)) and tol.get(tkey) is not None:
            r['residual_donor_minus_installed'] = donor_val - v; r['status'] = _status(r['residual_donor_minus_installed'], float(it.get('uncertainty') or 0.0), tol[tkey])
        elif _present(v) and isinstance(v, list) and isinstance(donor_val, list) and len(v) == len(donor_val) == 3:
            d = math.dist(v, donor_val); r['residual_distance'] = d; r['status'] = _status(d, float(it.get('uncertainty') or 0.0), tol[tkey])
        else:
            r['status'] = 'NOT_MEASURED' if not _present(v) else 'NO_DONOR_VALUE'
        rows.append(r)
    m = lambda k: round(me[k] * 1000, 2) if k in me else None
    row('palm_thickness_at_pocket_mm', m('palm_band_thickness_along_flexion_normal_m'), 'donor palm band thickness along the flexion normal (mesh, 0.09-0.15 m band)', 'contact_geometry_mm')
    row('palm_width_mcp_row_mm', m('palm_band_width_along_viewer_right_m'), 'donor palm band width', 'contact_geometry_mm')
    env = me.get('open_envelope_root_max_m'); row('flange_to_index_tip_open_mm', round(env[2] * 1000, 2) if env else None, 'donor open envelope extent along the fingers from the root (flange-side) link', 'link_length_mm')
    for f in FINGERS:
        row('%s_proximal_length_mm' % f, m('%s_proximal_segment_m' % f), 'donor mesh proximal segment', 'link_length_mm'); row('%s_mcp_to_tip_open_mm' % f, m('%s_mcp_to_distal_tip_m' % f), 'donor mesh MCP to distal tip', 'link_length_mm')
        o = fk.get(f, {}).get('mcp_joint_origin'); row('%s_mcp_hinge_centre' % f, [round(x * 1000, 2) for x in o] if o else None, 'donor URDF MCP joint origin in right_base_link (root) frame; installed D0 frame must be reconciled to the root frame before comparing', 'hinge_position_mm')
        o = fk.get(f, {}).get('distal_joint_origin'); row('%s_pip_hinge_centre_open' % f, [round(x * 1000, 2) for x in o] if o else None, 'donor URDF distal joint origin (root frame)', 'hinge_position_mm')
        for pk in ('pad_length_mm', 'pad_width_mm', 'pad_thickness_mm'): row('%s_%s' % (f, pk), None, 'donor: no pad geometry (rigid hulls)', 'pad_mm')
        a = acts.get(f, {}); ao, ac = _lookup({'geometry': g} if g else {}, 'geometry:%s_alpha_open_deg' % f)[0], _lookup({'geometry': g} if g else {}, 'geometry:%s_alpha_closed_deg' % f)[0]
        r = {'property': '%s_alpha_travel_deg' % f, 'unit': 'deg', 'donor': a.get('travel_deg'), 'donor_proxy': 'donor joint travel (URDF limits); datum differs from Figure 5 alpha', 'nominal': (NOMINAL.get('finger_alpha_open_deg', {}).get('value', 0) - NOMINAL.get('finger_alpha_closed_deg', {}).get('value', 0)) or None,
             'installed': (ao - ac) if _present(ao) and _present(ac) else 'NOT_MEASURED', 'tolerance': tol.get('joint_travel_deg')}
        if isinstance(r['installed'], (int, float)) and r['donor'] is not None: r['residual_donor_minus_installed'] = r['donor'] - r['installed']; r['status'] = _status(r['residual_donor_minus_installed'], 2.0 * math.sqrt(2), tol['joint_travel_deg'])
        else: r['status'] = 'NOT_MEASURED'
        rows.append(r)
    th = (donor or {}).get('thumb_rotation_datum') or {}
    for key, dv, proxy, tk in (('thumb_theta_travel_deg', acts.get('thumb_bend', {}).get('travel_deg'), 'donor thumb_2 travel', 'joint_travel_deg'), ('thumb_beta_travel_deg', th.get('travel_deg'), 'donor thumb_1 rotation travel (elevation datum)', 'joint_travel_deg')):
        pre = key.replace('_travel_deg', ''); o, c = _lookup({'geometry': g} if g else {}, 'geometry:%s_open_deg' % pre)[0], _lookup({'geometry': g} if g else {}, 'geometry:%s_closed_deg' % pre)[0]
        r = {'property': key, 'unit': 'deg', 'donor': dv, 'donor_proxy': proxy, 'installed': abs(o - c) if _present(o) and _present(c) else 'NOT_MEASURED', 'tolerance': tol.get(tk)}
        if isinstance(r['installed'], (int, float)) and dv is not None: r['residual_donor_minus_installed'] = dv - r['installed']; r['status'] = _status(r['residual_donor_minus_installed'], 2.0 * math.sqrt(2), tol[tk])
        else: r['status'] = 'NOT_MEASURED'
        rows.append(r)
    for key in ('thumb_rotation_axis_point', 'thumb_mp_hinge_centre_open', 'thumb_ip_hinge_centre_open', 'thumb_pad_length_mm', 'thumb_pad_width_mm', 'thumb_pad_thickness_mm', 'thumb_rotation_sense', 'aperture_thumb_tip_to_index_tip_open_mm', 'flange_to_wrist_yaw_link_translation'):
        dv = None; proxy = 'donor: not extracted here'
        if key == 'flange_to_wrist_yaw_link_translation':
            mt = (donor or {}).get('mount') or {}; dv = [round(x * 1000, 2) for x in mt.get('xyz', [])] or None; proxy = 'donor URDF right_base_joint xyz in right_wrist_yaw_link (source flange, not installed extrinsics)'
        if key == 'thumb_rotation_axis_point':
            o = fk.get('thumb', {}).get('rotation_joint_origin'); dv = [round(x * 1000, 2) for x in o] if o else None; proxy = 'donor thumb_1 joint origin (root frame)'
        row(key, dv, proxy, 'hinge_position_mm' if 'hinge' in key or 'axis' in key or 'translation' in key else 'pad_mm')
    return rows


def rows_mass(sheets, donor, tol):
    mc = sheets.get('mass_com'); mp = (donor or {}).get('mass_properties_open_root') or {}; rows = []
    v, ev = _lookup({'mass_com': mc} if mc else {}, 'mass_com:mass_hand_only_g'); dv = round(mp['mass_kg'] * 1000, 1) if 'mass_kg' in mp else None
    r = {'property': 'mass_hand_only_g', 'unit': 'g', 'donor': dv, 'donor_proxy': 'donor URDF link masses (hand only, root frame)', 'nominal': NOMINAL.get('hand_mass_kg', {}).get('value', 0) * 1000 or None, 'installed': v if _present(v) else 'NOT_MEASURED', 'tolerance': tol['mass_g'], 'evidence': ev if _present(v) else None}
    if _present(v) and dv is not None: r['residual_donor_minus_installed'] = dv - v; r['status'] = _status(r['residual_donor_minus_installed'], 1.0, tol['mass_g'])
    else: r['status'] = 'NOT_MEASURED'
    rows.append(r)
    for key in ('mass_as_mounted_total_g', 'mass_hand_with_cable_g', 'mass_wrist_adapter_g'):
        v, ev = _lookup({'mass_com': mc} if mc else {}, 'mass_com:' + key); rows.append({'property': key, 'unit': 'g', 'donor': None, 'donor_proxy': 'donor has no cable/adapter mass', 'installed': v if _present(v) else 'NOT_MEASURED', 'status': 'NO_DONOR_VALUE' if _present(v) else 'NOT_MEASURED'})
    v, ev = _lookup({'mass_com': mc} if mc else {}, 'mass_com:com_hand_only'); dv = [round(x * 1000, 2) for x in mp['com_m']] if 'com_m' in mp else None
    r = {'property': 'com_hand_only', 'unit': 'mm', 'donor': dv, 'donor_proxy': 'donor COM at the open pose in right_base_link (root) frame; installed D0 frame must be reconciled to the root frame', 'installed': v if _present(v) else 'NOT_MEASURED', 'tolerance': tol['com_mm']}
    if _present(v) and dv: r['residual_distance'] = math.dist(v, dv); r['status'] = _status(r['residual_distance'], 3.0, tol['com_mm'])
    else: r['status'] = 'NOT_MEASURED'
    rows.append(r)
    v, _ = _lookup({'mass_com': mc} if mc else {}, 'mass_com:com_as_mounted_hand_plus_adapter'); rows.append({'property': 'com_as_mounted_hand_plus_adapter', 'unit': 'mm', 'donor': None, 'donor_proxy': 'donor: hand only', 'installed': v if _present(v) else 'NOT_MEASURED', 'status': 'NO_DONOR_VALUE' if _present(v) else 'NOT_MEASURED'})
    return rows


def rows_actuator(sheets, donor):
    a = sheets.get('actuator'); acts = (donor or {}).get('actuators') or {}; rows = []
    for ax in NATIVE_ORDER:
        s = ((a or {}).get('axes') or {}).get(ax, {}); d = acts.get(ax, {})
        rows.append({'property': 'actuator_%s' % ax, 'donor': {'joint': d.get('joint'), 'limit_rad': d.get('limit_rad'), 'velocity_cap_rad_s': d.get('velocity_rad_s'), 'model': 'position-target PD drive, no actuator dynamics'},
                     'installed': {k: (s.get(k) if _present(s.get(k)) else 'NOT_MEASURED') for k in ('native_min_readback', 'native_max_readback', 'direction', 'readback_units', 'command_units', 't90_full_stroke_s', 'deadband_counts', 'angle_actual_at_command_1000', 'angle_actual_at_command_0')},
                     'nominal': {'speed_1000_full_stroke_ms': 600, 'source': 'e2_manual Table 42/47'}, 'status': 'MEASURED' if all(_present(s.get(k)) for k in ('native_min_readback', 'native_max_readback', 'direction')) else 'NOT_MEASURED'})
    m = build_map(a) if a else {'status': 'INCOMPLETE', 'missing': ['actuator_endpoints_sheet.json absent']}
    rows.append({'property': 'native_to_radian_map', 'status': m['status'], 'label': m.get('label'), 'missing': m.get('missing')})
    return rows


def rows_tactile(sheets):
    t = ((sheets.get('tactile') or {}).get('items') or {})
    return [{'property': 'tactile', 'donor': 'none (PhysX contact impulses only)', 'installed': {k: (t.get(k) if _present(t.get(k)) else 'NOT_MEASURED') for k in ('variant', 'channel_count_total', 'units', 'rate_hz_achieved', 'rest_nonzero_channels')}, 'arrays': t.get('arrays') or 'NOT_MEASURED',
             'status': 'MEASURED' if all(_present(t.get(k)) for k in ('variant', 'channel_count_total', 'units', 'rate_hz_achieved')) and t.get('arrays') else 'NOT_MEASURED'}]


def completeness(sheets):
    missing = {}
    for group, refs in REQUIRED_FOR_LABEL.items():
        for ref in refs:
            v, ev = _lookup(sheets, ref)
            if not _present(v): missing.setdefault(group, []).append(ref)
            elif not ev: missing.setdefault(group, []).append(ref + ' (no evidence reference)')
    for kind in ('geometry', 'mass_com', 'serial', 'actuator', 'tactile'):
        s = sheets.get(kind)
        if s is None: missing.setdefault('sheets', []).append(kind + '_sheet.json absent')
        elif s.get('evidence_class') != 'installed_measurement' or not s.get('measured_utc') or not s.get('operator'): missing.setdefault('sheets', []).append(kind + ': evidence_class/measured_utc/operator')
    label = 'RH56E2_INSTALLED_CALIBRATION_COMPLETE' if not missing else 'RH56E2_INSTALLED_CALIBRATION_INCOMPLETE'
    return label, missing


def compare(sheets_dir, donor_profile=None, ledger=None):
    sheets = {'geometry': _sheet(sheets_dir, 'geometry'), 'mass_com': _sheet(sheets_dir, 'mass_com'), 'serial': _sheet(sheets_dir, 'serial_firmware'), 'actuator': _sheet(sheets_dir, 'actuator_endpoints'), 'tactile': _sheet(sheets_dir, 'tactile')}
    donor = json.load(open(donor_profile)) if donor_profile else None
    tol = dict(TOL_DEFAULT)
    if ledger:
        L = json.load(open(ledger)); tol.update({k: v['value'] for k, v in (L.get('predeclared_tolerances') or {}).items()})
    rows = rows_geometry(sheets, donor, tol) + rows_mass(sheets, donor, tol) + rows_actuator(sheets, donor) + rows_tactile(sheets)
    label, missing = completeness(sheets)
    measured = [r for r in rows if r.get('status') in ('PASS', 'FAIL', 'INDETERMINATE')]
    return {'generated_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'), 'sheets_dir': sheets_dir, 'donor_profile': donor_profile, 'donor_urdf_sha256': (donor or {}).get('urdf_sha256'), 'tolerances': tol,
            'decision_rule': 'PASS iff |r|+U <= T; FAIL iff max(0,|r|-U) > T; else INDETERMINATE', 'rows': rows,
            'static_property_agreement': {'numerator_pass': sum(1 for r in measured if r['status'] == 'PASS'), 'denominator_measured': len(measured), 'not_measured': sum(1 for r in rows if r.get('status') == 'NOT_MEASURED')},
            'overall_real_hand_similarity_percent': None, 'label': label, 'missing_for_label': missing}


def to_md(rep):
    out = ['# Donor vs installed RH56E2 — %s' % rep['label'], '', 'Generated %s; donor URDF sha256 %s; no similarity percentage by design.' % (rep['generated_utc'], rep.get('donor_urdf_sha256')), '',
           '| property | donor | nominal | installed | residual | tol | status |', '|---|---|---|---|---|---|---|']
    for r in rep['rows']:
        res = r.get('residual_donor_minus_installed', r.get('residual_distance')); out.append('| %s | %s | %s | %s | %s | %s | %s |' % (r['property'], json.dumps(r.get('donor'), default=str)[:60], r.get('nominal', ''), json.dumps(r.get('installed'), default=str)[:60], ('%.2f' % res) if isinstance(res, (int, float)) else '', r.get('tolerance', ''), r.get('status')))
    out += ['', '## Missing for RH56E2_INSTALLED_CALIBRATION_COMPLETE', '']
    for g, refs in (rep['missing_for_label'] or {}).items(): out.append('- **%s**: %s' % (g, ', '.join(refs)))
    if not rep['missing_for_label']: out.append('- nothing missing')
    return '\n'.join(out) + '\n'


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter); p.add_argument('sheets_dir'); p.add_argument('--donor-profile'); p.add_argument('--ledger'); p.add_argument('--out-prefix', default='DONOR_VS_INSTALLED')
    a = p.parse_args(argv); rep = compare(a.sheets_dir, a.donor_profile, a.ledger)
    open(a.out_prefix + '.json', 'w').write(json.dumps(rep, indent=1, default=str)); open(a.out_prefix + '.md', 'w').write(to_md(rep))
    print(rep['label']); print('rows', len(rep['rows']), 'measured', rep['static_property_agreement']['denominator_measured'], 'not measured', rep['static_property_agreement']['not_measured'])
    for g, refs in (rep['missing_for_label'] or {}).items(): print(' missing', g, len(refs))
    return 0 if rep['label'].endswith('COMPLETE') and not rep['missing_for_label'] else 2


if __name__ == '__main__':
    sys.exit(main())
