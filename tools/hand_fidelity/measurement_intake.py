"""Intake validation for installed-hand measurement files (the sprint G/H `measurement_template.json` schema 2) BEFORE
they are compared with the simulator: evidence class and split, per-measurement finiteness, non-negative uncertainty,
declared units, repeat agreement, command/response vector dimensions, evidence-file hashes and minimum coverage.

This module does not compute residuals or any similarity figure; the comparator (`compare_measurements.py`, sprint G/H)
keeps that role. Synthetic fixtures exercise the importer and are refused as installed evidence unless explicitly allowed,
in which case the report is stamped synthetic.
"""
from __future__ import annotations

import math
import re

UNITS = {'palm_thickness_mm': 'mm', 'palm_width_mm': 'mm', 'flange_to_index_tip_mm': 'mm', 'index_proximal_length_mm': 'mm', 'index_mcp_to_tip_mm': 'mm',
         'little_mcp_to_tip_mm': 'mm', 'finger_flexion_travel_deg': 'deg', 'thumb_bend_travel_deg': 'deg', 'thumb_rotation_travel_deg': 'deg', 'hand_mass_g': 'g',
         'closing_time_ms': 'ms', 'thumb_rotation_open_angle_deg': 'deg', 'thumb_rotation_closed_angle_deg': 'deg', 'finger_open_angle_deg': 'deg', 'finger_closed_angle_deg': 'deg'}
AXES = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')
_SHA = re.compile(r'^[0-9a-f]{64}$')


class IntakeError(ValueError):
    """The file is refused before any comparison."""


def _num(v, what, *, nonneg=False):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise IntakeError('%s must be a finite number (got %r)' % (what, v))
    if nonneg and v < 0:
        raise IntakeError('%s must be non-negative (got %r)' % (what, v))
    return float(v)


def validate_intake(doc, *, allow_synthetic=False, min_measured=1, require_hashes=True):
    if not isinstance(doc, dict):
        raise IntakeError('measurement document must be a JSON object')
    cls = doc.get('evidence_class')
    if cls not in ('installed_measurement', 'synthetic_fixture'):
        raise IntakeError('evidence_class must be installed_measurement or synthetic_fixture')
    if cls == 'synthetic_fixture' and not allow_synthetic:
        raise IntakeError('synthetic fixture refused as installed evidence')
    if doc.get('split', doc.get('purpose')) != 'held_out_validation':
        raise IntakeError('split must be held_out_validation')
    if doc.get('side') not in ('left', 'right'):
        raise IntakeError('side must be left or right')
    if not isinstance(doc.get('hardware_model'), str) or not doc['hardware_model'].startswith('RH56E2-2'):
        raise IntakeError('hardware_model must name the installed RH56E2-2L/2R variant')
    if cls == 'installed_measurement' and (not doc.get('measured_utc') or not doc.get('operator')):
        raise IntakeError('installed measurements need measured_utc and a named operator')
    datums = doc.get('datums')
    if not isinstance(datums, dict) or not datums or not all(isinstance(v, str) and v.strip() for v in datums.values()):
        raise IntakeError('datums must be declared as non-empty strings')
    ev = doc.get('source_evidence') or {}
    files = list(ev.get('files') or []) + list(ev.get('photos') or [])
    if cls == 'installed_measurement' and not files:
        raise IntakeError('installed measurements need at least one evidence file or photo reference')
    hashes = {}
    for f in files:
        if isinstance(f, str):
            if require_hashes:
                raise IntakeError('evidence reference %r has no sha256; list files as {name, sha256}' % f)
            continue
        if not isinstance(f, dict) or not f.get('name') or not _SHA.match(str(f.get('sha256', ''))):
            raise IntakeError('evidence file entries need a name and a 64-hex sha256 (got %r)' % (f,))
        hashes[f['name']] = f['sha256']
    measured, not_measured = [], []
    for m in doc.get('measurements') or []:
        key = m.get('key')
        if key not in UNITS:
            raise IntakeError('unknown measurement key %r' % key)
        if m.get('unit') != UNITS[key]:
            raise IntakeError('unit for %s must be %s (got %r)' % (key, UNITS[key], m.get('unit')))
        u = _num(m.get('uncertainty'), key + '.uncertainty', nonneg=True)
        if m.get('value') is None:
            not_measured.append(key); continue
        v = _num(m['value'], key + '.value')
        reps = m.get('repeats') or []
        if not isinstance(reps, list):
            raise IntakeError('%s.repeats must be a list' % key)
        reps = [_num(r, key + '.repeats[]') for r in reps]
        if reps and abs(sum(reps) / len(reps) - v) > 2 * u + 1e-9:
            raise IntakeError('%s: reported value disagrees with its repeats beyond 2 uncertainties' % key)
        measured.append(key)
    if len(measured) < min_measured:
        raise IntakeError('insufficient coverage: %d measured, %d required' % (len(measured), min_measured))
    cr = doc.get('command_response') or {}
    for row in cr.get('per_axis') or []:
        if row.get('axis') not in AXES:
            raise IntakeError('command_response axis must be one of %s' % (AXES,))
        t, r, s = row.get('target_counts') or [], row.get('readback_counts') or [], row.get('timestamps_s') or []
        if not (len(t) == len(r) == len(s)):
            raise IntakeError('command_response %s: target/readback/timestamp lengths differ (%d/%d/%d)' % (row['axis'], len(t), len(r), len(s)))
        for name, seq, lo, hi in (('target_counts', t, 0, 1000), ('readback_counts', r, 0, 1000), ('timestamps_s', s, 0.0, float('inf'))):
            for x in seq:
                x = _num(x, '%s.%s' % (row['axis'], name))
                if not lo <= x <= hi:
                    raise IntakeError('%s.%s value %r outside [%s, %s]' % (row['axis'], name, x, lo, hi))
        if any(b < a for a, b in zip(s, s[1:])):
            raise IntakeError('command_response %s: timestamps must be non-decreasing' % row['axis'])
    return {'evidence_class': cls, 'synthetic': cls == 'synthetic_fixture', 'side': doc['side'], 'hardware_model': doc['hardware_model'],
            'measured_keys': measured, 'not_measured_keys': not_measured, 'evidence_hashes': hashes,
            'command_response_axes': [r['axis'] for r in (cr.get('per_axis') or [])], 'rule': 'validated for intake only; residuals come from the comparator, never a similarity percentage'}
