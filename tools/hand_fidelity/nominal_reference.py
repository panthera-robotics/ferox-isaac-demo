"""Manufacturer NOMINAL reference values for the Inspire RH56E2 with their source, page and angle definition, and the
datum-reconciled comparison against a donor profile. Nominal values are specifications, not installed calibration.

Angle definitions (RH56E2 user manual V1.0.0, PRJ-01-TS-U-011, Figure 5, pp. 25-26):
  alpha (little/ring/middle/index): angle between the proximal phalanx and the metacarpal plane, 170 deg open -> 91.5 deg closed;
  theta (thumb bending): angle between the thumb distal segment and a line parallel to the thumb rotation axis, 28.5 -> 64 deg;
  beta (thumb rotation): angle between the thumb line and the metacarpal plane in the plane normal to the rotation axis,
        170 deg open -> 75 deg closed.
The donor proxy for the metacarpal plane is the hand root x-z plane (the palm faces and the four MCP axes are parallel
to it); the donor thumb line is thumb_1 origin -> thumb_4 mesh tip. Both proxies carry a declared +-5 deg datum uncertainty.
"""
from __future__ import annotations

import math

import numpy as np

E2_MANUAL = {'id': 'e2_manual', 'title': 'Dexterous Hands User Manual for RH56E2 V1.0.0 (June 2025), Beijing Inspire-Robots, PRJ-01-TS-U-011',
             'url': 'https://en.inspire-robots.com/wp-content/uploads/2026/09/Dexterous-Hands-User-Manual-for-RH56E2_V1.0.0.pdf',
             'sha256': 'add7aab76578cf08275529431b8b428d2d7facff626fe3a0bb2edbd1978983cd', 'applies_to': 'RH56E2 series (0/2, L/R, T1/T2); side and revision not distinguished'}
E2_PRODUCT_PAGE = {'id': 'e2_product_page', 'title': 'RH56E2 product page, specifications table (T1 column)', 'url': 'https://en.inspire-robots.com/product/rh56e2/',
                   'sha256': 'db12bca328ec2e2f7957646006d08064d71dc6f48498627300840b9d4aa6abd6', 'applies_to': 'RH56E2-2R-T1 listed; specifications, not fitted-assembly measurements'}

NOMINAL = {
    'finger_alpha_open_deg': {'value': 170.0, 'source': 'e2_manual', 'where': 'Figure 5 p.25 (little/ring/middle/index 91.5-170 deg)', 'unit': 'deg'},
    'finger_alpha_closed_deg': {'value': 91.5, 'source': 'e2_manual', 'where': 'Figure 5 p.25', 'unit': 'deg'},
    'thumb_theta_open_deg': {'value': 28.5, 'source': 'e2_manual', 'where': 'Figure 5 p.25 (bending angle 28.5-64 deg)', 'unit': 'deg'},
    'thumb_theta_closed_deg': {'value': 64.0, 'source': 'e2_manual', 'where': 'Figure 5 p.25', 'unit': 'deg'},
    'thumb_beta_open_deg': {'value': 170.0, 'source': 'e2_manual', 'where': 'Figure 5 p.26 (rotation angle 75-170 deg)', 'unit': 'deg'},
    'thumb_beta_closed_deg': {'value': 75.0, 'source': 'e2_manual', 'where': 'Figure 5 p.26', 'unit': 'deg'},
    'commanded_axes': {'value': 6, 'source': 'e2_manual', 'where': 'Table 45 (six angle registers 1486-1497)', 'unit': 'count'},
    'angle_register_open_counts': {'value': 1000, 'source': 'e2_manual', 'where': '2.6.12 example: actual angle 1000 = fully open; set 0 -> bends toward the palm', 'unit': 'counts'},
    'hand_mass_kg': {'value': 0.79, 'tolerance': 0.01, 'source': 'e2_product_page', 'where': 'specifications table', 'unit': 'kg', 'inclusions': 'UNSPECIFIED (hand only? cabling? wrist module?)'},
}


def donor_angles(hand):
    """Datum-reconciled donor angles (deg) from a HandUrdf: alpha for each finger at open/closed, beta for the thumb
    rotation at open/closed (tip line and joint-origin line), theta travel."""
    out = {'metacarpal_plane_proxy': 'root x-z plane (palm faces and MCP axes parallel to it)', 'datum_uncertainty_deg': 5.0}
    fr0 = hand.frames({})
    for f in ('index', 'middle', 'ring', 'little'):
        jn = hand.actuators[f]; lo, hi = hand.joints[jn]['limit']
        vals = {}
        for label, q in (('open', lo), ('closed', hi)):
            fr = hand.frames({jn: q}); d = fr[hand.side + '_' + f + '_2'][:3, 3] - fr[hand.side + '_' + f + '_1'][:3, 3]
            elev = math.degrees(math.atan2(d[1], math.hypot(d[0], d[2])))
            vals['alpha_%s_deg' % label] = round(180.0 - elev, 3)
        out['finger_' + f] = vals
    jn = hand.actuators['thumb_rotation']; lo, hi = hand.joints[jn]['limit']
    have_mesh = hand.meshes_available()
    beta = {}
    for label, q in (('open', lo), ('closed', hi)):
        fr = hand.frames({jn: q}); base = fr[hand.side + '_thumb_1'][:3, 3]
        lines = {'joint_origin_line': fr[hand.side + '_thumb_4'][:3, 3]}
        if have_mesh:
            v = hand.link_vertices(hand.side + '_thumb_4'); T = fr[hand.side + '_thumb_4']; w = v @ T[:3, :3].T + T[:3, 3]
            lines['tip_line'] = w[np.argmax(np.linalg.norm(w - T[:3, 3], axis=1))]
        for lname, pt in lines.items():
            d = pt - base
            # medial direction = +x for the right hand, -x for the left (toward the little finger); beta measured from it in the x-y plane
            medial = 1.0 if hand.side == 'right' else -1.0
            beta['beta_%s_%s_deg' % (label, lname)] = round(math.degrees(math.atan2(d[1], medial * d[0])), 3)
    beta['rotation_travel_deg'] = round(math.degrees(hi - lo), 3)
    out['thumb_rotation'] = beta
    jb = hand.actuators['thumb_bend']; lo_b, hi_b = hand.joints[jb]['limit']
    out['thumb_bend'] = {'theta_travel_deg': round(math.degrees(hi_b - lo_b), 3), 'theta_datum': 'NOT_RECONCILED (the E2 theta reference line, parallel to the rotation axis, has no donor equivalent measured here)'}
    return out


def compare(hand, mass_kg):
    """Donor vs nominal rows with explicit datum status; never a similarity percentage."""
    d = donor_angles(hand)
    rows = []

    def row(prop, donor, nominal, unit, datum, note=''):
        diff = None if donor is None or nominal is None else round(donor - nominal, 3)
        rows.append({'property': prop, 'donor': donor, 'nominal': nominal, 'unit': unit, 'donor_minus_nominal': diff, 'datum_status': datum, 'note': note})

    fa = d['finger_index']
    row('finger alpha open', fa['alpha_open_deg'], NOMINAL['finger_alpha_open_deg']['value'], 'deg', 'RECONCILED_PROXY (+-5 deg)', 'donor opens further than nominal')
    row('finger alpha closed', fa['alpha_closed_deg'], NOMINAL['finger_alpha_closed_deg']['value'], 'deg', 'RECONCILED_PROXY (+-5 deg)', 'closed ends agree within the datum uncertainty')
    row('finger flexion travel', round(fa['alpha_open_deg'] - fa['alpha_closed_deg'], 3), NOMINAL['finger_alpha_open_deg']['value'] - NOMINAL['finger_alpha_closed_deg']['value'], 'deg', 'TRAVEL_ONLY', '')
    tb = d['thumb_rotation']
    key_open = 'beta_open_tip_line_deg' if 'beta_open_tip_line_deg' in tb else 'beta_open_joint_origin_line_deg'
    key_closed = key_open.replace('open', 'closed')
    row('thumb beta open', tb[key_open], NOMINAL['thumb_beta_open_deg']['value'], 'deg', 'RECONCILED_PROXY (+-5 deg, %s)' % key_open.replace('beta_open_', '').replace('_deg', ''), '')
    row('thumb beta closed (max opposition)', tb[key_closed], NOMINAL['thumb_beta_closed_deg']['value'], 'deg', 'RECONCILED_PROXY (+-5 deg)', 'donor stops short of the nominal maximum opposition')
    row('thumb rotation travel', tb['rotation_travel_deg'], NOMINAL['thumb_beta_open_deg']['value'] - NOMINAL['thumb_beta_closed_deg']['value'], 'deg', 'TRAVEL_ONLY', '')
    row('thumb bend travel', d['thumb_bend']['theta_travel_deg'], NOMINAL['thumb_theta_closed_deg']['value'] - NOMINAL['thumb_theta_open_deg']['value'], 'deg', 'TRAVEL_ONLY (theta datum not reconciled)', '')
    row('hand mass', round(mass_kg, 4), NOMINAL['hand_mass_kg']['value'], 'kg', 'INCLUSIONS_UNSPECIFIED', 'nominal +-0.01 kg; donor = URDF hand subtree; neither states cabling/adapter inclusion')
    row('commanded axes', len(hand.independent), NOMINAL['commanded_axes']['value'], 'count', 'EXACT', '')
    return {'sources': [E2_MANUAL, E2_PRODUCT_PAGE], 'donor_angles': d, 'rows': rows, 'installed_similarity_percent': None,
            'rule': 'nominal values are specifications; installed angles/mass are NOT_MEASURED; no row is an acceptance gate'}
