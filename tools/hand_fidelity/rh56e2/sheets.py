#!/usr/bin/env python3
"""Blank measurement sheets for the installed RH56E2 (schema rh56e2_sheets_v1). Every value starts as null; a sheet with a
null in a required field never becomes an installed measurement. Serial numbers and photos are PRIVATE: the sheets are
kept outside the public repository once filled. usage: sheets.py --side right --hardware-model RH56E2-2R-T1 --out-dir <dir>"""
from __future__ import annotations

import argparse
import json
import os
import sys

from . import register_map as rm

SCHEMA = 'rh56e2_sheets_v1'
FINGERS = ('index', 'middle', 'ring', 'little')
DATUMS = {'D0_flange_face': 'plane of the wrist-flange mounting face (hand side); origin at the flange centre',
          'D1_palm_centre_line': 'line on the palm face through the middle-finger MCP hinge centre, normal to the MCP hinge row',
          'D2_palm_plane': 'metacarpal plane of manual Figure 5: the plane through the four finger MCP hinge axes',
          'angle_zero': 'fully open = native angle readback 1000 counts unless the installed unit reports otherwise (record the actual readback)'}


def item(key, unit, unc, method, **extra):
    return {'key': key, 'value': None, 'unit': unit, 'uncertainty': unc, 'method': method, 'repeats': [], 'evidence': [], 'notes': None, **extra}


def vec_item(key, unit, unc, method, frame, **extra):
    return {'key': key, 'value_xyz': None, 'unit': unit, 'uncertainty': unc, 'method': method, 'frame': frame, 'repeats': [], 'evidence': [], 'notes': None, **extra}


def base(side, model, sheet):
    return {'schema': SCHEMA, 'sheet': sheet, 'side': side, 'hardware_model': model, 'serial_ref': None, 'measured_utc': None, 'operator': None,
            'evidence_class': 'installed_measurement', 'split': 'held_out_validation', 'datums': dict(DATUMS), 'source_evidence': {'photos': [], 'files': [], 'notes': None},
            'authorization_record': None, 'privacy': 'fill only from the physical unit; keep filled copies private (serials, photos)'}


def geometry_sheet(side, model):
    s = base(side, model, 'geometry'); items = []
    items += [item('palm_thickness_at_pocket_mm', 'mm', 0.1, 'caliper'), item('palm_width_mcp_row_mm', 'mm', 0.1, 'caliper'), item('flange_to_mcp_row_mm', 'mm', 0.5, 'caliper/rule from D0'),
              item('flange_to_index_tip_open_mm', 'mm', 0.5, 'caliper/rule from D0'), item('palm_length_mcp_row_to_wrist_edge_mm', 'mm', 0.5, 'caliper')]
    for f in FINGERS:
        items += [vec_item('%s_mcp_hinge_centre' % f, 'mm', 0.5, 'caliper + scale photo', 'D0 (x along the palm centre line toward the fingertips, y across the palm toward the thumb side, z out of the palm face)'),
                  vec_item('%s_pip_hinge_centre_open' % f, 'mm', 0.5, 'caliper + scale photo', 'D0'), item('%s_proximal_length_mm' % f, 'mm', 0.5, 'caliper between hinge centres'),
                  item('%s_mcp_to_tip_open_mm' % f, 'mm', 0.5, 'caliper'), item('%s_pad_length_mm' % f, 'mm', 0.5, 'caliper on the fingertip pad'), item('%s_pad_width_mm' % f, 'mm', 0.5, 'caliper'),
                  item('%s_pad_thickness_mm' % f, 'mm', 0.2, 'caliper (pad over the rigid tip)'), item('%s_alpha_open_deg' % f, 'deg', 2.0, 'protractor overlay on a D2-plane photo at command 1000'),
                  item('%s_alpha_closed_deg' % f, 'deg', 2.0, 'protractor overlay at command 0'), vec_item('%s_tip_open' % f, 'mm', 1.0, 'scale photo', 'D0'), vec_item('%s_tip_closed' % f, 'mm', 1.0, 'scale photo', 'D0')]
    items += [vec_item('thumb_rotation_axis_point', 'mm', 0.5, 'caliper + scale photo', 'D0'), vec_item('thumb_rotation_axis_direction', 'unit', 0.02, 'two-photo triangulation', 'D0'),
              vec_item('thumb_mp_hinge_centre_open', 'mm', 0.5, 'caliper + scale photo', 'D0'), vec_item('thumb_ip_hinge_centre_open', 'mm', 0.5, 'caliper + scale photo', 'D0'),
              item('thumb_segment_1_mm', 'mm', 0.5, 'caliper'), item('thumb_segment_2_mm', 'mm', 0.5, 'caliper'), item('thumb_segment_3_mm', 'mm', 0.5, 'caliper'),
              item('thumb_pad_length_mm', 'mm', 0.5, 'caliper'), item('thumb_pad_width_mm', 'mm', 0.5, 'caliper'), item('thumb_pad_thickness_mm', 'mm', 0.2, 'caliper'),
              item('thumb_theta_open_deg', 'deg', 2.0, 'protractor overlay (Figure 5 theta) at command 1000'), item('thumb_theta_closed_deg', 'deg', 2.0, 'protractor overlay at command 0'),
              item('thumb_beta_open_deg', 'deg', 2.0, 'protractor overlay (Figure 5 beta) at rotation command 1000'), item('thumb_beta_closed_deg', 'deg', 2.0, 'protractor overlay at rotation command 0'),
              item('thumb_rotation_sense', 'text', None, 'observe: does rotation command 1000->0 move the thumb TOWARD the palm centre line (opposition) or away; record CW/CCW seen from the palm side'),
              item('palm_pad_length_mm', 'mm', 0.5, 'caliper'), item('palm_pad_width_mm', 'mm', 0.5, 'caliper'), item('aperture_thumb_tip_to_index_tip_open_mm', 'mm', 1.0, 'caliper'),
              item('aperture_thumb_tip_to_index_tip_closed_mm', 'mm', 1.0, 'caliper'),
              vec_item('flange_to_wrist_yaw_link_translation', 'mm', 0.5, 'caliper on the adapter plate + scale photo', 'right_wrist_yaw_link (G1 URDF frame)'),
              vec_item('flange_to_wrist_yaw_link_rotation_rpy', 'deg', 1.0, 'scale photo / square', 'right_wrist_yaw_link')]
    s['items'] = items; return s


def mass_com_sheet(side, model):
    s = base(side, model, 'mass_com'); s['items'] = [
        item('mass_hand_only_g', 'g', 1.0, 'scale, hand detached (only if it is already detached for another reason; never detach for this sheet)'),
        item('mass_hand_with_cable_g', 'g', 1.0, 'scale'), item('mass_wrist_adapter_g', 'g', 1.0, 'scale'), item('mass_as_mounted_total_g', 'g', 5.0, 'scale or NOT_MEASURED'),
        {'key': 'scale_identity', 'value': None, 'unit': 'text', 'method': 'model + last calibration/known-mass check (record the reference mass and its reading)', 'evidence': []},
        {'key': 'com_method', 'value': None, 'unit': 'text', 'method': 'declare: three-point suspension (plumb lines photographed on the fiducial sheet) or two-edge balance; hand pose at measurement (open, command 1000 all axes)', 'evidence': []},
        vec_item('com_hand_only', 'mm', 3.0, 'suspension/balance', 'D0'), vec_item('com_as_mounted_hand_plus_adapter', 'mm', 3.0, 'suspension/balance', 'D0'),
        {'key': 'com_readings', 'value': None, 'unit': 'text', 'method': 'per suspension point: attachment point in D0, plumb-line direction from the photo, file names', 'evidence': []},
        item('mass_closed_pose_check_g', 'g', 1.0, 'repeat the scale reading with the hand closed (mass must not change; a change flags a cable pulling on the scale)')]
    return s


def serial_firmware_sheet(side, model):
    s = base(side, model, 'serial_firmware'); s['privacy'] = 'SERIAL NUMBERS ARE PRIVATE: this sheet never enters the public repository once filled'
    s['items'] = [
        {'key': 'nameplate_model_code', 'value': None, 'source': 'nameplate photo (private)'}, {'key': 'serial_number', 'value': None, 'source': 'nameplate photo (private)'},
        {'key': 'tactile_variant_code_on_nameplate', 'value': None, 'source': 'nameplate (T1/T2/C1...)'}, {'key': 'firmware_version', 'value': None, 'source': 'vendor tool / label / bridge readback; the E2 manual lists NO user register for a firmware version (Table 41) - do not infer'},
        {'key': 'hand_id_register_1000', 'value': None, 'source': 'register readback'}, {'key': 'baud_rate_register_1002', 'value': None, 'source': 'register readback'},
        {'key': 'ip_address_registers_1700_1703', 'value': None, 'source': 'register readback or the bridge configuration'}, {'key': 'transport_in_use', 'value': None, 'source': 'declare: rs485_native | modbus_rtu | modbus_tcp | CAN | Unitree DDS bridge (topic names)'},
        {'key': 'modbus_register_index_convention_verified', 'value': None, 'source': 'the convention under which a Modbus read of angle_actual reproduces the RS485/bridge six values (register_map.ADDRESSING_CONVENTIONS)'},
        {'key': 'bridge_software_identity', 'value': None, 'source': 'name + version + commit of the driver/bridge in use (e.g. Unitree inspire service), if any'},
        {'key': 'power_on_speed_1032', 'value': None, 'source': 'register readback (six values)'}, {'key': 'power_on_force_threshold_1044', 'value': None, 'source': 'register readback (six values)'},
        {'key': 'finger_motion_mode_1625', 'value': None, 'source': 'register readback (six values)'}, {'key': 'dof_status_1612_raw', 'value': None, 'source': 'register readback; meaning not in the manual'}]
    return s


def actuator_endpoints_sheet(side, model):
    s = base(side, model, 'actuator_endpoints'); axes = {}
    for ax in rm.NATIVE_ORDER:
        axes[ax] = {'angle_actual_at_command_1000': None, 'angle_actual_at_command_0': None, 'actuator_position_actual_at_command_1000': None, 'actuator_position_actual_at_command_0': None,
                    'native_min_readback': None, 'native_max_readback': None, 'readback_units': None, 'command_units': None,
                    'direction': None, 'direction_note': 'readback_follows_command_counts | readback_opposes_command_counts (from analyze_actuator_log)',
                    'physical_open_angle_deg': None, 'physical_closed_angle_deg': None, 'physical_angle_definition': 'alpha (fingers) / theta (thumb bend) / beta (thumb rotation), manual Figure 5',
                    'speed_setting_during_test': None, 't90_full_stroke_s': None, 'settle_5counts_s': None, 'deadband_counts': None, 'log_files': [], 'notes': None}
    s['axes'] = axes; s['thumb_rotation_scale_note'] = 'the map from rotation counts to beta degrees is a separate affine (counts 1000->0 vs beta open->closed measured), never assumed equal to the fingers'
    return s


def tactile_sheet(side, model):
    s = base(side, model, 'tactile'); s['items'] = {
        'variant': None, 'variant_source': 'raw block capture (tactile_logger --tactile-variant unknown) inspected against Table 56 / Table 58 layouts; nameplate code',
        'channel_count_total': None, 'arrays': [], 'array_template': {'name': None, 'byte_address': None, 'rows': None, 'cols': None, 'orientation_note': 'row 1 col 1 location on the physical pad, from a touch test'},
        'units': None, 'units_source': 'raw 0-4095 (Table 56) or the Table 57 record floats; physical calibration only if the vendor documents it', 'rate_hz_achieved': None,
        'rest_nonzero_channels': None, 'touch_test_files': [], 'notes': None}
    return s


def write_all(side, model, out_dir):
    os.makedirs(out_dir, exist_ok=True); paths = {}
    for name, fn in (('geometry_sheet.json', geometry_sheet), ('mass_com_sheet.json', mass_com_sheet), ('serial_firmware_sheet.json', serial_firmware_sheet), ('actuator_endpoints_sheet.json', actuator_endpoints_sheet), ('tactile_sheet.json', tactile_sheet)):
        p = os.path.join(out_dir, name); json.dump(fn(side, model), open(p, 'w'), indent=1); paths[name] = p
    p = os.path.join(out_dir, 'register_map_sheet.csv'); open(p, 'w').write(rm.to_csv()); paths['register_map_sheet.csv'] = p
    return paths


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--side', required=True, choices=['left', 'right']); p.add_argument('--hardware-model', required=True); p.add_argument('--out-dir', required=True)
    a = p.parse_args(argv)
    for k, v in write_all(a.side, a.hardware_model, a.out_dir).items(): print(k, '->', v)


if __name__ == '__main__':
    sys.exit(main())
