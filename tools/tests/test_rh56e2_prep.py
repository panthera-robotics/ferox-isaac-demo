"""RH56E2 measurement-preparation package: transcription self-checks against the manual's worked examples, the hardware gate,
dry-run loggers, the conversion template's refusal on missing values, and the completeness gate. CPU only; nothing here
touches a port, a socket or an installed hand."""
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__)); TOOLS = os.path.dirname(HERE)
sys.path.insert(0, TOOLS)
from hand_fidelity.rh56e2 import register_map as rm, transport as tr  # noqa: E402
from hand_fidelity.rh56e2.analyze_actuator_log import analyze, load  # noqa: E402
from hand_fidelity.rh56e2.compare_installed import compare  # noqa: E402
from hand_fidelity.rh56e2.conversion_template import build  # noqa: E402
from hand_fidelity.rh56e2.sheets import write_all  # noqa: E402

PY = sys.executable


def test_register_map_layout_and_manual_examples():
    assert rm.check_layout()
    assert rm.register('angle_set')['address'] == 1486 and rm.register('angle_actual')['address'] == 1546
    assert rm.decode_group('angle_actual', bytes.fromhex('6400640064006400d0070000')) == dict(zip(rm.NATIVE_ORDER, [100, 100, 100, 100, 2000, 0]))   # manual Table 7
    assert rm.encode_group('angle_set', dict(zip(rm.NATIVE_ORDER, [100, 100, 100, 100, 2000, 0]))).hex() == '6400640064006400d0070000'            # manual Table 11
    assert 'ABSENT' in rm.register('angle_actual')['note']                                                                                          # thumb-bending row missing in Table 49
    assert rm.axis_offset('angle_set', 'index') == 6 and rm.axis_offset('actuator_error_code', 'thumb_rotation') == 5
    with pytest.raises(ValueError):
        rm.encode_group('angle_actual', {})
    csv_rows = rm.to_csv().splitlines(); assert csv_rows[0].startswith('name,byte_address') and len(csv_rows) == 1 + len(rm.REGISTERS) + len(rm.TACTILE_PIEZORESISTIVE_TABLE56) + len(rm.TACTILE_CAPACITIVE_TABLE58)


def test_rs485_frames_match_manual_tables_6_7_11():
    assert tr.rs485_read_frame(1, 1546, 12).hex() == 'eb900104110a060c32'
    assert tr.rs485_write_frame(1, 1486, bytes.fromhex('6400640064006400d0070000')).hex() == 'eb90010f12ce056400640064006400d00700005c'
    hid, addr, payload = tr.rs485_parse_response(bytes.fromhex('90eb010f110a06') + bytes.fromhex('6400640064006400d0070000') + bytes([0x98]), 0x11)
    assert (hid, addr, payload.hex()) == (1, 1546, '6400640064006400d0070000')
    with pytest.raises(ValueError):
        tr.rs485_parse_response(bytes.fromhex('90eb010f110a06') + bytes(12) + bytes([0x00]), 0x11)
    assert tr.crc16_modbus(bytes.fromhex('010300000002')) == 0x0BC4


def test_hardware_gate_refuses_every_real_transport_without_all_three_switches(monkeypatch):
    monkeypatch.delenv(tr.ENV_FLAG, raising=False)
    for kind in ('rs485_native', 'modbus_tcp', 'modbus_rtu'):
        with pytest.raises(tr.HardwareNotAuthorized):
            tr.open_transport(kind, tr.Authorization(hardware_authorized=True, operator='someone'))
    monkeypatch.setenv(tr.ENV_FLAG, '1')
    with pytest.raises(tr.HardwareNotAuthorized):
        tr.open_transport('modbus_tcp', tr.Authorization(hardware_authorized=True, operator=None))
    with pytest.raises(tr.HardwareNotAuthorized):
        tr.Authorization(True, 'someone', allow_writes=False).check('modbus_tcp', write=True)
    assert tr.open_transport('dry_run', tr.Authorization()).kind == 'dry_run'


def test_modbus_index_conventions_are_explicit():
    assert tr._modbus_index(1546, 12, 'index_eq_byte_address_qty_words') == (1546, 6)
    assert tr._modbus_index(1546, 12, 'index_eq_half_byte_address') == (773, 6)
    with pytest.raises(ValueError):
        tr._modbus_index(1546, 12, 'guess')
    assert tr._words_to_payload([100, 2000], 2).hex() == '6400d007'


def test_dry_run_loggers_write_records_without_values(tmp_path):
    plan = tmp_path / 'plan.json'; plan.write_text(json.dumps({'steps': [{'t_s': 0.0, 'angle_set': {'index': 1000}}]}))
    out = tmp_path / 'act.jsonl'
    r = subprocess.run([PY, '-m', 'hand_fidelity.rh56e2.actuator_logger', '--out', str(out), '--side', 'right', '--hardware-model', 'RH56E2-2R-T1', '--rate-hz', '10', '--duration-s', '0.25', '--plan', str(plan)], cwd=TOOLS, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    recs = [json.loads(l) for l in out.read_text().splitlines()]
    assert recs[0]['kind'] == 'header' and recs[0]['dry_run'] is True and recs[0]['values_are_installed_measurements'] is False
    assert any(x['kind'] == 'command_planned_not_sent' for x in recs) and not any(x['kind'] == 'write' for x in recs)
    assert all(x['raw_hex'] is None and x['decoded'] is None for x in recs if x['kind'] == 'read')
    hdr, rr = load(str(out)); a = analyze(hdr, rr); assert a['verdict'].startswith('NOT_MEASURED') and a['axes']['index']['direction'] == 'NOT_MEASURED'
    out2 = tmp_path / 'tac.jsonl'
    r = subprocess.run([PY, '-m', 'hand_fidelity.rh56e2.tactile_logger', '--out', str(out2), '--side', 'right', '--hardware-model', 'RH56E2-2R-T1', '--rate-hz', '5', '--duration-s', '0.1'], cwd=TOOLS, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    recs = [json.loads(l) for l in out2.read_text().splitlines()]; assert recs[0]['tactile_variant'] == 'unknown' and all(x['decoded'] is None for x in recs if x['kind'] == 'read')
    bad = subprocess.run([PY, '-m', 'hand_fidelity.rh56e2.actuator_logger', '--out', str(tmp_path / 'x.jsonl'), '--side', 'right', '--hardware-model', 'RH56E2-2R-T1', '--transport', 'modbus_tcp'], cwd=TOOLS, capture_output=True, text=True)
    assert bad.returncode != 0 and 'refused' in bad.stderr


def test_analyze_computes_direction_and_step_response_from_a_real_shaped_log():
    recs = [{'kind': 'header', 'schema': 'rh56e2_log_v1', 'transport': 'modbus_tcp', 'dry_run': False, 'values_are_installed_measurements': True}]
    t = 0.0
    def rd(t, v): return {'kind': 'read', 'group': 'angle_actual', 'mono_s': t, 'decoded': dict(zip(rm.NATIVE_ORDER, [1000, 1000, 1000, v, 1000, 1000]))}
    for i in range(5): recs.append(rd(t + i * 0.05, 1000))
    recs.append({'kind': 'write', 'group': 'angle_set', 'mono_s': 0.26, 'sent': True, 'values': dict(zip(rm.NATIVE_ORDER, [-1, -1, -1, 0, -1, -1]))})
    for i, v in enumerate([1000, 900, 600, 300, 100, 20, 5, 0, 0, 0]): recs.append(rd(0.3 + i * 0.05, v))
    a = analyze(recs[0], recs)
    idx = a['axes']['index']; assert idx['direction'] == 'readback_follows_command_counts' and idx['steps'][0]['t90_s'] is not None and idx['angle_actual_min_max'] == [0, 1000]
    assert a['axes']['little']['direction'] == 'NOT_MEASURED'


def test_conversion_template_refuses_blanks_and_maps_a_filled_axis_set(tmp_path):
    paths = write_all('right', 'RH56E2-2R-T1', str(tmp_path)); sheet = json.load(open(paths['actuator_endpoints_sheet.json']))
    r = build(sheet); assert r['status'] == 'INCOMPLETE' and 'little.angle_actual_at_command_1000' in r['missing'] and 'six_axis_convention' not in r
    sheet.update({'measured_utc': '2026-01-01T00:00:00Z', 'operator': 'unit test (not a measurement)'})
    for ax in rm.NATIVE_ORDER:
        sheet['axes'][ax].update({'angle_actual_at_command_1000': 998, 'angle_actual_at_command_0': 3, 'direction': 'readback_follows_command_counts', 'readback_units': 'counts', 'command_units': 'counts',
                                  'physical_open_angle_deg': 170.0 if ax != 'thumb_bend' else 28.5, 'physical_closed_angle_deg': 91.5 if ax not in ('thumb_bend', 'thumb_rotation') else (64.0 if ax == 'thumb_bend' else 75.0), 'log_files': ['x.jsonl']})
    r = build(sheet); assert r['status'] == 'COMPLETE'
    m = r['axes']['index']['affine_native_to_donor_rad']; assert abs(m['slope_rad_per_count'] * 998 + m['intercept_rad']) < 1e-9 and abs(m['slope_rad_per_count'] * 3 + m['intercept_rad'] - 1.4381) < 1e-9
    assert r['six_axis_convention']['per_axis_endpoints']['thumb_rotation'] == [998.0, 3.0] and r['six_axis_convention']['axis_order'] == list(rm.NATIVE_ORDER)


def test_completeness_gate_on_blank_sheets(tmp_path):
    write_all('right', 'RH56E2-2R-T1', str(tmp_path)); rep = compare(str(tmp_path))
    assert rep['label'] == 'RH56E2_INSTALLED_CALIBRATION_INCOMPLETE' and rep['overall_real_hand_similarity_percent'] is None
    assert set(rep['missing_for_label']) >= {'palm_finger_geometry', 'hinge_positions', 'pad_dimensions', 'open_closed_endpoints', 'total_mass', 'approximate_com', 'actuator_native_min_max', 'direction', 'command_readback_units', 'speed_response', 'deadband', 'firmware_transport', 'thumb_rotation_direction_scale', 'tactile'}
    assert all(r.get('status') in ('NOT_MEASURED', 'INCOMPLETE') for r in rep['rows'])
