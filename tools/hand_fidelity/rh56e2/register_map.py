"""User-accessible register map of the Inspire RH56E2 dexterous hand, transcribed from the manufacturer manual.

Source: Dexterous Hands User Manual for RH56E2 V1.0.0 (June 2025), Beijing Inspire-Robots, PRJ-01-TS-U-011,
sha256 add7aab76578cf08275529431b8b428d2d7facff626fe3a0bb2edbd1978983cd (hashed local copy under the campaign's
audit/references/inspire/). Page numbers are the manual's printed page numbers (section 2.6, pp. 19-33).

Addresses are the manual's BYTE addresses as used by the RS485 native frames (Table 4: Address_L/Address_H, Register_Length in
bytes). How Modbus RTU / Modbus TCP holding-register indices relate to these byte addresses is NOT stated in the manual
(sections 2.4-2.5 only show generic Modbus examples); see ADDRESSING_CONVENTIONS and verify on the installed unit before
trusting any Modbus read. Nothing here is an installed measurement.
"""
from __future__ import annotations

import csv
import io

MANUAL = {'id': 'e2_manual', 'title': 'Dexterous Hands User Manual for RH56E2 V1.0.0 (June 2025), PRJ-01-TS-U-011',
          'sha256': 'add7aab76578cf08275529431b8b428d2d7facff626fe3a0bb2edbd1978983cd',
          'url': 'https://en.inspire-robots.com/wp-content/uploads/2026/09/Dexterous-Hands-User-Manual-for-RH56E2_V1.0.0.pdf'}

# native axis order used by every six-element register group (Table 42-52, 54, 55)
NATIVE_ORDER = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')

# (name, byte address, length in bytes, element type, count, permission, range/unit, manual reference, note)
REGISTERS = [
    ('hand_id', 1000, 1, 'byte', 1, 'RW', '1-254 (default 1); saveable', 'Table 41 p.20; 2.6.1 p.21', ''),
    ('baud_rate', 1002, 1, 'byte', 1, 'RW', 'RS485 0-3 (0=115200, 1=57600, 2=19200, 3=921600); CAN 0-1 (0=1000K, 1=500K); not saveable', 'Table 41; 2.6.2 p.21', ''),
    ('error_clear', 1004, 1, 'byte', 1, 'RW', '0-1; write 1 clears clearable actuator errors', 'Table 41; 2.6.3 p.21', 'over-temperature not clearable'),
    ('save_to_flash', 1005, 1, 'byte', 1, 'RW', '0-1; write 1 saves parameters', 'Table 41; 2.6.4 p.22', ''),
    ('restore_factory_defaults', 1006, 1, 'byte', 1, 'RW', '0-1', 'Table 41; 2.6.5 p.22', 'never part of a measurement procedure'),
    ('force_sensor_calibration', 1009, 1, 'byte', 1, 'RW', '0-1; write 1 starts calibration; palm open, fingers untouched', 'Table 41; 2.6.6 p.22', 'never part of a measurement procedure'),
    ('power_on_speed', 1032, 12, 'short', 6, 'RW', '0-1000 per axis; 1000 = 600 ms full stroke no-load; saveable', 'Table 42 p.22', ''),
    ('power_on_force_threshold', 1044, 12, 'short', 6, 'RW', '0-3000 per axis (fingertip grip, g); saveable', 'Table 43 p.23', ''),
    ('actuator_position_set', 1474, 12, 'short', 6, 'RW', '0-2000 per axis; 0 = min stroke = fingers open, 2000 = max stroke = bent; -1 = no action; not saveable', 'Table 44 p.24', 'manual: not recommended for setting the angle'),
    ('angle_set', 1486, 12, 'short', 6, 'RW', '-1, 0-1000 per axis; 1000 = fully open (2.6.12 example), 0 = bent toward the palm; -1 = no action', 'Table 45 pp.24-25; Figure 5 pp.25-26', 'physical angle definitions alpha/theta/beta in Figure 5'),
    ('force_threshold_set', 1498, 12, 'short', 6, 'RW', '0-3000 per axis (fingertip grip, g)', 'Table 46 p.26', ''),
    ('speed_set', 1522, 12, 'short', 6, 'RW', '0-1000 per axis; 1000 = 600 ms full stroke no-load; saveable', 'Table 47 p.27', ''),
    ('actuator_position_actual', 1534, 12, 'short', 6, 'R', '0-2000 per axis', 'Table 48 pp.27-28', ''),
    ('angle_actual', 1546, 12, 'short', 6, 'R', '0-1000 per axis', 'Table 49 p.28; Table 7 p.5 (RS485 example reads 12 bytes from 1546)', 'Table 49 lists 1546-1553 and 1556-1557 only: the thumb-bending row (1554-1555) is ABSENT from the manual; its position is inferred from the 12-byte block and the six-value example. VERIFY on hardware'),
    ('force_actual', 1582, 12, 'short', 6, 'R', '-4000..4000 per axis, unit g', 'Table 50 pp.28-29', ''),
    ('actuator_current', 1594, 12, 'short', 6, 'R', '0-2000 per axis, unit mA', 'Table 51 p.29', ''),
    ('actuator_error_code', 1606, 6, 'byte', 6, 'R', 'bit0 locked-rotor, bit1 over-temperature, bit2 over-current, bit3 abnormal motor operation, bit4 communication error', 'Table 52-53 p.30', ''),
    ('dof_status', 1612, 6, 'byte', 6, 'R', 'NOT DESCRIBED in the manual (Table 41 row only)', 'Table 41 p.20', 'meaning unknown; log raw'),
    ('actuator_temperature', 1618, 6, 'byte', 6, 'R', '0-100 per axis, unit degC', 'Table 54 p.31', ''),
    ('finger_motion_mode', 1625, 6, 'byte', 6, 'RW', '0 = speed-force protection, 1 = force closed-loop, per axis', 'Table 55 pp.31-32', ''),
    ('ip_octet_1', 1700, 1, 'byte', 1, 'RW', '0-255, default 192; effective after re-power', 'Table 41 p.20', ''),
    ('ip_octet_2', 1701, 1, 'byte', 1, 'RW', '0-255, default 168', 'Table 41 p.20', ''),
    ('ip_octet_3', 1702, 1, 'byte', 1, 'RW', '0-255, default 11', 'Table 41 p.20', ''),
    ('ip_octet_4', 1703, 1, 'byte', 1, 'RW', '0-255, default 210', 'Table 41 p.20', ''),
]

# Tactile blocks. The manual's section 2.6.21 (pp.33-35) describes the RH56DFTP piezoresistive layout (Table 56) and the
# RH56DFTP -C1 capacitive layout (Tables 57-58). Which layout an RH56E2-T1 unit carries is NOT stated: declare the
# variant only after the installed unit has been read.
TACTILE_PIEZORESISTIVE_TABLE56 = [  # (name, byte address, rows, cols, length bytes)
    ('little_tip', 3000, 3, 3, 18), ('little_nail', 3018, 12, 8, 192), ('little_pad', 3210, 10, 8, 160),
    ('ring_tip', 3370, 3, 3, 18), ('ring_nail', 3388, 12, 8, 192), ('ring_pad', 3580, 10, 8, 160),
    ('middle_tip', 3740, 3, 3, 18), ('middle_nail', 3758, 12, 8, 192), ('middle_pad', 3950, 10, 8, 160),
    ('index_tip', 4110, 3, 3, 18), ('index_nail', 4128, 12, 8, 192), ('index_pad', 4320, 10, 8, 160),
    ('thumb_tip', 4480, 3, 3, 18), ('thumb_nail', 4498, 12, 8, 192), ('thumb_middle', 4690, 3, 3, 18), ('thumb_tip_12x8', 4708, 12, 8, 192),
    ('palm', 4900, 8, 14, 224),
]
TACTILE_PIEZORESISTIVE_NOTE = ('each point is a 16-bit little-endian integer 0-4095; finger arrays are row-major from row 1 col 1; the palm array is '
                               'column-major starting at row 8 col 1 (Table 56 text, pp.33-34); Table 56 names two thumb "tip" arrays (3x3 at 4480 and 12x8 at 4708) as printed')
TACTILE_CAPACITIVE_TABLE58 = [('little', 3000, 58), ('ring', 3058, 58), ('middle', 3116, 58), ('index', 3174, 58), ('thumb', 3232, 58)]  # (finger, byte address, length)
TACTILE_CAPACITIVE_NOTE = ('58-byte record per finger (Table 57): normal force, normal-force delta, tangential force, tangential-force delta (float32 LE x4 each), '
                           'tangential direction (uint16 x2? printed "2"), proximity delta (4), check (2), reserved (2); direction 0-359 deg clockwise from the fingertip, 0xFFFF = invalid')

ADDRESSING_CONVENTIONS = {
    'rs485_native': 'byte addresses exactly as listed (Table 4/8 frames: header EB 90, ID, length, 0x11 read / 0x12 write, Address_L, Address_H, data, checksum = low byte of the sum of all bytes after the header); response header 90 EB',
    'modbus_holding_register_index': 'NOT STATED in the manual. Candidates to verify on the installed unit: (a) holding-register index == byte address, one 16-bit register per address step of 2 (read 1546 with quantity 6 -> six shorts), (b) index == byte address with quantity 12, (c) index == byte address / 2. The verification reads angle_actual through both the RS485 frame (if available) and Modbus and requires identical six values.',
    'modbus_tcp_endpoint': 'default IP 192.168.11.210, port 6000 (manual p.4); MBAP unit identifier 0xFF (2.5); function codes 03 / 06 / 16 only',
    'byte_order_in_native_frames': 'little-endian shorts (Table 7/11 examples: 0x64 0x00 = 100, 0xD0 0x07 = 2000)',
}


def register(name):
    for r in REGISTERS:
        if r[0] == name:
            return {'name': r[0], 'address': r[1], 'length_bytes': r[2], 'element': r[3], 'count': r[4], 'permission': r[5], 'range': r[6], 'reference': r[7], 'note': r[8]}
    raise KeyError(name)


def axis_offset(name, axis):
    """Byte offset of one native axis inside a six-element group."""
    r = register(name)
    if r['count'] != 6:
        raise ValueError('%s is not a per-axis group' % name)
    size = 2 if r['element'] == 'short' else 1
    return NATIVE_ORDER.index(axis) * size


def decode_group(name, payload):
    """Decode a raw payload read from a group into {axis: value} (shorts little-endian signed; bytes unsigned)."""
    r = register(name)
    if len(payload) != r['length_bytes']:
        raise ValueError('%s expects %d bytes, got %d' % (name, r['length_bytes'], len(payload)))
    if r['count'] == 1:
        return {'value': int.from_bytes(payload, 'little', signed=False)}
    if r['element'] == 'short':
        return {ax: int.from_bytes(payload[2 * i:2 * i + 2], 'little', signed=True) for i, ax in enumerate(NATIVE_ORDER)}
    return {ax: payload[i] for i, ax in enumerate(NATIVE_ORDER)}


def encode_group(name, values):
    """Encode {axis: int} (or a single int) into the little-endian payload of a writable group."""
    r = register(name)
    if 'W' not in r['permission']:
        raise ValueError('%s is read-only' % name)
    if r['count'] == 1:
        return int(values).to_bytes(1, 'little', signed=False)
    out = b''
    for ax in NATIVE_ORDER:
        v = int(values[ax])
        out += v.to_bytes(2, 'little', signed=True) if r['element'] == 'short' else v.to_bytes(1, 'little', signed=False)
    return out


def check_layout():
    """Non-overlap and ordering of the transcribed map (a transcription self-check, not a hardware check)."""
    spans = sorted((r[1], r[1] + r[2], r[0]) for r in REGISTERS)
    for (a0, a1, n), (b0, b1, m) in zip(spans, spans[1:]):
        if b0 < a1:
            raise AssertionError('overlap %s/%s' % (n, m))
    t = sorted((a, a + L, n) for n, a, _, _, L in TACTILE_PIEZORESISTIVE_TABLE56)
    for (a0, a1, n), (b0, b1, m) in zip(t, t[1:]):
        if b0 != a1:
            raise AssertionError('piezoresistive tactile blocks not contiguous at %s/%s' % (n, m))
    for n, a, rows, cols, L in TACTILE_PIEZORESISTIVE_TABLE56:
        if rows * cols * 2 != L:
            raise AssertionError('tactile %s: %dx%d points != %d bytes' % (n, rows, cols, L))
    return True


def to_csv():
    buf = io.StringIO(); w = csv.writer(buf)
    w.writerow(['name', 'byte_address', 'length_bytes', 'element', 'count', 'permission', 'range_or_unit', 'manual_reference', 'note', 'verified_on_installed_unit', 'installed_readback_example', 'verified_utc', 'operator'])
    for r in REGISTERS:
        w.writerow(list(r) + ['', '', '', ''])
    for n, a, rows, cols, L in TACTILE_PIEZORESISTIVE_TABLE56:
        w.writerow(['tactile_piezo_' + n, a, L, 'uint16', rows * cols, 'R', '%dx%d points 0-4095 (Table 56; variant applicability to E2-T1 UNVERIFIED)' % (rows, cols), 'Table 56 pp.33-34', '', '', '', '', ''])
    for n, a, L in TACTILE_CAPACITIVE_TABLE58:
        w.writerow(['tactile_cap_' + n, a, L, 'record', 1, 'R', 'Table 57 record (variant applicability to E2-T1 UNVERIFIED)', 'Table 57-58 p.35', '', '', '', '', ''])
    return buf.getvalue()


if __name__ == '__main__':
    check_layout(); print(to_csv())
