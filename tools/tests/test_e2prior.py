"""PUBLIC exact-E2 prior: the six-axis adapter (orders, counts <-> closure <-> radians, child-limit clamps, donor->E2 mapping), the
kinematic contract's consistency with the adapter, the embodiment manifest builder, and the Modbus byte-group fix (fake Modbus TCP
server in-process: both bytes of every word are retained, the declared byte order is explicit, 16-bit groups are unaffected).
CPU only; nothing opens a real port. The register image of the fake server is SYNTHETIC (no installed capture exists yet); the
INSTALLED_CAPTURE hook below is where an owner-provided capture (raw words + expected channels) plugs in."""
import json
import os
import socket
import struct
import sys
import threading

import pytest

HERE = os.path.dirname(os.path.abspath(__file__)); TOOLS = os.path.dirname(HERE)
sys.path.insert(0, TOOLS); sys.path.insert(0, os.path.dirname(TOOLS))
from hand_fidelity.rh56e2 import e2_adapter as ad, register_map as rm, transport as tr  # noqa: E402
from hand_fidelity.rh56e2.e2_embodiment_manifest import build as build_manifest  # noqa: E402

INSTALLED_CAPTURE = None   # {'words_1606': [...], 'words_1612': [...], 'words_1618': [...], 'byte_order': 'low_high'|'high_low', 'expected': {...}} when the owner provides one


def test_orders_and_names():
    assert ad.reorder([1, 2, 3, 4, 5, 6], ad.NATIVE_ORDER, ad.CONTRACT_ORDER) == [4, 3, 2, 1, 5, 6]
    assert ad.independent_joint_names('right') == ['right_index_proximal_joint', 'right_middle_proximal_joint', 'right_ring_proximal_joint', 'right_pinky_proximal_joint', 'right_thumb_proximal_pitch_joint', 'right_thumb_proximal_yaw_joint']
    assert ad.name_map('left')['left_thumb_1_joint'] == 'left_thumb_proximal_yaw_joint' and ad.name_map('left')['left_little_2_joint'] == 'left_pinky_intermediate_joint'
    with pytest.raises(ValueError):
        ad.e2_joint('centre', 'index')


def test_counts_closure_radians_round_trip_and_direction():
    q = ad.counts_to_e2([1000] * 6, 'right')
    assert all(abs(v) < 1e-12 for v in q.values())                                             # 1000 = fully open = URDF zero, thumb rotation 1000 = out = yaw 0
    q = ad.counts_to_e2([0] * 6, 'right')
    assert q['right_index_proximal_joint'] == pytest.approx(1.4381) and q['right_thumb_proximal_yaw_joint'] == pytest.approx(1.658) and q['right_thumb_proximal_pitch_joint'] == pytest.approx(0.62)
    assert q['right_index_intermediate_joint'] == pytest.approx(1.476374) and q['right_thumb_distal_joint'] == pytest.approx(0.45553093477052004)   # clamped at the child limit
    assert q['right_thumb_intermediate_joint'] == pytest.approx(0.8392 * 0.62)                                                                    # never saturates
    for counts in ([1000, 750, 500, 250, 0, 1000], [123, 456, 789, 12, 345, 678]):
        assert ad.e2_to_counts(ad.counts_to_e2(counts, 'left'), 'left') == counts
    half = ad.counts_to_e2([500] * 6, 'right')
    assert half['right_index_intermediate_joint'] == pytest.approx(1.0843 * 0.5 * 1.4381)      # below the child limit: pure ratio
    with pytest.raises(ValueError):
        ad.counts_to_closure([1001, 0, 0, 0, 0, 0])
    with pytest.raises(ValueError):
        ad.counts_to_closure([0] * 5)


def test_donor_rows_map_closure_preserving_not_radian_identity():
    q = ad.donor_rows_to_e2([1.4381, 1.4381, 1.4381, 1.4381, 0.5864, 1.1641], 'right')        # donor fully closed
    assert q['right_thumb_proximal_yaw_joint'] == pytest.approx(1.658) and q['right_thumb_proximal_pitch_joint'] == pytest.approx(0.62)
    q = ad.donor_rows_to_e2([0.71905, 0, 0, 0, 0.2932, 0.58205], 'left')                       # half closure on index / thumb axes
    assert q['left_index_proximal_joint'] == pytest.approx(0.71905) and q['left_thumb_proximal_yaw_joint'] == pytest.approx(0.829) and q['left_thumb_proximal_pitch_joint'] == pytest.approx(0.31)
    with pytest.raises(ValueError):
        ad.donor_rows_to_e2([0, 0, 0, 0, 0, 1.3], 'right')                                    # outside the donor thumb range: refused, not clipped


def test_contract_matches_adapter():
    c = ad.contract()
    for side in ad.SIDES:
        for axis in ad.CONTRACT_ORDER:
            assert c['independent_joints'][side][axis]['joint'] == ad.e2_joint(side, axis) and c['independent_joints'][side][axis]['closed_rad'] == ad.E2_LIMITS[axis][1]
        assert set(c['coupled_joints'][side]) == {side + '_' + k for k in ad.COUPLING} and all(v['clamp_at_child_limit'] for v in c['coupled_joints'][side].values())
    assert c['label'] == 'PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE' and '1000 = fully open' in c['count_mapping']['closure']


def test_embodiment_manifest_builder_validates_against_ferox_schema(tmp_path):
    root = os.path.dirname(TOOLS); donor_path = os.path.join(root, 'isaac', 'twin', 'inspire', 'embodiments', 'g1_edu29_rh56dftp_donor_v1.json')
    if not os.path.exists(donor_path):
        pytest.skip('donor manifest not in this checkout')
    donor = json.load(open(donor_path))
    asset = {'asset_id': 'g1_edu29_rh56e2_e2prior_v1', 'merged_urdf': {'sha256': '0' * 64, 'name': 'g1_29dof_rev_1_0_with_inspire_hand_E2.urdf'}, 'source': {'url': 'u', 'commit': 'c'},
             'mass_policy': {'primary': 'PUBLIC_URDF_LINK_MASSES', 'hand_total_kg': {'right': 0.776664, 'left': 0.776664}},
             'hands': {s: {'wrist_mount': {'xyz_m': [0.0415, 0, 0], 'rpy_rad': [1.5707963, 0, 1.5707963] if s == 'right' else [-1.5707963, 0, -1.5707963], 'provenance': 'test'}} for s in ('right', 'left')}}
    m = build_manifest(asset, donor, None); p = tmp_path / 'm.json'; p.write_text(json.dumps(m))
    from isaac.twin.inspire.embodiment import EmbodimentManifest
    em = EmbodimentManifest.load(str(p))
    assert em.hand_actuator('right', 'thumb_rotation')['closed_rad'] == 1.658 and em.hand_actuator('left', 'index')['joint'] == 'left_index_proximal_joint'
    assert all(v['status'] == 'NOT_RUN' for k, v in em.data['qualification']['claims'].items() if k != 'static_asset_audit')
    assert donor['hands']['right']['actuators']['thumb_rotation']['closed_rad'] == 1.1641       # donor untouched by the builder


# ---------------------------------------------------------------- Modbus byte-group fix

class FakeModbusTcp:
    """Holding-register image served over Modbus TCP (FC 03 / FC 16); records every request."""
    def __init__(self, image):
        self.image = dict(image); self.requests = []; self.sock = socket.socket(); self.sock.bind(('127.0.0.1', 0)); self.sock.listen(1); self.port = self.sock.getsockname()[1]
        self.t = threading.Thread(target=self._serve, daemon=True); self.t.start()

    def _serve(self):
        while True:
            conn, _ = self.sock.accept(); self._serve_conn(conn); conn.close()

    def _serve_conn(self, conn):
        while True:
            hdr = conn.recv(7)
            if len(hdr) < 7: break
            tid, proto, ln, unit = struct.unpack('>HHHB', hdr); pdu = conn.recv(ln - 1); fc = pdu[0]; self.requests.append((unit, pdu))
            if fc == 3:
                idx, qty = struct.unpack('>HH', pdu[1:5]); words = [self.image.get(idx + i, 0) for i in range(qty)]
                body = bytes([3, 2 * qty]) + b''.join(struct.pack('>H', w) for w in words)
            elif fc == 16:
                idx, qty, nb = struct.unpack('>HHB', pdu[1:6]); ws = struct.unpack('>%dH' % qty, pdu[6:6 + 2 * qty])
                for i, w in enumerate(ws): self.image[idx + i] = w
                body = pdu[:5]
            else:
                body = bytes([fc | 0x80, 1])
            conn.sendall(struct.pack('>HHHB', tid, 0, len(body) + 1, unit) + body)


SYNTHETIC_IMAGE = {   # convention (a): register index == byte address of the group, one register per short, CONSECUTIVE indices inside a group
    **{1546 + i: v for i, v in enumerate([1000, 750, 500, 250, 0, 1000])},          # angle_actual (six shorts)
    **{1534 + i: v for i, v in enumerate([0, 500, 1000, 1500, 2000, 1234])},         # actuator_position_actual
    1606: 0x0201, 1607: 0x0403, 1608: 0x0605,                                        # error codes: bytes 1..6 (low_high reads 1,2,3,4,5,6)
    1618: 0x1F1E, 1619: 0x2120, 1620: 0x2322,                                        # temperatures 30..35 degC
}


def _open(port, byte_order):
    auth = tr.Authorization(True, 'test operator', True); os.environ[tr.ENV_FLAG] = '1'
    try:
        return tr.open_transport('modbus_tcp', auth, host='127.0.0.1', port=port, unit_id=1, byte_order=byte_order)
    finally:
        del os.environ[tr.ENV_FLAG]


def test_byte_groups_emit_both_bytes_per_word_in_the_declared_order_and_16bit_groups_are_unaffected():
    srv = FakeModbusTcp(SYNTHETIC_IMAGE)
    for order, expected_err, expected_temp in (('low_high', [1, 2, 3, 4, 5, 6], [30, 31, 32, 33, 34, 35]), ('high_low', [2, 1, 4, 3, 6, 5], [31, 30, 33, 32, 35, 34])):
        t = _open(srv.port, order)
        err = t.read(1606, 6, 1); assert list(err) == expected_err and t.last_words == [0x0201, 0x0403, 0x0605] and tr.word_bytes(t.last_words) == [(1, 2), (3, 4), (5, 6)]
        temp = t.read(1618, 6, 1); assert list(temp) == expected_temp and len(temp) == 6
        assert rm.decode_group('actuator_temperature', temp) == dict(zip(rm.NATIVE_ORDER, expected_temp))
        ang = t.read(1546, 12, 2); assert rm.decode_group('angle_actual', ang) == dict(zip(rm.NATIVE_ORDER, [1000, 750, 500, 250, 0, 1000]))         # independent of the byte order
        pos = t.read(1534, 12, 2); assert rm.decode_group('actuator_position_actual', pos) == dict(zip(rm.NATIVE_ORDER, [0, 500, 1000, 1500, 2000, 1234]))
        assert srv.requests[-1][0] == 1                                                                                                                  # unit id 1 on the wire
        t.close()
    # undeclared: byte groups are not decoded, words retained, shorts still decoded
    t = _open(srv.port, None)
    assert t.read(1606, 6, 1) is None and t.last_words == [0x0201, 0x0403, 0x0605]
    assert rm.decode_group('angle_actual', t.read(1546, 12, 2))['index'] == 250
    with pytest.raises(ValueError):
        t.write(1625, bytes([1, 1, 1, 1, 1, 1]), 1)                                                   # byte-group write needs a declared order
    t.close()


def test_regression_the_old_decoder_kept_one_byte_per_word():
    words = [0x0201, 0x0403, 0x0605]
    old = bytes(w & 0xFF for w in words)                                                              # the defect: 3 bytes, high bytes lost
    assert len(old) == 3 and list(old) == [1, 3, 5]
    assert list(tr._words_to_payload(words, 1, 6, 'low_high')) == [1, 2, 3, 4, 5, 6] and list(tr._words_to_payload(words, 1, 6, 'high_low')) == [2, 1, 4, 3, 6, 5]
    assert tr._words_to_payload(words, 1, 6, None) is None
    assert tr._words_to_payload([1000, 0xFFFF], 2) == bytes.fromhex('e803ffff')                        # 16-bit path byte-identical to before the fix
    assert tr._payload_to_words(bytes([1, 2, 3, 4]), 1, 'low_high') == [0x0201, 0x0403] and tr._payload_to_words(bytes([1, 2]), 1, 'high_low') == [0x0102]
    with pytest.raises(ValueError):
        tr._payload_to_words(bytes([1]), 1, 'low_high')


def test_installed_profiles_and_cli_flags(tmp_path):
    assert tr.INSTALLED_PROFILES == {'left': {'host': '192.168.123.210', 'port': 6000, 'unit_id': 1}, 'right': {'host': '192.168.123.211', 'port': 6000, 'unit_id': 1}}
    import argparse
    from hand_fidelity.rh56e2.logging_common import add_common_args, header_for, open_from_args
    p = argparse.ArgumentParser(); add_common_args(p)
    a = p.parse_args(['--out', str(tmp_path / 'x.jsonl'), '--side', 'left', '--hardware-model', 'RH56E2-2L-T1', '--profile', 'left'])
    assert a.transport == 'dry_run' and a.modbus_byte_order is None                                     # dry-run remains the default; byte order undeclared
    h = header_for(a, 'test'); assert h['transport_params']['modbus_byte_order'] == 'UNDECLARED' and h['dry_run'] is True
    auth, t = open_from_args(a); assert t.kind == 'dry_run'
    a = p.parse_args(['--out', str(tmp_path / 'x.jsonl'), '--side', 'right', '--hardware-model', 'RH56E2-2R-T1', '--profile', 'left', '--transport', 'modbus_tcp'])
    with pytest.raises(SystemExit):
        open_from_args(a)                                                                                # profile/side mismatch refused before any socket
    a = p.parse_args(['--out', str(tmp_path / 'x.jsonl'), '--side', 'right', '--hardware-model', 'RH56E2-2R-T1', '--profile', 'right', '--transport', 'modbus_tcp'])
    with pytest.raises(tr.HardwareNotAuthorized):
        open_from_args(a)                                                                                # gate still closed without the three switches


def test_installed_capture_hook_is_documented_and_empty():
    assert INSTALLED_CAPTURE is None   # replace with the owner's capture; the synthetic image above is not an installed measurement


def test_wrist_mount_binding_accepts_the_e2_flange_and_keeps_the_donor():
    """The twin used to hard-code right_wrist_yaw_link -> right_base_link; the E2 flange child is right_base."""
    import xml.etree.ElementTree as ET
    from isaac.twin.inspire.embodiment import ContractError, fixed_joint_matrix, wrist_mount_joint, wrist_mount_pair_from_manifest, dependency_values_from_urdf
    e2 = ET.fromstring('<robot name="e2"><link name="right_wrist_yaw_link"/><link name="right_base"/><joint name="right_hand_connection_joint" type="fixed"><origin xyz="0.0415 0 0" rpy="1.5707963 0 1.5707963"/><parent link="right_wrist_yaw_link"/><child link="right_base"/></joint>'
                       '<link name="torso_link"/><link name="d435_link"/><joint name="d" type="fixed"><origin xyz="0 0 0" rpy="0 0 0"/><parent link="torso_link"/><child link="d435_link"/></joint></robot>')
    donor = ET.fromstring('<robot name="d"><link name="right_wrist_yaw_link"/><link name="right_base_link"/><joint name="right_base_joint" type="fixed"><origin xyz="0.0415 0 0" rpy="0 1.5707963 0"/><parent link="right_wrist_yaw_link"/><child link="right_base_link"/></joint></robot>')
    assert wrist_mount_joint(e2)['child_link'] == 'right_base' and wrist_mount_joint(donor)['child_link'] == 'right_base_link'
    assert wrist_mount_joint(e2, ('right_wrist_yaw_link', 'right_base'))['xyz_m'] == [0.0415, 0.0, 0.0]
    with pytest.raises(ContractError):
        wrist_mount_joint(e2, ('right_wrist_yaw_link', 'right_base_link'))
    assert wrist_mount_pair_from_manifest({'transforms': {'right': {'wrist_to_hand': {'frame': 'right_wrist_yaw_link -> right_base'}}}}) == ('right_wrist_yaw_link', 'right_base')
    assert wrist_mount_pair_from_manifest({'transforms': {'right': {'wrist_to_hand': None}}}) is None
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(ET.tostring(e2, encoding='unicode'))
    live = dependency_values_from_urdf(f.name, collision_cooking='meshes')
    import hashlib
    assert live['wrist_mount_sha256'] == hashlib.sha256(json.dumps(fixed_joint_matrix(wrist_mount_joint(e2))).encode()).hexdigest()
