"""One record format for every RH56E2 logger (JSONL, one object per line) and the CLI switches shared by the loggers."""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import platform
import time

from . import register_map as rm
from .transport import Authorization, open_transport

SCHEMA = 'rh56e2_log_v1'


def add_common_args(p):
    p.add_argument('--out', required=True, help='JSONL log path (private storage; never the public repository)')
    p.add_argument('--transport', default='dry_run', choices=['dry_run', 'rs485_native', 'modbus_tcp', 'modbus_rtu'])
    p.add_argument('--port', help='serial device for rs485_native / modbus_rtu'); p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--host', default='192.168.11.210'); p.add_argument('--tcp-port', type=int, default=6000)
    p.add_argument('--hand-id', type=int, default=1, help='RS485/RTU hand id (register 1000; default 1)')
    p.add_argument('--modbus-convention', default='index_eq_byte_address_qty_words', help='declared Modbus register-index convention (UNVERIFIED until checked on the unit)')
    p.add_argument('--side', required=True, choices=['left', 'right']); p.add_argument('--hardware-model', required=True, help='e.g. RH56E2-2R-T1 from the nameplate photo')
    p.add_argument('--hardware-authorized', action='store_true', help='first of three switches; also needs PANTHERA_HARDWARE_AUTHORIZED=1 and --operator')
    p.add_argument('--operator', help='named human present at the hand'); p.add_argument('--allow-writes', action='store_true', help='required for any register write')
    p.add_argument('--rate-hz', type=float, default=20.0); p.add_argument('--duration-s', type=float, default=10.0)
    p.add_argument('--note', default=None)


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


class Log:
    def __init__(self, path, header):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
        self.f = open(path, 'a'); self.n = 0; self.path = path
        self.emit({'kind': 'header', 'schema': SCHEMA, **header})

    def emit(self, rec):
        rec = {'seq': self.n, 'utc': utc_now(), 'mono_s': round(time.monotonic(), 6), **rec}; self.f.write(json.dumps(rec, default=str) + '\n'); self.f.flush(); self.n += 1; return rec

    def close(self):
        self.emit({'kind': 'footer', 'records': self.n}); self.f.close()


def header_for(args, purpose):
    return {'purpose': purpose, 'side': args.side, 'hardware_model': args.hardware_model, 'transport': args.transport,
            'transport_params': {'port': args.port, 'baud': args.baud, 'host': args.host, 'tcp_port': args.tcp_port, 'hand_id': args.hand_id, 'modbus_convention': args.modbus_convention},
            'authorization': Authorization(args.hardware_authorized, args.operator, args.allow_writes, args.note).as_record(),
            'manual': rm.MANUAL, 'addressing_conventions': rm.ADDRESSING_CONVENTIONS, 'host': platform.node(), 'note': args.note,
            'dry_run': args.transport == 'dry_run', 'values_are_installed_measurements': args.transport != 'dry_run'}


def open_from_args(args):
    auth = Authorization(args.hardware_authorized, args.operator, args.allow_writes, args.note)
    params = {'port': args.port, 'baud': args.baud, 'hand_id': args.hand_id, 'slave_id': args.hand_id, 'host': args.host, 'port_tcp': args.tcp_port, 'convention': args.modbus_convention}
    if args.transport == 'modbus_tcp':
        params = {'host': args.host, 'port': args.tcp_port, 'convention': args.modbus_convention}
    elif args.transport == 'modbus_rtu':
        params = {'port': args.port, 'baud': args.baud, 'slave_id': args.hand_id, 'convention': args.modbus_convention}
    elif args.transport == 'rs485_native':
        params = {'port': args.port, 'baud': args.baud, 'hand_id': args.hand_id}
    else:
        params = {}
    return auth, open_transport(args.transport, auth, **params)


def read_group(t, log, name, element_size=None):
    r = rm.register(name); size = 2 if r['element'] == 'short' else 1
    raw = t.read(r['address'], r['length_bytes']) if t.kind in ('dry_run', 'rs485_native') else t.read(r['address'], r['length_bytes'], element_size or size)
    rec = {'kind': 'read', 'group': name, 'address': r['address'], 'length_bytes': r['length_bytes'], 'raw_hex': raw.hex() if raw else None, 'decoded': rm.decode_group(name, raw) if raw and len(raw) == r['length_bytes'] else None}
    if raw is not None and len(raw) != r['length_bytes']:
        rec['error'] = 'short read: %d bytes' % len(raw)
    return log.emit(rec)


def write_group(t, log, auth, name, values):
    auth.check(t.kind, write=True)
    payload = rm.encode_group(name, values); r = rm.register(name)
    resp = t.write(r['address'], payload) if t.kind in ('dry_run', 'rs485_native') else t.write(r['address'], payload, 2 if r['element'] == 'short' else 1)
    return log.emit({'kind': 'write', 'group': name, 'address': r['address'], 'payload_hex': payload.hex(), 'values': values, 'response_hex': resp.hex() if resp else None, 'sent': t.kind != 'dry_run'})


def file_sha256(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()
