"""Transports for the RH56E2 register interface, gated so that nothing opens a serial port or a socket unless hardware work
has been explicitly authorized (three independent switches: the CLI flag, the environment variable, a named operator).

Kinds: dry_run (default; no I/O, every read returns None), rs485_native (manual Table 4/8 frames, needs pyserial),
modbus_tcp (stdlib sockets, FC 03/06/16, MBAP unit id 0xFF, default 192.168.11.210:6000), modbus_rtu (pyserial + CRC16).
The Modbus register-index convention is NOT stated in the manual and must be declared per unit (see register_map.
ADDRESSING_CONVENTIONS); the transport records the declared convention in every log record so a wrong guess is visible.
"""
from __future__ import annotations

import os
import socket
import struct
import time

from .register_map import ADDRESSING_CONVENTIONS, MANUAL  # noqa: F401  (documented conventions travel with the transport)

ENV_FLAG = 'PANTHERA_HARDWARE_AUTHORIZED'


class HardwareNotAuthorized(RuntimeError):
    """Raised before any port or socket is opened when the three authorization switches are not all set."""


class Authorization:
    def __init__(self, hardware_authorized=False, operator=None, allow_writes=False, note=None):
        self.hardware_authorized = bool(hardware_authorized); self.operator = operator; self.allow_writes = bool(allow_writes); self.note = note

    def check(self, kind, write=False):
        if kind == 'dry_run':
            return
        missing = []
        if not self.hardware_authorized: missing.append('--hardware-authorized flag')
        if os.environ.get(ENV_FLAG) != '1': missing.append('%s=1 in the environment' % ENV_FLAG)
        if not (self.operator and self.operator.strip()): missing.append('--operator <named human>')
        if write and not self.allow_writes: missing.append('--allow-writes (writes are refused by default)')
        if missing:
            raise HardwareNotAuthorized('hardware transport %r refused; missing: %s' % (kind, ', '.join(missing)))

    def as_record(self):
        return {'hardware_authorized': self.hardware_authorized, 'env_flag_set': os.environ.get(ENV_FLAG) == '1', 'operator': self.operator, 'allow_writes': self.allow_writes, 'note': self.note}


def _checksum(body):
    return sum(body) & 0xFF


def rs485_read_frame(hand_id, address, length):
    body = bytes([hand_id & 0xFF, 0x04, 0x11, address & 0xFF, (address >> 8) & 0xFF, length & 0xFF])
    return b'\xEB\x90' + body + bytes([_checksum(body)])


def rs485_write_frame(hand_id, address, payload):
    body = bytes([hand_id & 0xFF, (len(payload) + 3) & 0xFF, 0x12, address & 0xFF, (address >> 8) & 0xFF]) + bytes(payload)
    return b'\xEB\x90' + body + bytes([_checksum(body)])


def rs485_parse_response(frame, expect_cmd):
    """Return (hand_id, address, payload) from a 90 EB response; raises on header/checksum mismatch."""
    if len(frame) < 8 or frame[:2] != b'\x90\xEB':
        raise ValueError('bad response header %s' % frame[:2].hex())
    n = frame[3]; body = frame[2:2 + n + 3]
    if _checksum(frame[2:-1]) != frame[-1]:
        raise ValueError('checksum mismatch')
    if frame[4] != expect_cmd:
        raise ValueError('unexpected command byte 0x%02x' % frame[4])
    address = frame[5] | (frame[6] << 8)
    return frame[2], address, bytes(frame[7:-1])


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class DryRunTransport:
    kind = 'dry_run'

    def __init__(self, **kw):
        self.requests = []

    def read(self, address, length):
        self.requests.append(('read', address, length)); return None

    def write(self, address, payload):
        self.requests.append(('write', address, bytes(payload))); return None

    def close(self):
        pass


class Rs485NativeTransport:
    kind = 'rs485_native'

    def __init__(self, port, baud=115200, hand_id=1, timeout_s=0.2, **kw):
        import serial  # pyserial; optional dependency, imported only when a real transport is opened
        self.ser = serial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=timeout_s); self.hand_id = hand_id

    def read(self, address, length):
        self.ser.reset_input_buffer(); self.ser.write(rs485_read_frame(self.hand_id, address, length))
        frame = self.ser.read(8 + length)
        return rs485_parse_response(frame, 0x11)[2] if frame else None

    def write(self, address, payload):
        self.ser.reset_input_buffer(); self.ser.write(rs485_write_frame(self.hand_id, address, payload))
        frame = self.ser.read(9)
        return rs485_parse_response(frame, 0x12)[2] if frame else None

    def close(self):
        self.ser.close()


def _modbus_index(address, length, convention):
    """Map a manual byte address + byte length to (holding-register index, quantity) under a declared convention."""
    if convention == 'index_eq_byte_address_qty_words':          # candidate (a): one register per short, index = byte address
        return address, max(1, length // 2)
    if convention == 'index_eq_byte_address_qty_bytes':          # candidate (b)
        return address, length
    if convention == 'index_eq_half_byte_address':               # candidate (c)
        return address // 2, max(1, length // 2)
    raise ValueError('unknown Modbus addressing convention %r (declare one of the candidates and verify it)' % convention)


def _words_to_payload(words, element_size):
    """Modbus words are big-endian 16-bit; re-express them as the manual's little-endian byte payload."""
    if element_size == 1:
        return bytes(w & 0xFF for w in words)
    return b''.join(struct.pack('<h', struct.unpack('>h', struct.pack('>H', w))[0]) for w in words)


class ModbusTcpTransport:
    kind = 'modbus_tcp'

    def __init__(self, host='192.168.11.210', port=6000, convention='index_eq_byte_address_qty_words', timeout_s=0.5, unit_id=0xFF, **kw):
        self.sock = socket.create_connection((host, port), timeout=timeout_s); self.convention = convention; self.unit = unit_id; self.tid = 0

    def _txn(self, pdu):
        self.tid = (self.tid + 1) & 0xFFFF
        self.sock.sendall(struct.pack('>HHHB', self.tid, 0, len(pdu) + 1, self.unit) + pdu)
        hdr = self.sock.recv(7)
        if len(hdr) < 7: return None
        tid, proto, ln, unit = struct.unpack('>HHHB', hdr); body = b''
        while len(body) < ln - 1:
            chunk = self.sock.recv(ln - 1 - len(body))
            if not chunk: break
            body += chunk
        return body

    def read(self, address, length, element_size=2):
        idx, qty = _modbus_index(address, length, self.convention)
        body = self._txn(struct.pack('>BHH', 0x03, idx, qty))
        if not body or body[0] != 0x03: return None
        n = body[1]; words = struct.unpack('>%dH' % (n // 2), body[2:2 + n])
        return _words_to_payload(words, element_size)

    def write(self, address, payload, element_size=2):
        idx, qty = _modbus_index(address, len(payload), self.convention)
        if element_size == 2:
            words = [struct.unpack('>H', struct.pack('>h', struct.unpack('<h', payload[i:i + 2])[0]))[0] for i in range(0, len(payload), 2)]
        else:
            words = list(payload)
        pdu = struct.pack('>BHHB', 0x10, idx, len(words), 2 * len(words)) + b''.join(struct.pack('>H', w) for w in words)
        return self._txn(pdu)

    def close(self):
        self.sock.close()


class ModbusRtuTransport:
    kind = 'modbus_rtu'

    def __init__(self, port, baud=115200, slave_id=1, convention='index_eq_byte_address_qty_words', timeout_s=0.2, **kw):
        import serial
        self.ser = serial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=timeout_s); self.slave = slave_id; self.convention = convention

    def _txn(self, pdu, expect):
        frame = bytes([self.slave]) + pdu; frame += struct.pack('<H', crc16_modbus(frame))
        self.ser.reset_input_buffer(); self.ser.write(frame); resp = self.ser.read(expect)
        if len(resp) < 4 or crc16_modbus(resp[:-2]) != struct.unpack('<H', resp[-2:])[0]: return None
        return resp[1:-2]

    def read(self, address, length, element_size=2):
        idx, qty = _modbus_index(address, length, self.convention)
        body = self._txn(struct.pack('>BHH', 0x03, idx, qty), 5 + 2 * qty)
        if not body or body[0] != 0x03: return None
        words = struct.unpack('>%dH' % (body[1] // 2), body[2:2 + body[1]]); return _words_to_payload(words, element_size)

    def write(self, address, payload, element_size=2):
        idx, qty = _modbus_index(address, len(payload), self.convention)
        words = [struct.unpack('>H', struct.pack('>h', struct.unpack('<h', payload[i:i + 2])[0]))[0] for i in range(0, len(payload), 2)] if element_size == 2 else list(payload)
        pdu = struct.pack('>BHHB', 0x10, idx, len(words), 2 * len(words)) + b''.join(struct.pack('>H', w) for w in words)
        return self._txn(pdu, 8)

    def close(self):
        self.ser.close()


KINDS = {'dry_run': DryRunTransport, 'rs485_native': Rs485NativeTransport, 'modbus_tcp': ModbusTcpTransport, 'modbus_rtu': ModbusRtuTransport}


def open_transport(kind, authorization, **params):
    """The only way loggers obtain a transport: authorization is checked BEFORE any constructor runs."""
    if kind not in KINDS:
        raise ValueError('unknown transport kind %r' % kind)
    authorization.check(kind)
    t = KINDS[kind](**params); t.opened_mono_s = time.monotonic(); t.params = {k: v for k, v in params.items()}; return t
