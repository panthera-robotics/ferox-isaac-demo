#!/usr/bin/env python3
"""Tactile logger for the installed RH56E2-T1: reads the tactile register block and stores RAW bytes with the declared
variant. The manual (2.6.21) describes two layouts for the RH56DFTP family (piezoresistive Table 56: 17 arrays, 16-bit points
0-4095, addresses 3000-5123; capacitive Tables 57-58: five 58-byte records at 3000-3290); which one an E2-T1 unit carries is
not stated, so --tactile-variant unknown reads the whole 3000-5123 span in chunks and decodes nothing. Declare the variant
only after the raw capture has been inspected. Default transport dry_run (no I/O).
"""
from __future__ import annotations

import argparse
import sys
import time

from .logging_common import Log, add_common_args, header_for, open_from_args
from .register_map import TACTILE_CAPACITIVE_NOTE, TACTILE_CAPACITIVE_TABLE58, TACTILE_PIEZORESISTIVE_NOTE, TACTILE_PIEZORESISTIVE_TABLE56

CHUNK = 120  # bytes per read for the unknown-variant sweep (below the FC03 125-register bound under any convention)


def blocks_for(variant):
    if variant == 'piezoresistive_table56':
        return [(n, a, L, {'rows': r, 'cols': c}) for n, a, r, c, L in TACTILE_PIEZORESISTIVE_TABLE56], TACTILE_PIEZORESISTIVE_NOTE
    if variant == 'capacitive_table58':
        return [(n, a, L, {'record': 'Table 57'}) for n, a, L in TACTILE_CAPACITIVE_TABLE58], TACTILE_CAPACITIVE_NOTE
    return [('raw_%d' % a, a, min(CHUNK, 5124 - a), {}) for a in range(3000, 5124, CHUNK)], 'variant unknown: raw sweep 3000-5123, nothing decoded'


def decode(variant, name, raw, meta):
    if variant == 'piezoresistive_table56' and raw is not None:
        pts = [int.from_bytes(raw[i:i + 2], 'little') for i in range(0, len(raw), 2)]
        return {'points': pts, 'rows': meta['rows'], 'cols': meta['cols'], 'max': max(pts), 'nonzero': sum(1 for v in pts if v)}
    return None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter); add_common_args(p)
    p.add_argument('--tactile-variant', default='unknown', choices=['unknown', 'piezoresistive_table56', 'capacitive_table58'])
    a = p.parse_args(argv); blocks, note = blocks_for(a.tactile_variant)
    auth, t = open_from_args(a)
    log = Log(a.out, {**header_for(a, 'tactile_readback'), 'tactile_variant': a.tactile_variant, 'variant_note': note, 'blocks': [(n, ad, L) for n, ad, L, _ in blocks]})
    period = 1.0 / a.rate_hz; t0 = time.monotonic(); i = 0
    try:
        while time.monotonic() - t0 <= a.duration_s:
            sweep_t = time.monotonic()
            for n, ad, L, meta in blocks:
                raw = t.read(ad, L) if t.kind in ('dry_run', 'rs485_native') else t.read(ad, L, 2)
                log.emit({'kind': 'read', 'block': n, 'address': ad, 'length_bytes': L, 'raw_hex': raw.hex() if raw else None, 'decoded': decode(a.tactile_variant, n, raw, meta)})
            log.emit({'kind': 'sweep', 'index': i, 'sweep_s': round(time.monotonic() - sweep_t, 4)})
            i += 1; dt = t0 + i * period - time.monotonic()
            if dt > 0: time.sleep(dt)
    finally:
        log.close(); t.close()
    print('wrote', log.path, 'records', log.n, 'sweeps', i, '| dry_run' if t.kind == 'dry_run' else '')


if __name__ == '__main__':
    sys.exit(main())
