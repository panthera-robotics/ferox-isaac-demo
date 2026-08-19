#!/usr/bin/env python3
"""Measure the publication rate of a rosbag2 topic from its receive timestamps.

MM3 test (c) was written as "rt/lowstate at 500 Hz +-2%".  The driver's own probe
of the real robot said 851.4 Hz and the DT bag says something else again, so the
number was measured here rather than picked -- see docs/mm/evidence/MM3/PREREQS.md.

Deliberately reads only the sqlite `timestamp` column and never deserializes a
message, so it runs anywhere python3 does.  That is the point: the rate question
is answerable without unitree_hg, while the *field semantics* half of test (c) is
not, and only the latter needs ferox/twin-lowlevel:humble.

    python3 scripts/mm3_lowstate_rate.py <bag.db3> [topic ...]
"""

from __future__ import annotations

import sqlite3
import sys

import numpy as np

DEFAULT_TOPICS = ["/lowstate"]


def analyse(db: str, topics: list[str]) -> int:
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    known = {name: (tid, typ) for tid, name, typ in
             con.execute("select id, name, type from topics")}

    missing = [t for t in topics if t not in known]
    if missing:
        print(f"topics not in bag: {missing}\navailable: {sorted(known)}", file=sys.stderr)
        return 2

    for name in topics:
        tid, typ = known[name]
        ts = np.array([r[0] for r in con.execute(
            "select timestamp from messages where topic_id=? order by timestamp", (tid,))],
            dtype=np.int64)
        if len(ts) < 2:
            print(f"=== {name} === too few messages ({len(ts)})")
            continue

        span = (ts[-1] - ts[0]) / 1e9
        rate = (len(ts) - 1) / span
        dt = np.diff(ts) / 1e6  # ms

        # Per-wall-second counts.  The final bucket is partial by construction and
        # would otherwise drag the min down by ~half the rate, so it is dropped.
        buckets = np.bincount((ts - ts[0]) // 1_000_000_000)[:-1]

        print(f"=== {name}  ({typ}) ===")
        print(f"  msgs={len(ts)}  span={span:.4f}s  RATE={rate:.3f} Hz  period={1e3/rate:.4f} ms")
        print(f"  dt ms: mean={dt.mean():.4f} std={dt.std():.4f} min={dt.min():.4f} max={dt.max():.4f}")
        print(f"  dt ms percentiles: " + "  ".join(
            f"p{p}={np.percentile(dt, p):.4f}" for p in (1, 25, 50, 75, 99)))
        # Near-zero gaps are the bag-side signature of the robot's duplicate-tick
        # bursts: two messages landing in the same control cycle (driver probe saw
        # 3.6% of messages repeat a tick).  Counted, not emulated -- Class C-25.
        burst = int((dt < 0.05).sum())
        print(f"  sub-50us gaps (duplicate-tick burst proxy): {burst} ({100*burst/len(dt):.2f}%)")
        print(f"  per-second counts: mean={buckets.mean():.2f} min={buckets.min()} "
              f"max={buckets.max()} std={buckets.std():.2f}  n={len(buckets)}")
        print(f"  gate band at +-2%: {0.98*rate:.2f} .. {1.02*rate:.2f} Hz")
        print()
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(analyse(sys.argv[1], sys.argv[2:] or DEFAULT_TOPICS))
