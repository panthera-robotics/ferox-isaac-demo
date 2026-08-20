"""Shared-memory transport between the Isaac side of the bridge and the DDS side.

Why two processes at all.  docs/twin/CAMPAIGN.md 4.5 specifies "a DDS bridge
process", and the rate arithmetic agrees: rt/lowstate has to go out at 1041.68 Hz
(contract `lowlevel.lowstate_publish_hz`) while physics steps at 500 Hz, so the
publisher cannot be driven by the physics callback.  Running it as a thread inside
Isaac's python would put a 1 kHz timer in contention with the GIL that Isaac's own
main loop holds -- and rate fidelity is not incidental here, it IS gate (c).  The
split also keeps cyclonedds out of Isaac's python entirely.

Why a seqlock rather than a mutex.  The writer must never block: on the sim side it
runs inside `on_physics_step`, where blocking stalls the simulation, and a stale
read is harmless because the next one is 1 ms away.  So each record is bracketed by
an odd/even sequence counter; the reader retries while it sees a torn or in-progress
write.  No lock is ever held across a container boundary, which is the other reason
-- a crashed bridge process cannot wedge the sim.

Layout is a fixed numpy structured dtype, not pickled dicts.  The upstream Unitree
reference (unitree_sim_isaaclab/dds) serialises a python dict per message; at
1041.68 Hz that is thousands of allocations a second for data that is a flat float
array.
"""

from __future__ import annotations

import numpy as np

# 35 is the LowState_/LowCmd_ motor array length; the G1 drives 0..28 and leaves
# 29..34 identically zero (contract `lowlevel.lowstate_fields.motor_state_dead`).
N_MOTOR = 35
# 40 hand joints total: Dex5-1P is 20 DoF per hand. Kept as one flat block in
# ISAAC ARTICULATION ORDER; the name mapping is the caller's job -- RULE-HAND-NAME
# forbids index-slicing the hand block, and this module deliberately does not do it.
N_HAND = 40
# 12 contact zones per hand (docs/twin/CAMPAIGN.md 4.4 item 4).
N_ZONE = 12

STATE_SHM = "ferox_g1_lowstate"
CMD_SHM = "ferox_g1_lowcmd"

STATE_DTYPE = np.dtype([
    ("seq", np.uint64),           # seqlock: odd while a write is in flight
    ("stamp_ns", np.uint64),      # CLOCK_MONOTONIC on the writer
    ("sim_time", np.float64),
    ("physics_step", np.uint64),
    ("q", np.float32, N_MOTOR),
    ("dq", np.float32, N_MOTOR),
    ("tau_est", np.float32, N_MOTOR),
    ("quat_wxyz", np.float32, 4),
    ("gyro", np.float32, 3),
    ("accel", np.float32, 3),
    ("rpy", np.float32, 3),
    ("hand_q", np.float32, N_HAND),
    ("hand_dq", np.float32, N_HAND),
    ("hand_tau", np.float32, N_HAND),
    ("contact", np.float32, (2, N_ZONE)),   # [left, right] x 12 zones, newtons
])

CMD_DTYPE = np.dtype([
    ("seq", np.uint64),
    # stamp_ns is the monotonic time at which the DDS side accepted a *fresh*
    # rt/lowcmd. The sim compares it against its own CLOCK_MONOTONIC to run the
    # 100 ms fail-closed watchdog. Both containers share a kernel, so the two
    # readings are the same clock -- this is the one cross-process time assumption
    # in the bridge and it is why --ipc=host is required rather than merely handy.
    ("stamp_ns", np.uint64),
    ("cmd_count", np.uint64),     # total accepted lowcmds; lets the sim see liveness
    ("mode_pr", np.uint8),
    ("mode_machine", np.uint8),
    ("q_d", np.float32, N_MOTOR),
    ("dq_d", np.float32, N_MOTOR),
    ("kp", np.float32, N_MOTOR),
    ("kd", np.float32, N_MOTOR),
    ("tau_ff", np.float32, N_MOTOR),
    ("hand_q_d", np.float32, N_HAND),
    ("hand_kp", np.float32, N_HAND),
    ("hand_kd", np.float32, N_HAND),
    ("hand_tau_ff", np.float32, N_HAND),
    ("hand_stamp_ns", np.uint64, 2),   # per-hand freshness, left/right
])


class SeqlockChannel:
    """One shared-memory record with seqlock write/read.

    `create=True` allocates; `create=False` attaches to an existing segment and
    raises FileNotFoundError if the other side is not up yet, which is the caller's
    signal to wait rather than to invent data.
    """

    def __init__(self, name: str, dtype: np.dtype, create: bool):
        from multiprocessing import shared_memory

        self._dtype = dtype
        self._owner = create
        if create:
            # A segment left behind by a killed process would otherwise make every
            # subsequent run fail with FileExistsError, so an existing one of the
            # right name is reclaimed. Size is checked below by the ndarray view.
            try:
                shared_memory.SharedMemory(name=name).unlink()
            except FileNotFoundError:
                pass
            self._shm = shared_memory.SharedMemory(
                name=name, create=True, size=dtype.itemsize)
        else:
            self._shm = shared_memory.SharedMemory(name=name)
        self._rec = np.ndarray((), dtype=dtype, buffer=self._shm.buf)
        if create:
            self._rec[...] = np.zeros((), dtype=dtype)

    # ---------------------------------------------------------------- writing

    def write(self, **fields) -> None:
        """Publish one record. Never blocks; readers retry around it."""
        rec = self._rec
        rec["seq"] = np.uint64(int(rec["seq"]) + 1)      # -> odd: write in flight
        for k, v in fields.items():
            rec[k] = v
        rec["seq"] = np.uint64(int(rec["seq"]) + 1)      # -> even: record is whole

    # ---------------------------------------------------------------- reading

    def read(self, retries: int = 8):
        """Return a private copy of a consistent record, or None if never written.

        Returns None rather than a zeroed record when seq == 0, because a zeroed
        LowState is a *plausible* robot state (all joints at origin) and silently
        publishing it would be indistinguishable from the sim genuinely being there.
        """
        rec = self._rec
        for _ in range(retries):
            s0 = int(rec["seq"])
            if s0 == 0:
                return None
            if s0 & 1:
                continue
            snap = rec.copy()
            if int(rec["seq"]) == s0:
                return snap
        return None

    def close(self) -> None:
        self._shm.close()
        if self._owner:
            try:
                self._shm.unlink()
            except FileNotFoundError:
                pass


def open_state(create: bool) -> SeqlockChannel:
    return SeqlockChannel(STATE_SHM, STATE_DTYPE, create)


def open_cmd(create: bool) -> SeqlockChannel:
    return SeqlockChannel(CMD_SHM, CMD_DTYPE, create)
