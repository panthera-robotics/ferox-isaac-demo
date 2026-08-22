#!/bin/bash
# C-39 task 1, "Falls" branch: one simulator property per run, re-testing each.
#
# The A/B put us here -- the REFERENCE body falls in our simulator too
# (evidence/C39/AB_ASSET_VERDICT.md) -- so the remaining search space is our own PhysX
# configuration. Runs the list in the brief's order and STOPS at the first that stands
# SONIC, recording a sha256 of the runner and the exact environment for that row.
#
# Sole-occupancy is asserted before every run: Mohammed's rule is that any A/B taken
# while a second claude process exists is VOID, so a run that cannot prove it is the
# only one is not worth its 11 minutes.
set -u
DEMO_DIR=/root/panthera/ferox-isaac-demo
OUT=$DEMO_DIR/docs/mm/evidence/C39/bisect
mkdir -p "$OUT"
SUMMARY=$OUT/SUMMARY.tsv
[ -f "$SUMMARY" ] || printf 'label\tenv\treleased\tbase_z\tpitch_deg\tverdict\n' > "$SUMMARY"

run_one() {
  local label="$1" side="$2"; shift 2
  local n
  n=$(pgrep -fc 'native-binary/claude --output-format' 2>/dev/null || echo 0)
  if [ "$n" -gt 1 ]; then
    echo "[bisect] ABORT $label: $n claude processes -- result would be VOID"
    printf '%s\t%s\tVOID\t\t\tSECOND_INSTANCE\n' "$label" "$*" >> "$SUMMARY"
    return 1
  fi
  echo "[bisect] === $label ($side) :: $* ==="
  env WANT_T=75 "$@" bash "$DEMO_DIR/scripts/c39_ab_asset.sh" "$side" 1500 \
      > "/root/panthera/logs/bisect_${label}.log" 2>&1
  local v="$DEMO_DIR/docs/mm/evidence/C39/ab/${side}_verdict.txt"
  cp "$v" "$OUT/${label}_verdict.txt" 2>/dev/null
  cp "$DEMO_DIR/docs/mm/evidence/C39/ab/${side}_sim.log" "$OUT/${label}_sim.log" 2>/dev/null
  local rel bz pd res
  rel=$(grep -oP 'rig released at: \K.*' "$v" 2>/dev/null | head -1)
  bz=$(grep -oP 'final: .*base_z=\K[-+0-9.]+' "$v" 2>/dev/null | head -1)
  pd=$(grep -oP 'pitch=[-+0-9.]+ \(\K[-+0-9.]+' "$v" 2>/dev/null | head -1)
  res=$(grep -oP '^RESULT: \K.*' "$v" 2>/dev/null | head -1)
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$label" "$*" "${rel:-?}" "${bz:-?}" "${pd:-?}" "${res:-NORESULT}" >> "$SUMMARY"
  echo "[bisect] $label -> ${res:-NORESULT} (base_z ${bz:-?}, pitch ${pd:-?} deg)"
  [ "$res" = "STANDS" ] && return 0 || return 2
}

# Controls first -- both were taken while a second instance existed and are VOID.
run_one baseline_twin  twin_bare && { echo "[bisect] baseline STANDS?! stopping"; exit 0; }
run_one baseline_ref   ref       && { echo "[bisect] reference STANDS -- asset delta, stopping"; exit 0; }

# The brief's list, in order, one variable per run.
run_one solver_iters   twin_bare G1_SOLVER_ITERS=64,64            && exit 0
run_one friction_mult  twin_bare G1_PHYSX_TWEAKS=friction_combine=multiply && exit 0
run_one contact_off    twin_bare G1_PHYSX_TWEAKS=contact_offset=0.002,rest_offset=0.0 && exit 0
run_one depen_vel      twin_bare G1_PHYSX_TWEAKS=max_depen_vel=1.0 && exit 0
run_one self_coll      twin_bare G1_PHYSX_TWEAKS=self_collision=1  && exit 0
run_one dt_200hz       twin_bare G1_PHYSICS_HZ=200                 && exit 0
run_one implicit_drive twin_bare G1_LL_PD=implicit                 && exit 0

echo "[bisect] NONE of the list stands SONIC. Park it per the brief."
sha256sum "$DEMO_DIR/scripts/c39_ab_asset.sh" "$DEMO_DIR/isaac/run.py" \
          "$DEMO_DIR/isaac/twin/lowlevel_bridge/sim_side.py" > "$OUT/sha256.txt"
column -t -s $'\t' "$SUMMARY"
