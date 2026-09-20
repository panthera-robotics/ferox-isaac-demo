# Updating the fidelity ledger, the six-axis hand contract, the native-to-radian map, the tactile schema and the donor-vs-installed comparison

Preconditions: filled sheets (installed_measurement, measured_utc, operator, evidence with sha256), the actuator/tactile logs, the
donor profile of the same side (`python -m hand_fidelity.donor_profile <donor URDF> --side <side> --out <side>_profile.json`).

1. **Comparison first**: `python -m hand_fidelity.rh56e2.compare_installed <sheets dir> --donor-profile <profile> --ledger <FIDELITY_LEDGER_RH56E2 json> --out-prefix DONOR_VS_INSTALLED`.
   Rows use the ledger's predeclared tolerances (uncertainty-aware PASS/FAIL/INDETERMINATE rule); anything unmeasured stays NOT_MEASURED;
   no similarity percentage exists. The label is the last line.
2. **Fidelity ledger**: for each ledger property with an installed value, set `installed_hardware_agreement` from the row status and add
   `installed_value`, `installed_uncertainty`, `measured_utc`, `operator`, `evidence` (file names + sha256; private). Properties the
   sheets add (hinge positions, pads, COM, actuator direction/deadband/speed response, tactile) are appended as new rows with the same
   columns. `overall_real_hand_similarity_percent` stays null.
3. **Six-axis hand contract**: `python -m hand_fidelity.rh56e2.conversion_template actuator_endpoints_sheet.json --out native_to_radian_map.json`.
   When COMPLETE, its `six_axis_convention` block (`inspire_e2_installed_<side>_v1`: native axis order, open 1000 / closed 0, per-axis
   measured endpoints, evidence class MEASURED) is added to `tools/hand_fidelity/command_semantics.py` CONVENTIONS with the sheet's
   evidence; the runtime adapter keeps the closure-preserving import (never radian identity across datums).
4. **Native-to-radian map**: the per-axis affine maps in `native_to_radian_map.json` (counts → Figure 5 degrees, counts → donor joint
   radians) become the declared conversion for the installed hand; the thumb rotation keeps its own scale and sense.
5. **Tactile schema**: from tactile_sheet.json (variant, arrays with address/rows/cols/orientation, units, rate) write the schema the
   twin publishes for the installed hand; a variant not verified by a touch test stays `unknown` and the twin publishes nothing.
6. **Donor-vs-installed comparison**: file DONOR_VS_INSTALLED.json/.md next to the ledger; the ledger's "installed" column cites it.
7. **Final label**: only `compare_installed` prints `RH56E2_INSTALLED_CALIBRATION_COMPLETE`, and only when every required group
   (palm/finger geometry, hinge positions, pad dimensions, open/closed endpoints, total mass, approximate COM, six actuator native
   min/max, direction, command/readback units, speed response, deadband, firmware/transport, thumb rotation direction/scale, tactile
   channel count/layout/units/rate) is measured with an evidence reference on every sheet. Otherwise the label is
   `RH56E2_INSTALLED_CALIBRATION_INCOMPLETE` with the missing list, and the twin's hand stays PROVISIONAL (donor).
