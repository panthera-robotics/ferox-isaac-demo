"""Author a Livox Mid-360 RTX lidar in Isaac Sim 5.1 from the twin contract.

WHY NOT A JSON PROFILE
----------------------
Isaac Sim ships JSON lidar profiles under
isaacsim.sensors.rtx/data/lidar_configs/, and that folder's README says the file
name is the `config` argument to IsaacSensorCreateRtxLidar. That is STALE. In
5.1, commands.py resolves `config` against SUPPORTED_LIDAR_CONFIGS -- a hardcoded
dict of USD ASSET PATHS -- and the JSON folder is only on
app.sensors.nv.lidar.profileBaseFolder, which extension.toml itself labels "for
(deprecated) camera-based Lidar".

A name that is not in that dict does NOT raise. Measured in this container:

    IsaacSensorCreateRtxLidar(path="bogus_lidar", parent="/World",
                              config="Livox_Mid360")
    -> carb.log_warn("Config 'Livox_Mid360' not found ...")
    -> a fully-formed default OmniLidar with all 82 omni:sensor attributes,
       silently RENAMED to /World/World_bogus_lidar

A warning in a log nobody greps, a sensor that looks right in the outliner, and
the wrong physics. That is precisely the failure class the campaign's rule 7
exists to stop, so this module never passes a custom config name.

WHAT IT DOES INSTEAD
--------------------
Create from "Example_Rotary" -- a real, supported asset -- then overwrite the
omni:sensor:Core:* attributes from the contract, then ASSERT every one of them
read back correctly. Nothing here is trusted because it was set; it is trusted
because it was read back.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

# Created from this known-good NVIDIA asset, then overwritten attribute by
# attribute. Its own defaults (128 emitters, +-15 deg, 1-200 m) are NOT the
# Mid-360's and every one of them is replaced below.
BASE_CONFIG = "Example_Rotary"

# Every per-emitter array on the prim. If we resize elevationDeg to N but leave
# these at the base asset's 128, the sensor is internally inconsistent and the
# renderer's behaviour is undefined -- so all of them get rebuilt at length N.
PER_EMITTER_FLOAT_ARRAYS = (
    "azimuthDeg", "distanceCorrectionM", "elevationDeg",
    "focalDistM", "focalSlope", "horOffsetM", "reportRateDiv", "vertOffsetM",
)
PER_EMITTER_UINT_ARRAYS = ("channelId", "fireTimeNs")


class LidarAuthoringError(RuntimeError):
    """A sensor that is not what the contract says. Always fatal."""


def mid360_grid(points_per_second: float, scan_rate_hz: float,
                n_emitters: int) -> Tuple[int, int]:
    """Factorise the point budget into (azimuth columns per rev, report rate Hz).

    points/s = reportRateBaseHz * numberOfEmitters
    reportRateBaseHz = columns per revolution * scanRateBaseHz

    Verified against a shipped profile: Ouster OS1 REV7 128ch 10 Hz 1024res sets
    reportRateBaseHz 10240 with 128 emitters = 1 310 720 points/s, that sensor's
    published figure.
    """
    points_per_rev = points_per_second / scan_rate_hz
    columns = points_per_rev / n_emitters
    if abs(columns - round(columns)) > 1e-9:
        raise LidarAuthoringError(
            f"point budget does not factorise: {points_per_second} pts/s at "
            f"{scan_rate_hz} Hz over {n_emitters} emitters gives {columns} "
            "columns per revolution, which is not an integer"
        )
    columns = int(round(columns))
    return columns, int(round(columns * scan_rate_hz))


def _azimuth_sector(az_min: float, az_max: float) -> Tuple[float, float]:
    """Contract azimuth bounds -> Isaac's (validStart, validEnd) in 0..360."""
    if (az_max - az_min) >= 359.999:
        return 0.0, 360.0
    return float(az_min % 360.0), float(az_max % 360.0)


def mid360_attributes(model_params: Dict[str, Any], n_emitters: int = 40) -> Dict[str, Any]:
    """Build the omni:sensor:* attribute set for a Mid-360 from contract values.

    `model_params` is the sensor's model_params block in <robot>_contract.yaml --
    datasheet figures only. The discretisation (n_emitters x columns) is this
    module's modelling choice and is a declared Class-C deviation: the real
    Mid-360 draws a NON-REPETITIVE ROSETTE whose coverage densifies with dwell
    time, and Isaac has no such scanType. We spend the same points per second on
    a uniform grid. Per-frame point count and the range/FOV envelope match; the
    spatial distribution does not.
    """
    elev_min, elev_max = model_params["elevation_deg"]
    az_min, az_max = model_params["azimuth_deg"]
    near, far = model_params["range_m"]
    scan_hz = float(model_params["scan_rate_hz"])
    pps = float(model_params["points_per_second"])

    columns, report_rate = mid360_grid(pps, scan_hz, n_emitters)

    step = (elev_max - elev_min) / (n_emitters - 1)
    elevations = [float(elev_min + i * step) for i in range(n_emitters)]
    # One vertical line array: all emitters share an azimuth column. Ouster's
    # per-beam azimuth offsets exist because its beams are physically staggered;
    # ours would be an invention.
    azimuths = [0.0] * n_emitters
    # Fire the column's emitters evenly across the column period so per-point
    # timestamps are ordered rather than identical.
    column_period_ns = 1e9 / report_rate
    fire_step = int(column_period_ns / n_emitters)
    fire_times = [i * fire_step for i in range(n_emitters)]

    attrs: Dict[str, Any] = {
        "omni:sensor:Core:scanType": "ROTARY",
        "omni:sensor:Core:rotationDirection": "CW",
        "omni:sensor:Core:rayType": "IDEALIZED",
        "omni:sensor:Core:intensityProcessing": "NORMALIZATION",
        "omni:sensor:Core:intensityMappingType": "LINEAR",
        # The real cloud is published in the SENSOR frame, raw and not
        # self-filtered -- which is exactly why p2l needs range_min 0.30.
        "omni:sensor:Core:outputFrameOfReference": "SENSOR",

        "omni:sensor:Core:nearRangeM": float(near),
        "omni:sensor:Core:farRangeM": float(far),
        "omni:sensor:Core:minDistBetweenEchosM": float(near),
        "omni:sensor:Core:rangeAccuracyM": float(model_params["range_accuracy_m"]),
        "omni:sensor:Core:rangeResolutionM": 0.001,

        # Azimuth sector. The contract states the datasheet FOV as [-180, 180]
        # (a full circle); Isaac wants a [start, end] sweep in 0..360. Mapping each
        # bound through % 360 independently would collapse both to 180 -- a
        # zero-width sector, i.e. a lidar that silently returns nothing.
        "omni:sensor:Core:validStartAzimuthDeg": _azimuth_sector(az_min, az_max)[0],
        "omni:sensor:Core:validEndAzimuthDeg": _azimuth_sector(az_min, az_max)[1],

        "omni:sensor:Core:azimuthErrorMean": 0.0,
        "omni:sensor:Core:elevationErrorMean": 0.0,
        # Datasheet says "angular precision < 0.15 deg" without naming a sigma;
        # read as ~3 sigma. ASSUMED -- flagged in the contract.
        "omni:sensor:Core:azimuthErrorStd": 0.05,
        "omni:sensor:Core:elevationErrorStd": 0.05,

        "omni:sensor:Core:waveLengthNm": 905.0,
        "omni:sensor:Core:maxReturns": 2,

        # uint attributes -- must be ints, not floats, or Set() silently no-ops.
        "omni:sensor:Core:scanRateBaseHz": int(round(scan_hz)),
        "omni:sensor:Core:reportRateBaseHz": int(report_rate),
        "omni:sensor:Core:numberOfEmitters": int(n_emitters),
        "omni:sensor:Core:numberOfChannels": int(n_emitters),
        # NOTE: there is no omni:sensor:Core:emitterStateCount attribute on the
        # prim. `emitterStateCount` exists in the JSON profile schema only; on the
        # prim the state count is structural (one emitterState:s001 namespace).
        # The read-back assertion caught this -- which is the whole point of it.

        # Campaign 4.3: tickRate must equal scanRateBaseHz or the published cloud
        # is a partial sweep instead of a full revolution.
        "omni:sensor:tickRate": float(scan_hz),

        # Identity, so `modelName` in the outliner and in any dump says what this
        # sensor actually is rather than "Example_Rotary".
        "omni:sensor:modelName": "Livox_Mid360",
        "omni:sensor:marketName": "Livox Mid-360",
        "omni:sensor:modelVendor": "Livox",
    }

    s = "omni:sensor:Core:emitterState:s001:"
    attrs[s + "elevationDeg"] = elevations
    attrs[s + "azimuthDeg"] = azimuths
    attrs[s + "fireTimeNs"] = fire_times
    attrs[s + "channelId"] = [i + 1 for i in range(n_emitters)]
    for name in PER_EMITTER_FLOAT_ARRAYS:
        attrs.setdefault(s + name, [0.0] * n_emitters)

    attrs["_derived"] = {
        "azimuth_columns_per_rev": columns,
        "azimuth_step_deg": 360.0 / columns,
        "elevation_step_deg": step,
        "points_per_second": report_rate * n_emitters,
        "points_per_revolution": columns * n_emitters,
    }
    return attrs


def _set_and_verify(prim, name: str, value: Any) -> None:
    """Set one attribute and read it back. A set that did not take is fatal."""
    if not prim.HasAttribute(name):
        raise LidarAuthoringError(
            f"{prim.GetPath()}: no attribute {name!r}. The base asset "
            f"{BASE_CONFIG!r} did not provide it -- Isaac Sim's sensor schema has "
            "changed and this module must be re-derived against the new one."
        )
    attr = prim.GetAttribute(name)
    attr.Set(value)
    got = attr.Get()

    if isinstance(value, (list, tuple)):
        got_list = list(got) if got is not None else []
        if len(got_list) != len(value):
            raise LidarAuthoringError(
                f"{prim.GetPath()}: {name} length {len(got_list)} != {len(value)}"
            )
        for i, (a, b) in enumerate(zip(got_list, value)):
            if abs(float(a) - float(b)) > 1e-4:
                raise LidarAuthoringError(
                    f"{prim.GetPath()}: {name}[{i}] read back {a!r}, set {b!r}"
                )
        return

    if isinstance(value, float):
        if got is None or abs(float(got) - value) > 1e-4:
            raise LidarAuthoringError(f"{prim.GetPath()}: {name} read back {got!r}, set {value!r}")
        return

    if got != value:
        # int -> uint round-trips exactly; a mismatch here means the Set was
        # rejected (commonly: a float written to a uint attribute).
        raise LidarAuthoringError(f"{prim.GetPath()}: {name} read back {got!r}, set {value!r}")


def create_mid360(parent_path: str, name: str, model_params: Dict[str, Any],
                  translation, orientation, n_emitters: int = 40):
    """Create a Mid-360 RTX lidar under `parent_path` and prove it is one.

    Returns (prim, derived) where `derived` carries the discretisation actually
    used, for the gate report. Raises LidarAuthoringError rather than returning a
    sensor that is not what the contract asked for.
    """
    import omni.kit.commands
    from pxr import Gf

    want_path = f"{parent_path}/{name}"
    _, prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=name,
        parent=parent_path,
        config=BASE_CONFIG,
        translation=tuple(float(v) for v in translation),
        orientation=Gf.Quatd(*[float(v) for v in orientation]),
    )
    if prim is None:
        raise LidarAuthoringError(
            f"IsaacSensorCreateRtxLidar returned no prim for {want_path} with "
            f"config={BASE_CONFIG!r}"
        )
    # The command RENAMES on failure (measured: /World/bogus_lidar became
    # /World/World_bogus_lidar) so an unexpected path means we are holding a
    # fallback sensor, not ours.
    got_path = str(prim.GetPath())
    if got_path != want_path:
        raise LidarAuthoringError(
            f"lidar landed at {got_path}, expected {want_path} -- Isaac renames on "
            "config-resolution failure, so this is a fallback sensor"
        )

    attrs = mid360_attributes(model_params, n_emitters=n_emitters)
    derived = attrs.pop("_derived")
    for attr_name, value in attrs.items():
        _set_and_verify(prim, attr_name, value)
    return prim, derived
