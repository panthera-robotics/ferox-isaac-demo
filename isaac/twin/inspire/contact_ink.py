"""Evaluate and export measured nib/board contact traces using only the standard library.

No desired-text renderer or robot/driver dependency is present. Contact flags and force
must be supplied by the simulator's actual nib/board contact query. The evaluator cannot
attest their origin. SVG is a measured-trace plumbing artifact, never a grasp/standing
certificate. Default thresholds are proposed simulation targets, not hardware limits.

The board frame is metres, x/y in its plane and z normal to the marking surface. Positive
normal force is compressive and is the physics-step mean. Impulse, when supplied, is N s
for that same step. Ink is piecewise linear only between consecutive recorded physics
samples; missing samples invalidate acceptance and break the visible line.
"""
from __future__ import annotations

import csv
from dataclasses import MISSING, asdict, dataclass, fields
import html
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


class InvalidTrace(ValueError):
    """A trace/schema cannot support the requested measurement."""


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _point(value, width):
    if (not isinstance(value, (tuple, list)) or len(value) != width
            or not all(_finite(v) and abs(v) <= 1000 for v in value)):
        raise InvalidTrace(f"expected {width} finite board coordinates within 1 km")
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class IntendedStroke:
    stroke_id: str
    points_board_m: tuple[tuple[float, float], ...]

    def __post_init__(self):
        if not isinstance(self.stroke_id, str) or not self.stroke_id:
            raise InvalidTrace("stroke_id must be a nonempty string")
        points = tuple(_point(p, 2) for p in self.points_board_m)
        if len(points) < 2 or any(a == b for a, b in zip(points, points[1:])):
            raise InvalidTrace("each intended stroke needs nonzero ordered segments")
        object.__setattr__(self, "points_board_m", points)


@dataclass(frozen=True)
class ContactSample:
    physics_sequence: int
    physics_time_s: float
    physics_dt_s: float
    nib_position_board_m: tuple[float, float, float]
    nib_board_contact: bool | None
    pen_down: bool
    spring_compression_m: float
    stroke_id: str | None = None
    segment_index: int | None = None
    reference_fraction: float | None = None
    normal_force_n: float | None = None
    normal_impulse_ns: float | None = None
    attachment_active: bool | None = None
    fixture_support_active: bool | None = None
    holder_bottomed_out: bool | None = None

    def __post_init__(self):
        if type(self.physics_sequence) is not int or self.physics_sequence < 0:
            raise InvalidTrace("physics sequence must be a nonnegative integer")
        if not _finite(self.physics_time_s) or self.physics_time_s < 0:
            raise InvalidTrace("physics time must be finite and nonnegative")
        if not _finite(self.physics_dt_s) or self.physics_dt_s <= 0:
            raise InvalidTrace("physics dt must be finite and positive")
        object.__setattr__(self, "nib_position_board_m", _point(self.nib_position_board_m, 3))
        if self.nib_board_contact is not None and type(self.nib_board_contact) is not bool:
            raise InvalidTrace("nib contact must be explicit true/false or unavailable None")
        if type(self.pen_down) is not bool:
            raise InvalidTrace("pen-down flag must be boolean")
        for value in (self.attachment_active, self.fixture_support_active, self.holder_bottomed_out):
            if value is not None and type(value) is not bool:
                raise InvalidTrace("qualification flags must be booleans or unavailable None")
        if not _finite(self.spring_compression_m) or self.spring_compression_m < 0:
            raise InvalidTrace("spring compression must be finite and nonnegative")
        for value in (self.normal_force_n, self.normal_impulse_ns):
            if value is not None and (not _finite(value) or value < 0):
                raise InvalidTrace("normal force/impulse must be finite and nonnegative")
        if self.normal_impulse_ns is not None and not math.isfinite(self.normal_impulse_ns / self.physics_dt_s):
            raise InvalidTrace("impulse/dt produced non-finite force")
        if self.normal_force_n is not None and self.normal_impulse_ns is not None:
            impulse_force = self.normal_impulse_ns / self.physics_dt_s
            if not math.isclose(self.normal_force_n, impulse_force, rel_tol=1e-5, abs_tol=1e-8):
                raise InvalidTrace("step-mean normal force disagrees with impulse/dt")
        if self.pen_down:
            if (not isinstance(self.stroke_id, str) or not self.stroke_id
                    or type(self.segment_index) is not int or self.segment_index < 0
                    or not _finite(self.reference_fraction)
                    or not 0 <= self.reference_fraction <= 1):
                raise InvalidTrace("pen-down needs stroke ID, segment index and reference fraction")

    @property
    def force_n(self):
        if self.normal_force_n is not None:
            return self.normal_force_n
        if self.normal_impulse_ns is not None:
            return self.normal_impulse_ns / self.physics_dt_s
        return None


@dataclass(frozen=True)
class MarkingRule:
    coverage_tolerance_m: float = 0.003
    minimum_segment_coverage: float = 0.95
    maximum_path_p95_m: float = 0.003
    maximum_path_error_m: float = 0.010
    contact_plane_tolerance_m: float = 0.001
    minimum_force_n: float = 0.05
    maximum_force_n: float = 20.0
    spring_travel_m: float = 0.020
    maximum_sample_interval_s: float = 0.020
    ink_width_m: float = 0.0005
    allowed_pen_up_leakage_m: float = 0.0

    def __post_init__(self):
        if any(not _finite(getattr(self, f.name)) for f in fields(self)):
            raise InvalidTrace("every marking threshold must be finite")
        positive = (self.coverage_tolerance_m, self.maximum_path_p95_m,
                    self.maximum_path_error_m, self.contact_plane_tolerance_m,
                    self.minimum_force_n, self.maximum_force_n, self.spring_travel_m,
                    self.maximum_sample_interval_s, self.ink_width_m)
        if (any(v <= 0 for v in positive) or not 0 < self.minimum_segment_coverage <= 1
                or self.allowed_pen_up_leakage_m < 0 or self.minimum_force_n >= self.maximum_force_n
                or self.maximum_path_p95_m > self.maximum_path_error_m):
            raise InvalidTrace("marking thresholds must define a positive ordered envelope")


def samples_from_columns(columns: Mapping[str, Sequence]) -> tuple[ContactSample, ...]:
    """Validate recording widths before assembling rows; never silently zip/truncate."""
    valid_names = {f.name for f in fields(ContactSample)}
    required = {f.name for f in fields(ContactSample) if f.default is MISSING}
    if not isinstance(columns, Mapping) or not required <= set(columns) <= valid_names:
        raise InvalidTrace("contact columns have missing required or unknown fields")
    try:
        widths = {len(values) for values in columns.values()}
    except TypeError as exc:
        raise InvalidTrace("contact columns must be sequences") from exc
    if len(widths) != 1:
        raise InvalidTrace("contact column lengths differ")
    return tuple(ContactSample(**{name: values[i] for name, values in columns.items()})
                 for i in range(widths.pop()))


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _intersect(a, b):
    lo, hi = max(a[0], b[0]), min(a[1], b[1])
    return (lo, hi) if lo <= hi else None


def _linear_interval(intercept, slope, low, high):
    if abs(slope) < 1e-15:
        return (0.0, 1.0) if low <= intercept <= high else None
    a, b = (low - intercept) / slope, (high - intercept) / slope
    return _intersect((min(a, b), max(a, b)), (0.0, 1.0))


def _quadratic_interval(offset, direction, radius):
    a, b = _dot(direction, direction), 2 * _dot(offset, direction)
    c = _dot(offset, offset) - radius * radius
    if a < 1e-24:
        return (0.0, 1.0) if c <= 1e-18 else None
    discriminant = b * b - 4 * a * c
    if discriminant < -1e-18:
        return None
    root = math.sqrt(max(0.0, discriminant))
    return _intersect(((-b - root) / (2 * a), (-b + root) / (2 * a)), (0.0, 1.0))


def _capsule_intervals(a, b, u, v, radius):
    """Intervals on intended A→B inside the tolerance capsule of measured U→V."""
    d, e, w = _sub(b, a), _sub(v, u), _sub(a, u)
    ee = _dot(e, e)
    if ee < 1e-24:
        interval = _quadratic_interval(w, d, radius)
        return [interval] if interval else []
    s0, sd = _dot(w, e) / ee, _dot(d, e) / ee
    intervals = []
    regions = [(-math.inf, 0.0, w, d),
               (0.0, 1.0, (w[0] - s0 * e[0], w[1] - s0 * e[1]),
                (d[0] - sd * e[0], d[1] - sd * e[1])),
               (1.0, math.inf, _sub(a, v), d)]
    for low, high, offset, direction in regions:
        region = _linear_interval(s0, sd, low, high)
        inside = _quadratic_interval(offset, direction, radius)
        if region is not None and inside is not None:
            intersection = _intersect(region, inside)
            if intersection is not None:
                intervals.append(intersection)
    return intervals


def _union_length(intervals):
    end, total = 0.0, 0.0
    for lo, hi in sorted(intervals):
        total += max(0.0, hi - max(lo, end))
        end = max(end, hi)
    return min(1.0, total)


def _percentile95(values):
    # Nearest-rank quantile, specified so independent implementations agree.
    return sorted(values)[math.ceil(0.95 * len(values)) - 1] if values else None


def evaluate(strokes, samples, rule=MarkingRule()):
    """Return a JSON-safe report. Invalid evidence and failed targets remain distinct.

    Coverage is geometric per intended ordered segment. Both individual contact dots and
    consecutive contact lines contribute tolerance capsules. A wrong-order segment earns
    no coverage even if its geometry is exactly right. Path error compares measured XY
    with the *synchronized reference fraction of that segment*, never another letter.
    Pen-up contact still deposits ink and is measured as leakage, not suppressed.
    """
    strokes, samples = tuple(strokes), tuple(samples)
    if not strokes or any(type(s) is not IntendedStroke for s in strokes):
        raise InvalidTrace("at least one validated intended stroke is required")
    if any(type(s) is not ContactSample for s in samples) or type(rule) is not MarkingRule:
        raise InvalidTrace("validated samples and marking rule required")
    if len({s.stroke_id for s in strokes}) != len(strokes):
        raise InvalidTrace("intended stroke IDs must be unique")
    segments = {}
    for stroke in strokes:
        for index, (a, b) in enumerate(zip(stroke.points_board_m, stroke.points_board_m[1:])):
            segments[(stroke.stroke_id, index)] = dict(a=a, b=b, rank=len(segments), intervals=[])
    issues, invalid = set(), set()
    if not samples:
        invalid.add("missing_contact_trace")
    errors, paths = [], []
    endpoint_errors = {f'{s.stroke_id}:{endpoint}': None for s in strokes for endpoint in ('start', 'end')}
    corner_errors = {f'{s.stroke_id}:corner{i}': None for s in strokes for i in range(1, len(s.points_board_m)-1)}
    previous = None
    previous_mark = False
    previous_key = None
    previous_ordered = False
    last_rank, last_fraction = -1, -1.0
    marked_count, pen_up_count, pen_up_extent = 0, 0, 0.0
    force_values, max_compression = [], 0.0
    stroke_map = {s.stroke_id: s for s in strokes}
    for sample in samples:
        contiguous = False
        if previous is not None:
            if (sample.physics_sequence <= previous.physics_sequence
                    or sample.physics_time_s <= previous.physics_time_s):
                raise InvalidTrace("repeated/reordered physics sample or reset requires a new trace")
            contiguous = (sample.physics_sequence == previous.physics_sequence + 1
                          and sample.physics_time_s - previous.physics_time_s <= rule.maximum_sample_interval_s)
            if not contiguous:
                invalid.add("missing_physics_samples")
            if contiguous and not math.isclose(sample.physics_time_s - previous.physics_time_s,
                                               sample.physics_dt_s, rel_tol=1e-6, abs_tol=1e-9):
                invalid.add("physics_step_time_mismatch")
                contiguous = False
        if sample.nib_board_contact is None:
            invalid.add("contact_observation_unavailable")
        if sample.attachment_active:
            issues.add("attachment_active")
        if sample.fixture_support_active:
            issues.add("fixture_support_active")
        for status, reason in ((sample.attachment_active, "attachment_status_unavailable"),
                               (sample.fixture_support_active, "support_status_unavailable"),
                               (sample.holder_bottomed_out, "holder_stop_status_unavailable")):
            if status is None:
                invalid.add(reason)
        bottomed = sample.holder_bottomed_out or sample.spring_compression_m >= rule.spring_travel_m
        if bottomed:
            issues.add("holder_bottomed_out")
        max_compression = max(max_compression, sample.spring_compression_m)
        force = sample.force_n
        if force is not None:
            force_values.append(force)
        if sample.nib_board_contact is True and force is None:
            invalid.add("contact_force_unavailable")
        if sample.nib_board_contact is False and force is not None and force > 0:
            invalid.add("contact_flag_force_disagree")
        if sample.nib_board_contact is True and force is not None and not rule.minimum_force_n <= force <= rule.maximum_force_n:
            issues.add("normal_force_outside_window")
        if sample.nib_board_contact is True and abs(sample.nib_position_board_m[2]) > rule.contact_plane_tolerance_m:
            invalid.add("contact_point_off_board_plane")
        marked = (sample.nib_board_contact is True and force is not None
                  and rule.minimum_force_n <= force <= rule.maximum_force_n
                  and abs(sample.nib_position_board_m[2]) <= rule.contact_plane_tolerance_m
                  and not bottomed)
        xy = sample.nib_position_board_m[:2]
        key, ordered = None, False
        if sample.pen_down:
            key = (sample.stroke_id, sample.segment_index)
            if key not in segments:
                raise InvalidTrace("sample references an unknown intended stroke segment")
            segment = segments[key]
            rank, fraction = segment['rank'], sample.reference_fraction
            ordered = (rank == last_rank and fraction >= last_fraction) or rank == last_rank + 1
            if not ordered:
                issues.add("stroke_order_violation")
            else:
                last_rank, last_fraction = rank, fraction
            a, b = segment['a'], segment['b']
            reference = (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))
            error = math.dist(xy, reference)
            errors.append(error)
            last_index = len(stroke_map[sample.stroke_id].points_board_m) - 2
            if fraction == 0.0:
                target = endpoint_errors if sample.segment_index == 0 else corner_errors
                label = f'{sample.stroke_id}:start' if sample.segment_index == 0 else f'{sample.stroke_id}:corner{sample.segment_index}'
                target[label] = max(target.get(label) or 0.0, error)
            if fraction == 1.0:
                target = endpoint_errors if sample.segment_index == last_index else corner_errors
                label = f'{sample.stroke_id}:end' if sample.segment_index == last_index else f'{sample.stroke_id}:corner{sample.segment_index + 1}'
                target[label] = max(target.get(label) or 0.0, error)
        if marked:
            marked_count += 1
            if previous_mark and contiguous:
                paths[-1].append(xy)
            else:
                paths.append([xy])
            if not sample.pen_down:
                pen_up_count += 1
                if not (previous_mark and contiguous and not previous.pen_down):
                    pen_up_extent += rule.ink_width_m
                if previous_mark and contiguous:
                    pen_up_extent += math.dist(previous.nib_position_board_m[:2], xy)
            elif previous_mark and contiguous and not previous.pen_down:
                # Conservatively include the contact transition back into pen-down.
                pen_up_extent += math.dist(previous.nib_position_board_m[:2], xy)
            if key is not None and ordered:
                segment = segments[key]
                segment['intervals'].extend(_capsule_intervals(segment['a'], segment['b'], xy, xy,
                                                               rule.coverage_tolerance_m))
                if previous_mark and contiguous and previous_key == key and previous_ordered:
                    segment['intervals'].extend(_capsule_intervals(segment['a'], segment['b'],
                        previous.nib_position_board_m[:2], xy, rule.coverage_tolerance_m))
        previous, previous_mark, previous_key, previous_ordered = sample, marked, key, ordered
    coverage, covered_length, intended_length = [], 0.0, 0.0
    for (stroke_id, index), segment in segments.items():
        fraction = _union_length(segment['intervals'])
        length = math.dist(segment['a'], segment['b'])
        coverage.append(dict(stroke_id=stroke_id, segment_index=index,
                             intended_length_m=length, covered_length_m=length*fraction,
                             coverage_fraction=fraction))
        intended_length += length; covered_length += length * fraction
        if fraction < rule.minimum_segment_coverage:
            issues.add("incomplete_segment_coverage")
    p95, maximum = _percentile95(errors), max(errors) if errors else None
    if not errors:
        issues.add("no_pen_down_samples")
    elif p95 > rule.maximum_path_p95_m or maximum > rule.maximum_path_error_m:
        issues.add("path_error_exceeds_target")
    if pen_up_extent > rule.allowed_pen_up_leakage_m:
        issues.add("pen_up_ink_leakage")
    return dict(schema_version=1, artifact_kind="measured_contact_trace_plumbing",
                physics_grasp_or_standing_certified=False, valid=not invalid,
                accepted=not invalid and not issues, invalid_reasons=sorted(invalid),
                failure_reasons=sorted(issues), samples=len(samples), marked_samples=marked_count,
                intended_length_m=intended_length, covered_length_m=covered_length,
                coverage_fraction=covered_length/intended_length, segments=coverage,
                path_error_p95_m=p95, path_error_max_m=maximum,
                path_error_quantile="nearest_rank", endpoint_errors_m=endpoint_errors,
                corner_errors_m=corner_errors, pen_up_marked_samples=pen_up_count,
                pen_up_leakage_extent_m=pen_up_extent,
                normal_force_min_n=min(force_values) if force_values else None,
                normal_force_max_n=max(force_values) if force_values else None,
                spring_compression_max_m=max_compression if samples else None,
                marking_rule=asdict(rule), ink_paths_board_m=paths)


def export_svg(report, destination):
    """Render only measured eligible contact paths already present in the report."""
    points = [p for path in report['ink_paths_board_m'] for p in path]
    width = report['marking_rule']['ink_width_m']
    if points:
        xmin, xmax = min(p[0] for p in points)-width, max(p[0] for p in points)+width
        ymin, ymax = min(p[1] for p in points)-width, max(p[1] for p in points)+width
    else:
        xmin, xmax, ymin, ymax = 0.0, 0.1, 0.0, 0.1
    desc = html.escape("Measured nib/board contact trace; plumbing artifact only. "
                       "No physical grasp or standing qualification is certified.")
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{xmin} {-ymax} {xmax-xmin} {ymax-ymin}">',
             f'<desc>{desc}</desc>', '<g fill="none" stroke="#111" stroke-linecap="round" stroke-linejoin="round"',
             f' stroke-width="{width}">']
    for path in report['ink_paths_board_m']:
        if len(path) == 1:
            x, y = path[0]
            parts.append(f'<circle cx="{x}" cy="{-y}" r="{width/2}" fill="#111" stroke="none"/>')
        else:
            coords = ' '.join(f'{x},{-y}' for x,y in path)
            parts.append(f'<polyline points="{coords}"/>')
    parts.extend(['</g>', '</svg>'])
    Path(destination).write_text('\n'.join(parts)+'\n', encoding='utf-8')


def export_csv(samples, destination):
    """Export raw contact evidence, preserving None as empty instead of fabricated zero."""
    names = [f.name for f in fields(ContactSample)]
    with Path(destination).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        for sample in samples:
            row = asdict(sample)
            row['nib_position_board_m'] = json.dumps(row['nib_position_board_m'], allow_nan=False)
            writer.writerow(row)
