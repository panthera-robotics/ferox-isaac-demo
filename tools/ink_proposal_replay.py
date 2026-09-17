#!/usr/bin/env python3
"""Replay a run's exported contact samples (contact_samples.csv) through the authoritative evaluator
and the PROPOSED pen-up specification; prints both, changes nothing. OWNER_REVIEW_REQUIRED."""
import csv, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaac' / 'twin'))
from inspire.contact_ink import IntendedStroke, ContactSample, MarkingRule
from inspire.contact_ink_proposal import evaluate_v2


def load(run):
    m = json.loads((run / 'metrics.json').read_text())
    strokes = [IntendedStroke(k, tuple(tuple(p) for p in v)) for k, v in m['intended_strokes'].items()]
    samples = []
    for r in csv.DictReader((run / 'contact_samples.csv').open()):
        f = lambda k: (None if r[k] in ('', 'None') else float(r[k]))
        b = lambda k: (None if r[k] in ('', 'None') else r[k] == 'True')
        pos = json.loads(r['nib_position_board_m']) if r['nib_position_board_m'].startswith('[') else tuple(float(x) for x in r['nib_position_board_m'].strip('()').split(','))
        samples.append(ContactSample(physics_sequence=int(r['physics_sequence']), physics_time_s=float(r['physics_time_s']), physics_dt_s=float(r['physics_dt_s']),
            nib_position_board_m=tuple(pos), nib_board_contact=b('nib_board_contact'), pen_down=r['pen_down'] == 'True', spring_compression_m=float(r['spring_compression_m']),
            stroke_id=(None if r['stroke_id'] in ('', 'None') else r['stroke_id']), segment_index=(None if r['segment_index'] in ('', 'None') else int(r['segment_index'])),
            reference_fraction=f('reference_fraction'), normal_force_n=f('normal_force_n'), normal_impulse_ns=f('normal_impulse_ns'),
            attachment_active=b('attachment_active'), fixture_support_active=b('fixture_support_active'), holder_bottomed_out=b('holder_bottomed_out')))
    scored = [s for s in samples if not s.fixture_support_active]
    return strokes, scored, m


def main(run):
    run = Path(run)
    strokes, samples, m = load(run)
    rule = MarkingRule(spring_travel_m=m['contact_writing']['evaluation']['marking_rule']['spring_travel_m'], maximum_sample_interval_s=.0055)
    report = evaluate_v2(strokes, samples, rule)
    out = {'run': run.name, 'old': {k: report.get(k) for k in ('accepted', 'failure_reasons', 'path_error_p95_m', 'path_error_max_m', 'coverage_fraction', 'pen_up_leakage_extent_m', 'unintended_contact_bridge_extent_m')}, 'proposal': report['proposal']}
    (run / 'contact_ink_proposal_replay.json').write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main(sys.argv[1])
