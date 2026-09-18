"""Uncertainty-aware comparison of ONE simulator quantity with ONE measured quantity, only when both are the same
well-defined quantity on the same datum and unit. The decision rule is the sprint G/H comparator's:
PASS iff |r| + U <= T; FAIL iff max(0, |r| - U) > T; else INDETERMINATE, with r = sim - measured, U the declared
applicable uncertainty bound and T the predeclared tolerance. Incompatible datums or units are refused, never
reconciled silently (a mesh-band thickness is not a caliper thickness at the pocket unless the datum says so).
"""
from __future__ import annotations

import math


class IncompatibleComparison(ValueError):
    pass


def compatible_compare(sim, measured, *, tolerance, uncertainty, unit_sim, unit_measured, datum_sim, datum_measured, quantity):
    for name, v in (('sim', sim), ('measured', measured), ('tolerance', tolerance), ('uncertainty', uncertainty)):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise IncompatibleComparison('%s: %s must be a finite number' % (quantity, name))
    if uncertainty < 0 or tolerance <= 0:
        raise IncompatibleComparison('%s: uncertainty must be >= 0 and tolerance > 0' % quantity)
    if unit_sim != unit_measured:
        raise IncompatibleComparison('%s: unit mismatch %r vs %r' % (quantity, unit_sim, unit_measured))
    if not datum_sim or not datum_measured or datum_sim.strip().lower() != datum_measured.strip().lower():
        raise IncompatibleComparison('%s: datum mismatch %r vs %r — not the same quantity' % (quantity, datum_sim, datum_measured))
    r = float(sim) - float(measured)
    if abs(r) + uncertainty <= tolerance:
        status = 'PASS'
    elif max(0.0, abs(r) - uncertainty) > tolerance:
        status = 'FAIL'
    else:
        status = 'INDETERMINATE'
    return {'quantity': quantity, 'unit': unit_sim, 'datum': datum_sim, 'residual': r, 'uncertainty': float(uncertainty), 'tolerance': float(tolerance), 'status': status,
            'rule': 'PASS iff |r|+U<=T; FAIL iff max(0,|r|-U)>T; else INDETERMINATE'}
