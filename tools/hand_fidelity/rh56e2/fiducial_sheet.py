#!/usr/bin/env python3
"""Printable A4 scale/fiducial sheet (SVG, millimetre units) for the RH56E2 photo guide: 10 mm grid, mm rulers on two edges,
a 100 mm print-scale check bar, a datum cross, and four fiducial markers with known centres and size (ArUco DICT_4X4_50 ids 0-3
when OpenCV is importable; otherwise a nested-square fallback with the same centres so the geometry stays usable). The
marker ids/centres are written next to the SVG as JSON. usage: fiducial_sheet.py --out fiducial_A4.svg"""
from __future__ import annotations

import argparse
import json
import sys

W, H = 210.0, 297.0          # A4 portrait, mm
MARK = 30.0                  # marker side, mm
CENTRES = {0: (30.0, 30.0), 1: (180.0, 30.0), 2: (30.0, 267.0), 3: (180.0, 267.0)}


def aruco_bits(marker_id, dict_name='DICT_4X4_50'):
    try:
        import cv2
        import numpy as np
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        img = cv2.aruco.generateImageMarker(d, marker_id, 6) if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(d, marker_id, 6)
        return [[int(img[r, c] == 0) for c in range(6)] for r in range(6)], dict_name
    except Exception:
        return None, None


def marker_svg(cx, cy, bits, marker_id):
    x0, y0 = cx - MARK / 2, cy - MARK / 2; out = ['<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="white"/>' % (x0, y0, MARK, MARK)]
    if bits:
        cell = MARK / 6
        for r in range(6):
            for c in range(6):
                if bits[r][c]: out.append('<rect x="%.3f" y="%.3f" width="%.3f" height="%.3f" fill="black"/>' % (x0 + c * cell, y0 + r * cell, cell + 0.01, cell + 0.01))
    else:  # fallback: nested squares (outer ring 5 mm, inner square 10 mm) + id dots
        out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="black"/>' % (x0, y0, MARK, MARK)); out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="white"/>' % (x0 + 5, y0 + 5, MARK - 10, MARK - 10))
        out.append('<rect x="%.2f" y="%.2f" width="10" height="10" fill="black"/>' % (cx - 5, cy - 5))
        for i in range(marker_id + 1): out.append('<circle cx="%.2f" cy="%.2f" r="1.2" fill="white"/>' % (x0 + 7 + i * 4, y0 + 7))
    out.append('<text x="%.2f" y="%.2f" font-size="3" font-family="sans-serif" text-anchor="middle">id %d  centre (%.0f, %.0f) mm  side %.0f mm</text>' % (cx, cy + MARK / 2 + 4.5, marker_id, cx, cy, MARK))
    return '\n'.join(out)


def build():
    s = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.0fmm" height="%.0fmm" viewBox="0 0 %.0f %.0f">' % (W, H, W, H), '<rect width="%.0f" height="%.0f" fill="white"/>' % (W, H)]
    for x in range(0, int(W) + 1, 10): s.append('<line x1="%d" y1="0" x2="%d" y2="%.0f" stroke="#999" stroke-width="%.2f"/>' % (x, x, H, 0.25 if x % 50 else 0.45))
    for y in range(0, int(H) + 1, 10): s.append('<line x1="0" y1="%d" x2="%.0f" y2="%d" stroke="#999" stroke-width="%.2f"/>' % (y, W, y, 0.25 if y % 50 else 0.45))
    for x in range(0, int(W) + 1):   # top ruler, mm ticks
        L = 5 if x % 10 == 0 else 3 if x % 5 == 0 else 1.5; s.append('<line x1="%d" y1="0" x2="%d" y2="%.1f" stroke="black" stroke-width="0.2"/>' % (x, x, L))
        if x % 10 == 0: s.append('<text x="%d" y="8" font-size="2.6" font-family="sans-serif" text-anchor="middle">%d</text>' % (x, x))
    for y in range(0, int(H) + 1):   # left ruler
        L = 5 if y % 10 == 0 else 3 if y % 5 == 0 else 1.5; s.append('<line x1="0" y1="%d" x2="%.1f" y2="%d" stroke="black" stroke-width="0.2"/>' % (y, L, y))
        if y % 10 == 0 and y: s.append('<text x="9" y="%d" font-size="2.6" font-family="sans-serif" dominant-baseline="middle">%d</text>' % (y, y))
    s.append('<rect x="55" y="140" width="100" height="6" fill="black"/><text x="105" y="152" font-size="3.2" font-family="sans-serif" text-anchor="middle">PRINT-SCALE CHECK: this bar must measure 100.0 mm; record the printed length on the photo log</text>')
    s.append('<line x1="95" y1="100" x2="115" y2="100" stroke="black" stroke-width="0.4"/><line x1="105" y1="90" x2="105" y2="110" stroke="black" stroke-width="0.4"/><circle cx="105" cy="100" r="1" fill="none" stroke="black" stroke-width="0.3"/>')
    s.append('<text x="105" y="116" font-size="3" font-family="sans-serif" text-anchor="middle">DATUM CROSS (105, 100) mm - align the flange face edge or the palm centre line here</text>')
    s.append('<text x="105" y="200" font-size="3.4" font-family="sans-serif" text-anchor="middle">RH56E2 measurement sheet - A4 portrait - 10 mm grid - markers %s - do not scale to fit</text>' % ('ArUco DICT_4X4_50' if aruco_bits(0)[0] else 'nested-square fallback'))
    meta = {'sheet': 'A4 portrait 210x297 mm', 'grid_mm': 10, 'ruler_mm': 1, 'scale_check_bar_mm': 100, 'datum_cross_mm': [105, 100], 'markers': {}}
    for mid, (cx, cy) in CENTRES.items():
        bits, dname = aruco_bits(mid); s.append(marker_svg(cx, cy, bits, mid)); meta['markers'][mid] = {'centre_mm': [cx, cy], 'side_mm': MARK, 'dictionary': dname or 'fallback_nested_squares'}
    s.append('</svg>'); return '\n'.join(s), meta


def raster(path, dpi=300):
    """PNG version drawn with OpenCV (same geometry; for printers that reject SVG and for the detector self-check)."""
    import cv2
    import numpy as np
    px = dpi / 25.4; w, h = int(round(W * px)), int(round(H * px)); img = np.full((h, w), 255, np.uint8)
    for x in range(0, int(W) + 1, 10): cv2.line(img, (int(x * px), 0), (int(x * px), h), 150, 2 if x % 50 else 4)
    for y in range(0, int(H) + 1, 10): cv2.line(img, (0, int(y * px)), (w, int(y * px)), 150, 2 if y % 50 else 4)
    for x in range(0, int(W) + 1):
        L = 5 if x % 10 == 0 else 3 if x % 5 == 0 else 1.5; cv2.line(img, (int(x * px), 0), (int(x * px), int(L * px)), 0, 2)
    for y in range(0, int(H) + 1):
        L = 5 if y % 10 == 0 else 3 if y % 5 == 0 else 1.5; cv2.line(img, (0, int(y * px)), (int(L * px), int(y * px)), 0, 2)
    cv2.rectangle(img, (int(55 * px), int(140 * px)), (int(155 * px), int(146 * px)), 0, -1)
    cv2.putText(img, 'PRINT-SCALE CHECK: bar = 100.0 mm', (int(55 * px), int(152 * px)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
    cv2.line(img, (int(95 * px), int(100 * px)), (int(115 * px), int(100 * px)), 0, 4); cv2.line(img, (int(105 * px), int(90 * px)), (int(105 * px), int(110 * px)), 0, 4)
    d = None
    try:
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    except Exception:
        pass
    for mid, (cx, cy) in CENTRES.items():
        side = int(round(MARK * px)); x0, y0 = int(round(cx * px - side / 2)), int(round(cy * px - side / 2))
        if d is not None:
            m = cv2.aruco.generateImageMarker(d, mid, side) if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(d, mid, side)
            img[y0:y0 + side, x0:x0 + side] = m
        else:
            cv2.rectangle(img, (x0, y0), (x0 + side, y0 + side), 0, -1); cv2.rectangle(img, (x0 + int(5 * px), y0 + int(5 * px)), (x0 + side - int(5 * px), y0 + side - int(5 * px)), 255, -1)
        cv2.putText(img, 'id %d (%d,%d) mm' % (mid, cx, cy), (x0, y0 + side + int(6 * px)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
    cv2.imwrite(path, img); return img


def self_check(img, dpi=300):
    """Detect the markers in the raster and compare their centres with the declared centres (mm)."""
    import cv2
    import numpy as np
    px = dpi / 25.4; d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()) if hasattr(cv2.aruco, 'ArucoDetector') else None
    corners, ids, _ = det.detectMarkers(img) if det else cv2.aruco.detectMarkers(img, d)
    found = {}
    for c, i in zip(corners, ids.flatten() if ids is not None else []):
        cen = c[0].mean(axis=0) / px; found[int(i)] = [round(float(cen[0]), 2), round(float(cen[1]), 2)]
    errs = {i: round(float(np.hypot(found[i][0] - CENTRES[i][0], found[i][1] - CENTRES[i][1])), 3) for i in found if i in CENTRES}
    return {'detected_ids': sorted(found), 'centres_mm': found, 'centre_error_mm': errs, 'ok': sorted(found) == sorted(CENTRES) and all(e < 0.5 for e in errs.values())}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--out', required=True); p.add_argument('--png', help='also write a 300 dpi PNG and run the ArUco detector self-check'); a = p.parse_args(argv)
    svg, meta = build(); open(a.out, 'w').write(svg)
    if a.png:
        img = raster(a.png); meta['raster_png'] = a.png; meta['detector_self_check'] = self_check(img)
    json.dump(meta, open(a.out.rsplit('.', 1)[0] + '.json', 'w'), indent=1)
    print(a.out, 'markers', {k: v['dictionary'] for k, v in meta['markers'].items()}, meta.get('detector_self_check'))


if __name__ == '__main__':
    sys.exit(main())
