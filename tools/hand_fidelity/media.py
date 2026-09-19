"""Small labelled PNG diagrams for the hand-fidelity report, drawn with Pillow only (no matplotlib): source-vs-donor
actuator angle curves, the import-policy map, and the wrist-frame load/COM diagram. Every image carries its evidence
class in the title (CPU / NOMINAL / DONOR / ASSUMED); none is runtime evidence.
"""
from __future__ import annotations

import math

from PIL import Image, ImageDraw, ImageFont


def _font(size=13):
    for name in ('DejaVuSans.ttf', 'DejaVuSansMono.ttf', 'LiberationSans-Regular.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            continue
    return ImageFont.load_default()


class Chart:
    def __init__(self, w=1150, h=600, *, title, xlabel, ylabel, xr, yr, footer=''):
        self.im = Image.new('RGB', (w, h), (255, 255, 255)); self.d = ImageDraw.Draw(self.im)
        self.w, self.h, self.xr, self.yr = w, h, xr, yr
        self.l, self.r, self.t, self.b = 80, w - 30, 60, h - 70
        self.f, self.fs = _font(13), _font(11)
        self.d.text((12, 10), title, fill=(0, 0, 0), font=_font(15))
        self.d.rectangle([self.l, self.t, self.r, self.b], outline=(0, 0, 0))
        for i in range(6):
            x = self.l + (self.r - self.l) * i / 5; y = self.b - (self.b - self.t) * i / 5
            xv = xr[0] + (xr[1] - xr[0]) * i / 5; yv = yr[0] + (yr[1] - yr[0]) * i / 5
            self.d.line([x, self.b, x, self.b + 5], fill=(0, 0, 0)); self.d.text((x - 12, self.b + 8), '%.2g' % xv, fill=(0, 0, 0), font=self.fs)
            self.d.line([self.l - 5, y, self.l, y], fill=(0, 0, 0)); self.d.text((self.l - 44, y - 6), '%.3g' % yv, fill=(0, 0, 0), font=self.fs)
            self.d.line([x, self.t, x, self.b], fill=(235, 235, 235)); self.d.line([self.l, y, self.r, y], fill=(235, 235, 235))
        self.d.rectangle([self.l, self.t, self.r, self.b], outline=(0, 0, 0))
        self.d.text((self.l + (self.r - self.l) / 2 - 40, self.b + 26), xlabel, fill=(0, 0, 0), font=self.f)
        self.d.text((8, self.t - 24), ylabel, fill=(0, 0, 0), font=self.fs)
        if footer:
            self.d.text((12, h - 22), footer, fill=(90, 90, 90), font=self.fs)
        self._legend_y = self.t + 8

    def px(self, x, y):
        return (self.l + (x - self.xr[0]) / (self.xr[1] - self.xr[0]) * (self.r - self.l), self.b - (y - self.yr[0]) / (self.yr[1] - self.yr[0]) * (self.b - self.t))

    def line(self, pts, color, label=None, width=2, dash=False):
        p = [self.px(x, y) for x, y in pts]
        if dash:
            for i in range(0, len(p) - 1, 2):
                self.d.line([p[i], p[i + 1]], fill=color, width=width)
        else:
            self.d.line(p, fill=color, width=width)
        if label:
            self.d.line([self.r - 250, self._legend_y + 6, self.r - 225, self._legend_y + 6], fill=color, width=3)
            self.d.text((self.r - 218, self._legend_y), label, fill=(0, 0, 0), font=self.fs); self._legend_y += 16

    def marker(self, x, y, color, text=''):
        px, py = self.px(x, y); self.d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=color)
        if text:
            tw = self.d.textlength(text, font=self.fs)
            self.d.text((px - tw - 8 if px + tw + 10 > self.r else px + 6, py - 14), text, fill=color, font=self.fs)

    def save(self, path):
        self.im.save(path); return path


def actuator_angle_chart(path, nominal_rows, donor_angles):
    """Nominal E2 (manual Fig. 5) vs donor finger alpha and thumb beta against closure fraction c (0 = open, 1 = closed)."""
    c = Chart(title='Finger alpha and thumb beta vs closure — NOMINAL (E2 manual Fig. 5) vs DONOR (FTP URDF, datum proxy ±5°) — CPU, not measured',
              xlabel='closure fraction c (0 = open endpoint, 1 = closed endpoint)', ylabel='angle to the metacarpal plane [deg]', xr=(0, 1), yr=(60, 180),
              footer='alpha: proximal phalanx vs metacarpal plane; beta: thumb line vs metacarpal plane in the rotation plane. Donor proxy = root x-z plane, thumb tip line. Installed hand: NOT_MEASURED.')
    fa = donor_angles['finger_index']; tb = donor_angles['thumb_rotation']
    c.line([(0, nominal_rows['finger_alpha_open_deg']), (1, nominal_rows['finger_alpha_closed_deg'])], (0, 90, 200), 'finger alpha NOMINAL 170 -> 91.5')
    c.line([(0, fa['alpha_open_deg']), (1, fa['alpha_closed_deg'])], (0, 160, 80), 'finger alpha DONOR %.1f -> %.1f' % (fa['alpha_open_deg'], fa['alpha_closed_deg']))
    c.line([(0, nominal_rows['thumb_beta_open_deg']), (1, nominal_rows['thumb_beta_closed_deg'])], (200, 60, 0), 'thumb beta NOMINAL 170 -> 75')
    k = 'beta_open_tip_line_deg' if 'beta_open_tip_line_deg' in tb else 'beta_open_joint_origin_line_deg'
    c.line([(0, tb[k]), (1, tb[k.replace('open', 'closed')])], (230, 130, 0), 'thumb beta DONOR %.1f -> %.1f' % (tb[k], tb[k.replace('open', 'closed')]))
    c.marker(1, tb[k.replace('open', 'closed')], (230, 130, 0), '25 deg short of nominal opposition')
    return c.save(path)


def import_policy_chart(path):
    c = Chart(title='Finger import policies: Unitree inspire_hand-space radians -> FTP donor radians — closure-preserving (A) vs radian-identity (B) — CPU',
              xlabel='source finger value [rad] (inspire_hand joint space, 0 = open, 1.7 = closed)', ylabel='donor finger target [rad]', xr=(0, 1.7), yr=(0, 1.7),
              footer='A: q_donor = q_src/1.7 * 1.4381. B: q_donor = q_src (refused above 1.4381). At the piston dataset grasp value 1.3 rad the policies differ by 0.2003 rad. Both EXPLORATORY (datum identity unverified).')
    c.line([(0, 0), (1.7, 1.4381)], (0, 120, 200), 'A closure_preserving')
    c.line([(0, 0), (1.4381, 1.4381)], (200, 40, 40), 'B radian_identity (refused > 1.4381)')
    c.line([(1.3, 0), (1.3, 1.3)], (120, 120, 120), None, width=1, dash=True)
    c.marker(1.3, 1.3 / 1.7 * 1.4381, (0, 120, 200), 'A: 1.0997'); c.marker(1.3, 1.3, (200, 40, 40), 'B: 1.3000')
    c.marker(1.4381, 1.4381, (200, 40, 40), 'donor limit 1.4381 (coordinate endpoint)')
    return c.save(path)


def load_diagram(path, record):
    """Side view of the right wrist_yaw frame (x along the forearm): placeholder, hardware feed-forward effective COM and the donor COMs."""
    im = Image.new('RGB', (1180, 430), (255, 255, 255)); d = ImageDraw.Draw(im); f, fs = _font(13), _font(11)
    d.text((12, 10), 'Right wrist load along the forearm axis — frame right_wrist_yaw_link (x -> hand) — DONOR vs ASSUMED; E2 COM NOT_MEASURED (CPU diagram)', fill=(0, 0, 0), font=_font(15))
    x0, y0, scale = 120, 230, 3600.0   # px per m
    d.line([x0 - 40, y0, x0 + 0.20 * scale, y0], fill=(0, 0, 0), width=2); d.text((x0 + 0.20 * scale - 40, y0 + 8), 'x [m]', fill=(0, 0, 0), font=fs)
    for xm in (0.0, 0.0415, 0.10, 0.15, 0.20):
        px = x0 + xm * scale; d.line([px, y0 - 4, px, y0 + 4], fill=(0, 0, 0)); d.text((px - 12, y0 + 12), '%.3g' % xm, fill=(0, 0, 0), font=fs)
    d.text((x0 - 40, y0 - 30), 'wrist_yaw origin', fill=(0, 0, 0), font=fs)
    d.line([x0 + 0.0415 * scale, y0 - 60, x0 + 0.0415 * scale, y0 + 40], fill=(150, 150, 150)); d.text((x0 + 0.0415 * scale + 4, y0 - 60), 'flange / rubber_hand frame +0.0415', fill=(110, 110, 110), font=fs)
    rows = [('URDF placeholder rubber_hand 0.170 kg (bare g1_29dof)', 0.0951, (120, 120, 120), -110),
            ('HW GravityFeedforward effective COM 0.79 kg (ASSUMED: +0.04 m blob)', record['driver_hardware_feedforward_assumption']['right']['effective_com_m_wrist_yaw'][0], (200, 40, 40), -80),
            ('DONOR open 0.8783 kg', record['records']['right']['open']['components']['hand']['com_m'][0], (0, 120, 60), -50),
            ('DONOR closed 0.8783 kg', record['records']['right']['closed']['components']['hand']['com_m'][0], (0, 160, 100), -20),
            ('LEFT donor open, mirrored (F4)', record['records']['left']['open']['components']['hand']['com_m'][0], (0, 90, 180), 40)]
    for label, xm, color, dy in rows:
        px = x0 + xm * scale; d.ellipse([px - 6, y0 + dy - 6, px + 6, y0 + dy + 6], fill=color); d.text((px + 10, y0 + dy - 7), '%s @ x = %.4f m' % (label, xm), fill=color, font=fs)
    dr = record['driver_hardware_feedforward_assumption']['right']
    d.text((12, 335), 'Horizontal forearm: donor first moment %.4f kg m vs assumption %.4f kg m -> %.3f N m uncompensated (right); left %.3f N m.' % (dr['first_moment_kg_m']['donor_open'], dr['first_moment_kg_m']['assumption'], dr['gravity_moment_difference_N_m_at_horizontal_extension'], record['driver_hardware_feedforward_assumption']['left']['gravity_moment_difference_N_m_at_horizontal_extension']), fill=(0, 0, 0), font=f)
    d.text((12, 360), 'Pinocchio reduced-model check (F5_PLACEMENT_STUDY.json), G1 zero pose (forearms horizontal): R shoulder pitch -0.68, elbow -0.69, wrist pitch -0.57 N m; forearm vertical: ~0.', fill=(0, 0, 0), font=fs)
    d.text((12, 385), 'Wrist adapter and held tool: null (unknown, never zero). SIM route unaffected (frozen donor inertias). CPU diagram, no runtime evidence.', fill=(90, 90, 90), font=fs)
    im.save(path); return path
