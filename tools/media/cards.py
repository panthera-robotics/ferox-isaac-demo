"""Title cards and text overlays for the progress montage. PIL only, 1080p.

Runs on the host -- no matplotlib here, and none needed: a title card is a rectangle
and some text, and PIL draws both. Fonts are whatever DejaVu the box has; if none is
found it falls back to PIL's bitmap default rather than failing, because a montage
without a font is still a montage.
"""
from __future__ import annotations

import os
from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
BG = (18, 20, 24)
FG = (238, 240, 244)
DIM = (150, 158, 170)
ACCENT = (90, 200, 250)
OK = (120, 220, 140)
WARN = (250, 190, 90)

_FONTS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
]


def font(size: int, bold: bool = True):
    for p in _FONTS:
        if os.path.exists(p):
            if bold and "Bold" not in p and any("Bold" in q for q in _FONTS if os.path.exists(q)):
                continue
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    for p in _FONTS:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def card(gate: str, title: str, result: str, note: str = "", caveat: str = "") -> Image.Image:
    """A 2 s title card: gate tag, what the clip shows, the one-line result."""
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 14, H], fill=ACCENT)
    d.text((110, 300), gate, font=font(44), fill=ACCENT)
    y = 370
    for line in _wrap(title, 40):
        d.text((110, y), line, font=font(78), fill=FG)
        y += 92
    y += 24
    for line in _wrap(result, 62):
        d.text((110, y), line, font=font(42, bold=False), fill=OK)
        y += 56
    if note:
        y += 16
        for line in _wrap(note, 78):
            d.text((110, y), line, font=font(32, bold=False), fill=DIM)
            y += 44
    if caveat:
        y += 16
        for line in _wrap(caveat, 78):
            d.text((110, y), line, font=font(32, bold=False), fill=WARN)
            y += 44
    return im


def _wrap(text: str, n: int):
    out, cur = [], ""
    for word in text.split():
        if len(cur) + len(word) + 1 > n:
            out.append(cur); cur = word
        else:
            cur = (cur + " " + word).strip()
    if cur:
        out.append(cur)
    return out or [""]


def fit(img: Image.Image, w: int = W, h: int = H, bg=BG) -> Image.Image:
    """Letterbox onto a 1080p canvas without cropping or stretching."""
    im = img.convert("RGB")
    s = min(w / im.width, h / im.height)
    im = im.resize((max(1, int(im.width * s)), max(1, int(im.height * s))), Image.LANCZOS)
    canvas = Image.new("RGB", (w, h), bg)
    canvas.paste(im, ((w - im.width) // 2, (h - im.height) // 2))
    return canvas


def overlay(img: Image.Image, lines, corner="bl", pad=36) -> Image.Image:
    """Burn text into a frame. lines = [(text, size, colour), ...]"""
    im = img.copy()
    d = ImageDraw.Draw(im, "RGBA")
    sizes = [s for _, s, _ in lines]
    boxh = sum(int(s * 1.5) for s in sizes) + pad
    widest = max(int(len(t) * s * 0.56) for t, s, _ in lines)
    boxw = min(W - 2 * pad, widest + 2 * pad)
    x0 = pad if "l" in corner else W - boxw - pad
    y0 = H - boxh - pad if "b" in corner else pad
    d.rectangle([x0, y0, x0 + boxw, y0 + boxh], fill=(10, 12, 16, 205))
    y = y0 + pad // 2
    for t, s, c in lines:
        d.text((x0 + pad // 2, y), t, font=font(s, bold=False), fill=c)
        y += int(s * 1.5)
    return im


def label(img: Image.Image, text: str, size=40, colour=None) -> Image.Image:
    return overlay(img, [(text, size, colour or FG)], corner="tl")
