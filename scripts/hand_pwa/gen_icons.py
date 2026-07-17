"""Generate the PWA icons for the ✋ hand-teleop page (scripts/hand.html).

Committed output: icon-192.png / icon-512.png in this directory. server.py
serves them at /hand/icon-192.png and /hand/icon-512.png (referenced by the
Web App Manifest so the Quest App Library shows a recognizable "ロボットハンド操作"
tile).

Font-free by design: the glyph is drawn from primitives (palm + 5 fingers) so
there is no dependency on a CJK / color-emoji font being installed. Re-run to
regenerate:  python scripts/hand_pwa/gen_icons.py

Requires Pillow. If Pillow is unavailable the icons are already committed, so
this script only needs to run when the design changes.
"""
from __future__ import annotations
import pathlib

from PIL import Image, ImageDraw

HERE = pathlib.Path(__file__).resolve().parent

# Theme (mirrors scripts/hand.html :root tokens).
BG = (30, 30, 30, 255)        # --bg #1e1e1e
PANEL = (14, 99, 156, 255)    # --accent-bg #0e639c (rounded plate behind hand)
HAND = (78, 201, 255, 255)    # --accent #4ec9ff


def _rrect(draw, box, radius, fill):
    draw.rounded_rectangle(box, radius=radius, fill=fill)


def render(size: int) -> Image.Image:
    """Draw at 4x then downsample for clean anti-aliasing."""
    s = size * 4
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    # Full-bleed rounded background (safe for maskable-ish display).
    _rrect(d, (0, 0, s - 1, s - 1), radius=int(s * 0.22), fill=BG)
    # Inner accent plate.
    m = int(s * 0.12)
    _rrect(d, (m, m, s - 1 - m, s - 1 - m), radius=int(s * 0.14), fill=PANEL)

    # Hand: palm + 4 fingers + thumb, all rounded rects in accent cyan.
    cx = s * 0.5
    palm_w = s * 0.42
    palm_h = s * 0.30
    palm_top = s * 0.50
    palm_left = cx - palm_w / 2
    _rrect(d, (palm_left, palm_top, palm_left + palm_w, palm_top + palm_h),
           radius=int(palm_w * 0.28), fill=HAND)

    # 4 fingers rising from the palm top.
    fw = palm_w * 0.17
    gap = (palm_w - 4 * fw) / 3
    heights = [0.20, 0.26, 0.24, 0.19]  # index..pinky, as fraction of s
    for i, hf in enumerate(heights):
        fx = palm_left + i * (fw + gap)
        ftop = palm_top - hf * s
        _rrect(d, (fx, ftop, fx + fw, palm_top + fw),
               radius=int(fw * 0.5), fill=HAND)

    # Thumb: angled rounded rect on the left side of the palm.
    tw = palm_w * 0.19
    thumb = Image.new("RGBA", img.size, (0, 0, 0, 0))
    td = ImageDraw.Draw(thumb)
    ty0 = palm_top + palm_h * 0.10
    _rrect(td, (cx - palm_w * 0.62, ty0, cx - palm_w * 0.62 + tw, ty0 + palm_h * 0.85),
           radius=int(tw * 0.5), fill=HAND)
    thumb = thumb.rotate(-32, center=(cx - palm_w * 0.45, palm_top + palm_h * 0.4),
                         resample=Image.BICUBIC)
    img.alpha_composite(thumb)

    return img.resize((size, size), Image.LANCZOS)


def main():
    for sz in (192, 512):
        out = HERE / f"icon-{sz}.png"
        render(sz).save(out, format="PNG")
        print(f"wrote {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
