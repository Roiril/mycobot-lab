"""ロボットハンド操作アプリの launcher アイコンを生成する。

単純な「手」のアイコン（丸角の背景 + 手のひら + 5本指）を PIL で描き、
Android の各密度の mipmap PNG として res/mipmap-*/ic_launcher.png に出力する。

再生成: python gen_icon.py
（生成物 PNG はソースとしてコミットする。ビルドには不要だが差分追跡のため。）
"""
import os
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "res")

# 密度ごとの launcher icon 一辺 px
DENSITIES = {
    "mipmap-mdpi": 48,
    "mipmap-hdpi": 72,
    "mipmap-xhdpi": 96,
    "mipmap-xxhdpi": 144,
    "mipmap-xxxhdpi": 192,
}

BG = (0x2D, 0x7D, 0xF6)      # 青
HAND = (0xFF, 0xFF, 0xFF)    # 白い手


def draw_icon(size: int) -> Image.Image:
    # 4倍のスーパーサンプリングで描いて縮小（アンチエイリアス）
    s = size * 4
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    # 丸角背景
    radius = int(s * 0.22)
    d.rounded_rectangle([0, 0, s - 1, s - 1], radius=radius, fill=BG)

    # 手のひら（丸角の四角）
    palm_w = int(s * 0.44)
    palm_h = int(s * 0.34)
    palm_x = (s - palm_w) // 2
    palm_y = int(s * 0.50)
    d.rounded_rectangle(
        [palm_x, palm_y, palm_x + palm_w, palm_y + palm_h],
        radius=int(s * 0.10), fill=HAND,
    )

    # 4本指（人差し〜小指）
    finger_w = int(s * 0.085)
    gap = int(s * 0.035)
    total = 4 * finger_w + 3 * gap
    start_x = (s - total) // 2
    finger_tops = [0.30, 0.24, 0.27, 0.34]  # 指先の高さ（中指が一番長い）
    for i, top in enumerate(finger_tops):
        fx = start_x + i * (finger_w + gap)
        fy = int(s * top)
        fb = palm_y + int(s * 0.06)
        d.rounded_rectangle(
            [fx, fy, fx + finger_w, fb],
            radius=finger_w // 2, fill=HAND,
        )

    # 親指（左側に斜め）
    thumb_w = int(s * 0.095)
    thumb_top = palm_y + int(s * 0.02)
    thumb_bottom = palm_y + int(s * 0.24)
    tx = palm_x - int(s * 0.02)
    d.rounded_rectangle(
        [tx - thumb_w, thumb_top, tx, thumb_bottom],
        radius=thumb_w // 2, fill=HAND,
    )

    return img.resize((size, size), Image.LANCZOS)


def main() -> None:
    for folder, size in DENSITIES.items():
        out_dir = os.path.join(RES, folder)
        os.makedirs(out_dir, exist_ok=True)
        icon = draw_icon(size)
        out_path = os.path.join(out_dir, "ic_launcher.png")
        icon.save(out_path, "PNG")
        print(f"wrote {out_path} ({size}x{size})")


if __name__ == "__main__":
    main()
