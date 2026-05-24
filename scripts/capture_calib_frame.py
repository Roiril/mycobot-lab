"""Capture chessboard frames for camera calibration.

Saves JPEG frames directly from a `cv2.VideoCapture` into
`data/calib_images/<cam_id>/`. Intended as the manual-capture companion to
`scripts/calibrate_intrinsics.py`.

Modes:
  default: interactive — press Enter to capture, Ctrl-C to quit
  --auto : automatic — capture every --interval seconds until --count reached

Usage:
  python scripts/capture_calib_frame.py --cam wrist
  python scripts/capture_calib_frame.py --cam wrist --count 20 --auto --interval 1.5

Notes:
  --index defaults to the calibration.json value for <cam>, falling back to 3.
  Do NOT run while scripts/server.py holds the same camera — close the server first.
"""
from __future__ import annotations
import argparse
import json
import math
import pathlib
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parent.parent
CALIB_PATH = ROOT / "data" / "calibration.json"


def _resolve_index(cam_id: str, override: int | None) -> int:
    if override is not None:
        return int(override)
    if CALIB_PATH.exists():
        try:
            with open(CALIB_PATH, "r", encoding="utf-8") as f:
                calib = json.load(f)
            cam = calib.get("cameras", {}).get(cam_id, {})
            if "index" in cam:
                return int(cam["index"])
        except Exception as e:
            print(f"WARN: failed to read {CALIB_PATH}: {e}", file=sys.stderr)
    return 3  # matches src/arm/constants.py DEFAULT_CAM_INDEX


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", default="wrist", help="camera id (subdir under data/calib_images)")
    ap.add_argument("--index", type=int, default=None,
                    help="cv2.VideoCapture index (default: read from calibration.json or 3)")
    ap.add_argument("--count", type=int, default=20, help="number of frames to capture")
    ap.add_argument("--interval", type=float, default=1.5, help="seconds between --auto frames")
    ap.add_argument("--auto", action="store_true", help="auto-capture every --interval seconds")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    if args.count <= 0:
        ap.error("--count must be > 0")
    if not math.isfinite(args.interval) or args.interval <= 0:
        ap.error("--interval must be > 0")

    try:
        import cv2
    except Exception as e:
        sys.exit(f"cv2 unavailable: {e}")

    out_dir = ROOT / "data" / "calib_images" / args.cam
    out_dir.mkdir(parents=True, exist_ok=True)

    index = _resolve_index(args.cam, args.index)
    print(f"opening camera index={index} ({args.width}x{args.height})...")
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    # warm-up
    for _ in range(3):
        cap.read()
        time.sleep(0.05)

    if not cap.isOpened():
        sys.exit(f"failed to open camera index={index}")

    saved = 0
    try:
        while saved < args.count:
            if args.auto:
                time.sleep(args.interval)
            else:
                try:
                    input(f"[{saved+1}/{args.count}] Move the board, press Enter to capture (Ctrl-C to quit): ")
                except EOFError:
                    print("\nstdin closed, switching to auto mode")
                    args.auto = True
                    time.sleep(args.interval)
            ok, frame = cap.read()
            if not ok or frame is None:
                print("  WARN: read failed, retrying")
                time.sleep(0.2)
                continue
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = out_dir / f"{ts}_{saved:02d}.jpg"
            cv2.imwrite(str(path), frame)
            saved += 1
            print(f"  saved {path}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        cap.release()

    print(f"\ndone: {saved} frame(s) in {out_dir}")


if __name__ == "__main__":
    main()
