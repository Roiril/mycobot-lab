"""Camera intrinsic calibration (Phase 1 minimum).

Reads chessboard images from `data/calib_images/<cam_id>/*.jpg`, runs
cv2.calibrateCamera, and writes the resulting K + dist coefficients into
`data/calibration.json` under cameras.<cam_id>.intrinsics.

Hand-eye (T_ee_cam) is Phase 1 manual. Pass `--hand-eye X,Y,Z,RX,RY,RZ` to set
real values; omit to keep whatever is already in calibration.json (placeholder
flag is preserved when --hand-eye is omitted).

Usage:
  python scripts/calibrate_intrinsics.py --cam wrist
  python scripts/calibrate_intrinsics.py --cam wrist --rows 6 --cols 9 --square-mm 25
  python scripts/calibrate_intrinsics.py --cam wrist --hand-eye 30,0,0,0,0,0
  python scripts/calibrate_intrinsics.py --cam wrist --table-z-mm -15.0

Hand-eye coordinate convention (see docs/AGENT_API.md §8.10):
  X,Y,Z = J6 origin → camera lens center, expressed in EE frame (mm)
  RX,RY,RZ = intrinsic XYZ Euler from EE frame to camera frame, applied as
             Rz · Ry · Rx (deg)
"""
from __future__ import annotations
import argparse
import json
import math
import os
import pathlib
import sys
import datetime
import glob

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parent.parent


def _euler_xyz_to_R(rx_deg, ry_deg, rz_deg):
    D = math.pi / 180.0
    rx, ry, rz = rx_deg * D, ry_deg * D, rz_deg * D
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Intrinsic XYZ Euler: R = Rz · Ry · Rx
    return [
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,            cx*cy],
    ]


def parse_hand_eye(spec: str):
    """Parse '--hand-eye X,Y,Z,RX,RY,RZ' string into a 4x4 transform list.

    Raises ValueError on malformed input (wrong arity, non-numeric, NaN, Inf).
    """
    try:
        parts = [float(x) for x in spec.split(",")]
    except ValueError as e:
        raise ValueError(f"--hand-eye contains non-numeric value: {e}") from None
    if len(parts) != 6:
        raise ValueError(
            f"--hand-eye must have exactly 6 comma-separated values (got {len(parts)}); "
            "format: 'X,Y,Z,RX,RY,RZ' (mm,mm,mm,deg,deg,deg)"
        )
    for i, v in enumerate(parts):
        if not math.isfinite(v):
            raise ValueError(f"--hand-eye value [{i}] is not finite: {v}")
    x, y, z, rx, ry, rz = parts
    R = _euler_xyz_to_R(rx, ry, rz)
    T = [
        [R[0][0], R[0][1], R[0][2], x],
        [R[1][0], R[1][1], R[1][2], y],
        [R[2][0], R[2][1], R[2][2], z],
        [0.0,     0.0,     0.0,     1.0],
    ]
    return T


def _looks_like_default_placeholder(entry: dict) -> bool:
    """Heuristic: is the existing hand_eye matrix the shipped placeholder?

    The placeholder is identity rotation + translation (30, 0, 0). Used as a
    safety net when older calibration files lack an explicit `placeholder` flag.
    """
    T = entry.get("hand_eye_T_ee_cam")
    if not isinstance(T, list) or len(T) != 4:
        return False
    try:
        # identity rotation + (30, *, *) translation
        if abs(T[0][0] - 1.0) > 1e-6 or abs(T[1][1] - 1.0) > 1e-6 or abs(T[2][2] - 1.0) > 1e-6:
            return False
        # off-diagonals zero
        for (i, j) in [(0, 1), (0, 2), (1, 0), (1, 2), (2, 0), (2, 1)]:
            if abs(T[i][j]) > 1e-6:
                return False
        return abs(T[0][3] - 30.0) < 1e-6
    except Exception:
        return False


def calibrate(cam_id: str, rows: int, cols: int, square_mm: float):
    try:
        import cv2
    except Exception as e:
        sys.exit(f"cv2 unavailable: {e}")

    img_dir = ROOT / "data" / "calib_images" / cam_id
    images = sorted(glob.glob(str(img_dir / "*.jpg")) + glob.glob(str(img_dir / "*.png")))
    if not images:
        sys.exit(f"no images found in {img_dir}; capture chessboard frames first")

    pattern_size = (cols, rows)
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm

    objpoints = []
    imgpoints = []
    img_shape = None
    used = 0
    total = 0
    for fp in images:
        total += 1
        img = cv2.imread(fp)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]  # (w, h)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not ret:
            print(f"  skip (no corners): {fp}")
            continue
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        objpoints.append(objp)
        imgpoints.append(corners2)
        used += 1
        print(f"  ok: {fp}")

    if used < 5:
        sys.exit(f"need >=5 valid images, got {used}")

    ret, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
    print(f"\ncalibration done: {used}/{total} images valid, RMS reprojection error = {ret:.3f} px")
    print(f"K = \n{K}")
    print(f"dist = {dist.ravel()}")
    return K.tolist(), dist.ravel().tolist(), list(img_shape), float(ret), used, total


def _atomic_write_json(path: pathlib.Path, data) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def update_calibration_file(
    cam_id: str,
    K,
    dist,
    resolution,
    hand_eye_T,
    hand_eye_explicit: bool,
    table_z_mm,
    table_z_unc_mm,
):
    """Merge new calibration into data/calibration.json.

    Returns a summary dict with placeholder/path info for the final stdout line.
    """
    calib_path = ROOT / "data" / "calibration.json"
    if calib_path.exists():
        with open(calib_path, "r", encoding="utf-8") as f:
            calib = json.load(f)
    else:
        calib = {"version": 1, "cameras": {}, "workspace": {"table_z_mm": 0.0, "table_z_uncertainty_mm": 5.0}}

    cams = calib.setdefault("cameras", {})
    entry = cams.get(cam_id, {})
    prev_placeholder = bool(entry.get("placeholder", False)) or _looks_like_default_placeholder(entry)

    entry.update({
        "index": entry.get("index", 0),
        "role": entry.get("role", cam_id),
        "resolution": list(resolution),
        "intrinsics": {"K": K, "dist": dist},
        "calibrated_at": datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
    })

    # Hand-eye placeholder guard (B-H5): preserve placeholder flag if user did not
    # provide --hand-eye AND existing hand_eye is still the shipped default.
    if hand_eye_explicit:
        if hand_eye_T is not None:
            entry["hand_eye_T_ee_cam"] = hand_eye_T
        entry["placeholder"] = False
    else:
        if prev_placeholder:
            entry["placeholder"] = True
            print(
                "[!] hand-eye placeholder PRESERVED: intrinsics 校正は完了したが hand_eye は placeholder のまま。\n"
                "    --hand-eye X,Y,Z,RX,RY,RZ で実測値を指定してください (docs/AGENT_API.md §8.10 参照)。",
                file=sys.stderr,
            )
        else:
            # Existing hand_eye is presumed real — keep placeholder=false
            entry["placeholder"] = False

    cams[cam_id] = entry

    # Workspace updates
    ws = calib.setdefault("workspace", {"table_z_mm": 0.0, "table_z_uncertainty_mm": 5.0})
    if table_z_mm is not None:
        ws["table_z_mm"] = float(table_z_mm)
    if table_z_unc_mm is not None:
        ws["table_z_uncertainty_mm"] = float(table_z_unc_mm)

    _atomic_write_json(calib_path, calib)
    print(f"\nwrote {calib_path}")
    return {
        "placeholder": bool(entry.get("placeholder", False)),
        "table_z_mm": ws.get("table_z_mm"),
        "table_z_uncertainty_mm": ws.get("table_z_uncertainty_mm"),
        "path": str(calib_path),
    }


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__,
    )
    ap.add_argument("--cam", default="wrist", help="camera id in calibration.json (default: wrist)")
    ap.add_argument("--rows", type=int, default=6, help="chessboard inner-corner rows")
    ap.add_argument("--cols", type=int, default=9, help="chessboard inner-corner cols")
    ap.add_argument("--square-mm", type=float, default=25.0, help="single chessboard square edge in mm")
    ap.add_argument(
        "--hand-eye", default=None,
        help=(
            "hand-eye T_ee_cam as 'X,Y,Z,RX,RY,RZ' (mm,mm,mm,deg,deg,deg). "
            "X,Y,Z = J6 origin → camera lens center in EE frame. "
            "RX,RY,RZ = intrinsic XYZ Euler (R = Rz·Ry·Rx) from EE to camera frame. "
            "Omit to preserve existing value (placeholder flag also preserved)."
        ),
    )
    ap.add_argument(
        "--table-z-mm", type=float, default=None,
        help="set workspace.table_z_mm in calibration.json (mm). Table surface Z in base frame; "
             "negative if table is below base flange. Omit to leave unchanged.",
    )
    ap.add_argument(
        "--table-z-uncertainty-mm", type=float, default=None,
        help="set workspace.table_z_uncertainty_mm in calibration.json (mm). Default 5.0 if never set.",
    )
    args = ap.parse_args()

    # Validate hand-eye early (argparse error before slow calibration)
    hand_eye_T = None
    if args.hand_eye is not None:
        try:
            hand_eye_T = parse_hand_eye(args.hand_eye)
        except ValueError as e:
            ap.error(str(e))

    if args.table_z_mm is not None and not math.isfinite(args.table_z_mm):
        ap.error("--table-z-mm must be a finite number")
    if args.table_z_uncertainty_mm is not None and (
        not math.isfinite(args.table_z_uncertainty_mm) or args.table_z_uncertainty_mm < 0
    ):
        ap.error("--table-z-uncertainty-mm must be non-negative finite number")

    K, dist, resolution, rms, used, total = calibrate(args.cam, args.rows, args.cols, args.square_mm)
    summary = update_calibration_file(
        args.cam, K, dist, resolution, hand_eye_T,
        hand_eye_explicit=(args.hand_eye is not None),
        table_z_mm=args.table_z_mm,
        table_z_unc_mm=args.table_z_uncertainty_mm,
    )
    print(
        f"\nSUMMARY: cam={args.cam} | {used}/{total} images | RMS={rms:.3f}px | "
        f"placeholder={'true' if summary['placeholder'] else 'false'} | "
        f"table_z_mm={summary['table_z_mm']} | "
        f"table_z_uncertainty_mm={summary['table_z_uncertainty_mm']}"
    )


if __name__ == "__main__":
    main()
