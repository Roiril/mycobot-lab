"""Camera intrinsic calibration (Phase 1 minimum).

Reads chessboard images from `data/calib_images/<cam_id>/*.jpg`, runs
cv2.calibrateCamera, and writes the resulting K + dist coefficients into
`data/calibration.json` under cameras.<cam_id>.intrinsics.

Hand-eye (T_ee_cam) is not measured here — Phase 1 keeps it as a manual entry
via --hand-eye CLI. Phase 2 will add ChArUco-based automated hand-eye sweep.

Usage:
  python scripts/calibrate_intrinsics.py --cam wrist
  python scripts/calibrate_intrinsics.py --cam wrist --rows 6 --cols 9 --square-mm 25
  python scripts/calibrate_intrinsics.py --cam wrist --hand-eye 30,0,0,0,0,0   # x,y,z,rx,ry,rz mm/deg
"""
from __future__ import annotations
import argparse
import json
import pathlib
import sys
import datetime
import glob

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parent.parent


def _euler_xyz_to_R(rx_deg, ry_deg, rz_deg):
    import math
    D = math.pi / 180.0
    rx, ry, rz = rx_deg * D, ry_deg * D, rz_deg * D
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,            cx*cy],
    ]


def _hand_eye_T_from_xyzrpy(spec: str):
    parts = [float(x) for x in spec.split(",")]
    if len(parts) != 6:
        raise ValueError("--hand-eye must be 'x,y,z,rx,ry,rz' (mm, mm, mm, deg, deg, deg)")
    x, y, z, rx, ry, rz = parts
    R = _euler_xyz_to_R(rx, ry, rz)
    T = [
        [R[0][0], R[0][1], R[0][2], x],
        [R[1][0], R[1][1], R[1][2], y],
        [R[2][0], R[2][1], R[2][2], z],
        [0.0,     0.0,     0.0,     1.0],
    ]
    return T


def calibrate(cam_id: str, rows: int, cols: int, square_mm: float):
    try:
        import cv2
    except Exception as e:
        sys.exit(f"cv2 unavailable: {e}")

    img_dir = ROOT / "data" / "calib_images" / cam_id
    images = sorted(glob.glob(str(img_dir / "*.jpg")) + glob.glob(str(img_dir / "*.png")))
    if not images:
        sys.exit(f"no images found in {img_dir}; capture chessboard frames first")

    # Chessboard *inner corner* count
    pattern_size = (cols, rows)
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm  # in mm

    objpoints = []
    imgpoints = []
    img_shape = None
    used = 0
    for fp in images:
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
    print(f"\ncalibration done: {used} images, RMS reprojection error = {ret:.3f} px")
    print(f"K = \n{K}")
    print(f"dist = {dist.ravel()}")
    return K.tolist(), dist.ravel().tolist(), list(img_shape)


def update_calibration_file(cam_id: str, K, dist, resolution, hand_eye_T):
    calib_path = ROOT / "data" / "calibration.json"
    if calib_path.exists():
        with open(calib_path, "r", encoding="utf-8") as f:
            calib = json.load(f)
    else:
        calib = {"version": 1, "cameras": {}, "workspace": {"table_z_mm": 0.0, "table_z_uncertainty_mm": 5.0}}
    cams = calib.setdefault("cameras", {})
    entry = cams.get(cam_id, {})
    entry.update({
        "index": entry.get("index", 0),
        "role": entry.get("role", cam_id),
        "resolution": list(resolution),
        "intrinsics": {"K": K, "dist": dist},
        "calibrated_at": datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
        "placeholder": False,
    })
    if hand_eye_T is not None:
        entry["hand_eye_T_ee_cam"] = hand_eye_T
    cams[cam_id] = entry
    with open(calib_path, "w", encoding="utf-8") as f:
        json.dump(calib, f, ensure_ascii=False, indent=2)
    print(f"\nwrote {calib_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", default="wrist", help="camera id in calibration.json")
    ap.add_argument("--rows", type=int, default=6, help="chessboard inner-corner rows")
    ap.add_argument("--cols", type=int, default=9, help="chessboard inner-corner cols")
    ap.add_argument("--square-mm", type=float, default=25.0, help="single chessboard square edge in mm")
    ap.add_argument("--hand-eye", default=None,
                    help="optional hand-eye T_ee_cam as 'x,y,z,rx,ry,rz' (mm/deg). If omitted, existing value is preserved.")
    args = ap.parse_args()

    K, dist, resolution = calibrate(args.cam, args.rows, args.cols, args.square_mm)
    hand_eye_T = _hand_eye_T_from_xyzrpy(args.hand_eye) if args.hand_eye else None
    update_calibration_file(args.cam, K, dist, resolution, hand_eye_T)


if __name__ == "__main__":
    main()
