---
name: camera-upright-calibration
description: CAMERA_UPRIGHT_J6_DEG=90 は J1=90° でしか画像が正立しない。J1 によって画像 roll が変わる未解決のキャリブレーションズレ
metadata:
  type: project
---

# Wrist カメラの「upright」が J1 に依存している件

## 観察事実（2026-05-25 実機確認）

`face(direction)` で 4 方向（J1 = 0 / 90 / 165 / -90）に向けて wrist camera フレームを撮ったところ：

| 方向 | J1 | 画像 |
|---|---|---|
| right | 90° | ✅ 正立 |
| back  | 0° | ❌ 90° 横倒し |
| front | 165° | ❌ 90° 横倒し |
| left  | -90° | ❌ 90° 横倒し |

姿勢は全て `[J1, 0, -90, 0, 0, 90]`（J6=`CAMERA_UPRIGHT_J6_DEG`）。

## 矛盾点

[src/arm/constants.py:73](../../src/arm/constants.py:73) のコメントは：

```python
# at J6=-90° it's upright. J6 rotates around flange +Z = camera optical axis,
# so changing J6 only rolls the image, not the camera position.
CAMERA_UPRIGHT_J6_DEG = 90.0
```

コメントは `-90°` と書いているのに実値は `90.0`。さらに「J6 を変えれば image roll だけが変わる」と言いつつ、実際は **J1 が変わると image roll も変わっている**。

つまり「カメラを upright にする J6 値は J1 に依存する」のが実機の挙動。これは：
- カメラの物理取付け方向が想定とずれている、または
- DH の flange frame と camera frame の関係が `T_base_cam_wrist` で正しくモデル化されていない

可能性が高い。

**Why:** 「四方を見回す」要件で初めて顕在化。これまで OBSERVE 系姿勢は J1≈0 固定で使われていて、image roll の異常はあったが「単一姿勢でのズレ」として処理されていた。
**How to apply:**
- `face()` / `point_at` / vision-based 検出を「カメラ正立を前提」にしてはいけない（現状 J1=90° 以外では破綻）
- 正しい修正は CAMERA_UPRIGHT_J6_DEG を J1 の関数にする：観察上は `J6_upright ≈ 90 + (J1 - 90)` ＝ `J6 = J1` で正立する仮説が立つ（要追加検証）
- ただし vision/transforms.T_base_cam_wrist が DH と整合していれば、この補正は IK/perceive 側で吸収されるべき。安易に gestures 側で hack せず、まず `T_base_cam_wrist` の検証から
- 関連: [[hardware]], [[mycobot-firmware-quirks]]

## 関連ファイル

- `src/arm/constants.py` — `CAMERA_UPRIGHT_J6_DEG`
- `src/arm/gestures.py:27` — `face()` が J6 に上記定数を渡す
- `src/arm/vision/transforms.py` — `T_base_cam_wrist`
