---
name: dual-impl-single-source
description: FK/IK が CPU(numpy)/GPU(torch)/JS の複数ランタイムに重複実装されている。定数・seed・スコア式は必ず単一ソースから供給し、ドリフトを防ぐ
metadata:
  type: rule
---

このコードベースは同じ運動学を**複数ランタイムで重複実装**している。これは性能上不可避だが、
**値が片方だけ更新されてドリフトする事故**が起きやすい。実際に 1 回やらかした。

## 重複している実装マップ

| ロジック | 実装場所 | ランタイム |
|---|---|---|
| FK (link_frames) | `src/arm/kinematics.py` | Python（**真のソース**）|
| FK | `src/arm/ik_gpu.py` `fk_batch` | torch |
| FK | `scripts/ui.html` `linkFrames` | JS |
| IK seed | `src/arm/ik_policy.py` `POLICY_SEED_TEMPLATES` | **単一ソース** |
| IK seed (消費) | `ik_gpu._policy_seeds_batch` | torch（templates を参照）|
| posture スコア | `ik_policy.posture_score` / `ik_gpu._posture_score_batch` | weight は共有 import、式は別実装 |

## 守るルール

1. **URDF 幾何（URDF_LINKS）と関節限界は kinematics.py が唯一の真実**。
   - JS 側は `/kinematics` エンドポイント経由で取得（ハードコードしない）
   - torch 側も `from .kinematics import URDF_LINKS, JOINT_LIMITS`
2. **IK seed は `POLICY_SEED_TEMPLATES`（ik_policy）だけに書く**。CPU/GPU 両方そこから構築。
   - 過去の事故：肘上げ seed を ik_gpu だけ更新 → reachable_grid.py(CPU) と petal(GPU) で別姿勢
   - 検証：`_policy_seeds(t)` と `_policy_seeds_batch([t])` が `np.allclose` であること
3. **スコア重み（W_*）と中立角（J*_NEUTRAL）は ik_policy が真**。torch 側は import して使う。
   式本体を変える時は **両方の関数を必ずペアで直す**（grep `posture_score` で全箇所）

## How to apply

FK/IK/姿勢ポリシーに触る時は、まず「これは何ランタイムに重複しているか」を上表で確認。
1 箇所直したら対応する全実装を同期。可能なら定数・テンプレートを ik_policy/kinematics に寄せて
「消費側」に薄くする方向へ寄せる（seed はこの形に整理済み）。
