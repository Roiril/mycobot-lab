---
name: feedback-localstorage-version
description: UIの永続化されるデフォルト値（posePolicy 等）の意味を変えたら LSKEY のバージョンを必ず上げる
metadata:
  type: feedback
---

`scripts/ui.html` の `LSKEY` 経由で localStorage に保存される設定値（`posePolicy` / `speed` / `mode` 等）は、
**起動時に hardcode のデフォルトを上書きする**。

→ 「コードの hardcode を変更」しても、ブラウザ側に古い値がある限り反映されない。
ユーザーは新挙動を見れず、原因不明の不具合として現れる。

**ルール**：以下を変更したら必ず `LSKEY` の末尾 `v<N>` を +1 する。

- `posePolicy` のデフォルト
- 何かのドロップダウンや checkbox のデフォルト state
- 保存される数値のスケール／単位
- 保存される構造そのもの（フィールド追加・削除）

**Why:**
今回 `extend_toward` → `preserve` をデフォルトに変えたが、ユーザーの localStorage に古い `extend_toward` が
残っていたため「ボタンは姿勢任意なのに IK は指差し要求」という挙動になり、長時間原因が掴めなかった。

**How to apply:**
- 単純な値追加（後方互換アリ）はバンプ不要
- 「同じキーで意味が変わる」場合は必ずバンプ
- バンプしたらユーザーに「localStorage がリセットされます」と一声かける
