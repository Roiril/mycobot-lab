---
description: アームの接続状態と角度を確認
---

`scripts/check.py` を実行し、port / version / powered / angles / coords を表示してください。

version=-1 など異常時は順に確認：

1. M5 画面が `Connect test / Atom: ok` 表示になっているか
2. COM ポートが OS から見えているか
3. 別アプリ（myStudio 等）が COM を掴んでいないか
4. ボーレート不明時は `scripts/sweep.py`
