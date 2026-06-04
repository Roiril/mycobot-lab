---
name: server-ui-reload
description: server.py は ui.html をリクエストごとに読み直す。編集はブラウザリロードで即反映される。
metadata:
  type: project
---

`scripts/server.py` は `do_GET` でリクエストのたびに `ui.html` をディスクから読み直す。
起動時キャッシュはしていない（過去は `INDEX_HTML` 変数に固定していたが廃止済み）。

**Why:** 起動時1回読み込みだと、ui.html を編集してもブラウザリロードだけでは反映されず、
原因不明の「なぜ動かない」が繰り返し発生していた。

**How to apply:**
- ui.html を編集したら **ブラウザリロードだけで反映される**。サーバー再起動は不要。
- もし再起動が必要な場面があるとすれば `server.py` 自体を変更したとき。
- 将来 `INDEX_HTML` 的なキャッシュ変数を復活させない。

**再起動時の必須チェック**（過去ハマったパターン）：
1. `taskkill /IM python.exe` を打っても他の python 用途で失敗することがある — **PID指定で個別kill** が確実
2. 再起動後は必ず `netstat -ano | grep ":8000.*LISTENING"` で **1つだけ** listen している状態を確認
   — 旧プロセスが残っていると、ブラウザ要求がどちらに行くかランダムになり「変更が反映されたり / されなかったり」する地獄になる
3. ブラウザ側のキャッシュが古いHTMLを掴んでいることもある → **Ctrl+Shift+R**（ハードリロード）で確認
4. 最終確認: `curl -s http://localhost:8000/ | grep -c "<最近追加したマーカー>"` が >0 になっているか
