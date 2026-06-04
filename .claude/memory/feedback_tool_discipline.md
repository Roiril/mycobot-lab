---
name: tool-discipline
description: Bash を並列で詰め込むと連鎖キャンセルで固まる。1コールずつ＋専用ツール優先
metadata:
  type: feedback
---

ツール呼び出しが「実行中」のまま固まる／大量に Cancelled 連鎖する事故を繰り返した。原因と対策：

**原因**
1. 1メッセージに Bash/Edit/Read を10〜60個並列で詰め込み、1つが失敗/承認待ちになると依存後続が全部 Cancelled 連鎖。
2. `C:\Users\kouga\AppData\Local\Temp\inspect.py`（Blender 由来 `import bpy`）が Python stdlib `inspect` を shadow → Temp 直下で python 実行すると謎エラーで停止。
3. `python -c` に日本語を含めると Windows cp932 で UnicodeEncodeError。

**How to apply**
- **Bash は原則1コールずつ**。並列するのは互いに独立かつ確実に成功する読み取りのみ（2〜3個まで）。編集と検証を同じメッセージに混ぜない。
- ファイル内容の確認は **Read / Grep ツール**を使う。`python -c` や `sed`/`awk` での自作ダンプは禁止（cp932・shadow で詰まる）。
- 一時スクリプトは Temp 直下でなく `C:\Users\kouga\AppData\Local\Temp\claude\` 配下に置く（inspect.py shadow 回避）。
- JS 構文チェックは `node --check <抽出ファイル>` を1コールで。抽出は Write でファイルを作ってから。
- Edit が「String not found」で空振りしたら、**まず Read で実ディスクを確認**してから当てる（推測で再試行しない）。

関連: [[feedback_act_dont_just_promise]] [[quest-reload-protocol]]
