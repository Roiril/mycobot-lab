---
name: quest-reload-protocol
description: ui.html/サーバを修正したら毎回必ず Quest に反映する手順（VR閉じる→リロード→recorder再注入）
metadata:
  type: feedback
---

WebXR 開発中、**コードを修正したら毎回必ず**この手順で Quest に反映する。ユーザーが2回明示的に要請（強い習慣化要求）。例外なし。

## 必須フロー（修正のたび）

正本ツールは repo の **`scripts/quest/qctl.py`** + コマンド **`/quest-reload`**。手順全体は [docs/QUEST_DEV.md]。旧 `%TEMP%\claude\xr\` の throwaway 版は**廃止**（揮発する＋inspect.py shadow 地雷）。

1. （サーバ `.py`/`.ino` も変えたなら）**サーバ再起動**（ino は arduino-cli upload も）
2. **その場リロード** — `python scripts/quest/qctl.py reload`（VR終了→CDP `Page.reload`→3.5s→recorder再注入を一発）。
   ⚠ **`adb am start VIEW` は使わない**: Oculus Browser は VIEW intent で**毎回新タブを開く**→タブ増殖→GPU 圧迫→jank。必ず CDP `Page.reload`。増えたら `qctl tabs` で1枚に。
3. **確認** — `python scripts/quest/qctl.py check`

**Why:** VR 中リロードのセッション中途切れ、反映漏れの「変わってない」誤判断、タブ増殖 jank を全部防ぐ。

**How to apply:** ui.html だけなら 2→3（都度読み直しなので再起動不要）。サーバ/ドライバ/ファームも触ったら 1→2→3。「直したから試して」の前に必ず完了。adb は Quest 名指し `-s 2G0YC1ZF890864`（Pixel も繋がるため）。

## adb forward/reverse（Pixel 接続で外れることあり、`qctl` がページを見つけられなければ張り直す）
`adb -s 2G0YC1ZF890864 forward tcp:9223 localabstract:chrome_devtools_remote`（CDP）
`adb -s 2G0YC1ZF890864 reverse tcp:8001 tcp:8001`（サーバ localhost 転送）

関連: [[server_ui_reload]]（ui.html は都度読み直し）・[[hand_teleop]]・docs/QUEST_DEV.md（環境固有値・ハマりどころ集）
