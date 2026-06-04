---
description: ui.html/サーバ修正を Quest に正しく反映（VR終了→CDP その場リロード→recorder再注入）。タブ増殖させない
---

Quest WebXR に修正を反映する。**`adb am start VIEW` は使わない**（新タブ増殖→jank）。詳細: [docs/QUEST_DEV.md](../../docs/QUEST_DEV.md)、[memory](../memory/feedback_quest_reload_protocol.md)。

手順（この順で実行する）:

1. **サーバ側(.py/.ino)も変えたなら先に反映**:
   - `hand/hand_control.ino` を変えた → arduino-cli で upload（サーバが COM 掴んでると失敗するので先にサーバ停止）
   - `scripts/server.py` / `hand/hand_driver.py` を変えた → サーバ再起動（`python scripts/server.py --offline --real-hand --port 8001`）
   - `scripts/ui.html` だけなら再起動不要（都度読み直し）

2. **ポート転送を念のため張り直す**（Pixel 接続で外れることがある）:
   ```bash
   ADB="C:/Users/kouga/AppData/Local/Programs/SideQuest/resources/app.asar.unpacked/build/platform-tools/adb.exe"
   "$ADB" -s 2G0YC1ZF890864 reverse tcp:8001 tcp:8001
   "$ADB" -s 2G0YC1ZF890864 forward tcp:9223 localabstract:chrome_devtools_remote
   ```

3. **その場リロード + recorder 再注入**（一発）:
   ```bash
   python scripts/quest/qctl.py reload
   ```

4. **確認**: `python scripts/quest/qctl.py check`（VRモード/接続/hook）。タブが増えてたら `python scripts/quest/qctl.py tabs` で1枚に。

「直したから試して」とユーザーに渡す前に、必ず 3 まで完了していること。
