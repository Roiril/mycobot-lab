# Quest WebXR 開発ループ（シュビー用）

Meta Quest で WebXR（アーム teleop / ✋ハンド teleop）を実機検証するときの**手順とハマりどころ**を1枚に集約。挙動の仕様は [VR_TELEOP.md](VR_TELEOP.md)、ハンド構成は [hand/HANDOFF.md](../hand/HANDOFF.md)。

> ⚠ 二系統注意: アーム(myCobot) と ハンド(Arduino) は別物。混同しない（[CLAUDE.md](../CLAUDE.md) の二系統テーブル）。

## 環境（このマシン固有）

| 項目 | 値 |
|---|---|
| Quest デバイス id | `2G0YC1ZF890864`（Quest 3）※Pixel も繋がるので adb は必ず `-s` で名指し |
| adb | SideQuest 同梱: `C:\Users\kouga\AppData\Local\Programs\SideQuest\resources\app.asar.unpacked\build\platform-tools\adb.exe` |
| arduino-cli | Arduino IDE 同梱: `C:\Users\kouga\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe` |
| サーバポート | 8001 |
| CDP ポート | 9223 |
| ハンド COM | 動的（≈COM10、Arduino UNO）。アーム CH9102 と別 |

## なぜ localhost:8001（HTTPS 不要）

WebXR は localhost 以外 HTTPS 必須。**`adb reverse` で Quest の localhost:8001 を PC へ転送**すれば localhost 扱い＝HTTPS 不要・LAN 公開なし・token 不要で `immersive-vr` が動く。証明書セットアップは不要。

## ブリングアップ（最初の1回）

```bash
ADB="C:/Users/kouga/AppData/Local/Programs/SideQuest/resources/app.asar.unpacked/build/platform-tools/adb.exe"
Q="-s 2G0YC1ZF890864"

# 1. サーバ起動（仮想アーム + 実ハンド。アームを動かしたいなら --real-hand を外し --offline も外す）
python scripts/server.py --offline --real-hand --port 8001 &

# 2. ポート転送：reverse=サーバ(Quest→PC) / forward=CDP(PC→Quest)
"$ADB" $Q reverse tcp:8001 tcp:8001
"$ADB" $Q forward tcp:9223 localabstract:chrome_devtools_remote

# 3. Quest ブラウザでタブを開く（最初の1回だけ VIEW intent 可。以降は CDP nav）
"$ADB" $Q shell am start -a android.intent.action.VIEW -d "http://localhost:8001" com.oculus.browser
```

ヘッドセット内: 「VR 開始」→ 制御方式で 🦾アーム / ✋ハンド を選ぶ。

## 修正→反映ループ（毎回これ）

**コードを直したら必ず**この手順で反映する（[memory: quest-reload-protocol](../.claude/memory/feedback_quest_reload_protocol.md)、ユーザーが強く要請）。スラッシュコマンド [/quest-reload](../.claude/commands/quest-reload.md) で一発。

| 変えたもの | 手順 |
|---|---|
| `scripts/ui.html` のみ | `python scripts/quest/qctl.py reload` だけ（ui.html はリクエスト都度読み直し→サーバ再起動不要） |
| `scripts/server.py` / `hand/hand_driver.py` | サーバ再起動 → `qctl reload` |
| `hand/hand_control.ino` | arduino-cli で COM へ upload（下記）→ サーバ再起動 → `qctl reload` |

`qctl reload` = VR終了 → **CDP `Page.reload`（その場・新タブを作らない）** → 3.5s 待ち → recorder 再注入。

> ⚠ **`adb am start VIEW` で更新しない**: Oculus Browser は VIEW intent で毎回新タブを開く → タブ増殖 → GPU 圧迫 → jank。増えたら `python scripts/quest/qctl.py tabs` で1枚に。

## デバッグツール（`scripts/quest/qctl.py`）

```bash
python scripts/quest/qctl.py check        # VRモード/バッジ/in-VR/hook/ハンドus を一覧
python scripts/quest/qctl.py reload        # 反映（上記）
python scripts/quest/qctl.py nav <url>     # 同一タブで遷移（新タブ作らない）
python scripts/quest/qctl.py end           # VR セッション終了
python scripts/quest/qctl.py tabs          # 1タブに集約
```

画面確認（パススルー＋ブラウザ）は **PowerShell で** screencap（Git Bash だと `/sdcard/` が MSYS パス変換で壊れる）:
```powershell
& $adb shell screencap -p /sdcard/s.png; & $adb pull /sdcard/s.png "$env:TEMP\s.png"
```

## ファーム書き込み（ハンドの .ino を変えたとき）

```powershell
$cli = "C:\Users\kouga\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
& $cli board list                      # COM 確認（Arduino UNO の行）
& $cli compile --fqbn arduino:avr:uno hand\hand_control
& $cli upload -p COM10 --fqbn arduino:avr:uno hand\hand_control
```
書き込み中はサーバが COM を掴んでると失敗 → 先にサーバ停止。

## ハマりどころ（実際に踏んだ）

- **「動かない」の第一容疑は外部6V電源**。ソフト/シリアルが全部正常でもサーボ用 6V が落ちてると動かない（Arduino ロジックは USB 給電で生きるので紛らわしい）。電池4本/共通GND/極性を確認。→ [hand/HANDOFF.md](../hand/HANDOFF.md)
- **ハンドを USB 抜き差ししたらサーバ再起動が必須**。ドライバは自動再接続しない。古いシリアルハンドルを掴んだままだと `t` は ack 無しなので `connected:True` と誤表示するが書き込みは届かない。`GetPortNames` に COM が二重表示されたらこれ。
- **`cur_us` が変わる ≠ 実機に届いた**。`cur_us` はドライバ内部状態。実機到達はファームの ack（`open`/`close`/`n`）で確認（teleop `t` は ack 無し）。
- **firmware ack は遅延する**: open/close/n は blocking ランプ（~1.3s）。短い read 窓だと ack が次の read にずれて見える。生きてないわけではない。
- **CDP forward が外れる**: Pixel が繋がると forward が落ちることがある。`qctl` がページを見つけられなければ forward を張り直す。
- **inspect.py shadow（過去の地雷）**: `%TEMP%\inspect.py`（`import bpy`）が stdlib inspect を shadow する。だから Quest ヘルパは Temp でなく **この repo（`scripts/quest/`）に置く**。Temp の旧 `xr/` 版は廃止。

関連: [VR_TELEOP.md](VR_TELEOP.md)・[hand/HANDOFF.md](../hand/HANDOFF.md)・[memory/hand_teleop](../.claude/memory/hand_teleop.md)・[memory/quest-reload-protocol](../.claude/memory/feedback_quest_reload_protocol.md)
