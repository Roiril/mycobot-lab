# Quest WebXR 開発ループ（シュビー用）

Meta Quest で WebXR（アーム teleop / ✋ハンド teleop）を実機検証するときの**手順とハマりどころ**を1枚に集約。挙動の仕様は [VR_TELEOP.md](VR_TELEOP.md)、ハンド構成は [hand/HANDOFF.md](../hand/HANDOFF.md)。

> ⚠ 二系統注意: アーム(myCobot) と ハンド(Arduino) は別物。混同しない（[CLAUDE.md](../CLAUDE.md) の二系統テーブル）。

## 環境（このマシン固有）

| 項目 | 値 |
|---|---|
| Quest デバイス id | `2G0YC1ZF7S06BW`, `2G0YC1ZF890864`（Quest 3 ×2）※Pixel も繋がるので adb は必ず `-s` で名指し。動的に `adb devices` から取るのが正 |
| adb | SideQuest 同梱: `C:\Users\kouga\AppData\Local\Programs\SideQuest\resources\app.asar.unpacked\build\platform-tools\adb.exe`（`scripts/quest/adb_util.py` が自動解決）|
| arduino-cli | Arduino IDE 同梱: `C:\Users\kouga\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe` |
| サーバポート | 8001 |
| CDP ポート | 9223（1台目）/ 9224（2台目）… `deploy_hand.py` は 9223+デバイス番号を割当 |
| ハンド COM | 動的（≈COM9、FTDI）。アーム CH9102 と別 |

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

## 2台運用（Quest ×2 へハンドページを一括配備）

`scripts/quest/deploy_hand.py` が **接続中の全 Quest** に対して reverse/forward を張り、
既存タブを CDP で `/hand` に向ける（タブが無い時だけ VIEW intent を1回）。ワンコマンド起動
`scripts/teleop_all.ps1` はサーバ起動（cockpit/hand/home）に続けてこれを呼ぶ。

### ポート割当（デバイス番号 i = adb devices の並び順）

| デバイス | reverse（Quest→PC） | forward（PC→CDP） |
|---|---|---|
| 1台目 (i=0) | `tcp:8001 tcp:8001` | `tcp:9223 → chrome_devtools_remote` |
| 2台目 (i=1) | `tcp:8001 tcp:8001` | `tcp:9224 → chrome_devtools_remote` |

- **reverse は両機とも同じ `tcp:8001`**（各 Quest の localhost:8001 が同じ PC の hand server を指す）。ハンドは 1本のサーバ・COM9 を共有し、`/hand/fingers` は latest-wins（排他なし）。
- **forward は機ごとに別ポート**（CDP は device-local。9223 / 9224）。qctl で 2台目を触る時は `--serial <s2> --cdp 9224`。

### 使い方

```bash
python scripts/quest/deploy_hand.py                 # 全デバイス
python scripts/quest/deploy_hand.py --serial 2G0YC1ZF7S06BW --serial 2G0YC1ZF890864
python scripts/quest/deploy_hand.py --dry-run       # adb/CDP コマンドを表示するだけ（実機に触らない）
```

`--dry-run` は adb コマンド組み立ての確認用（実機不要）。既存タブの遷移は CDP `Page.navigate`、
新タブを開くのは「localhost タブが1つも無い時に1回だけ」。**`am start VIEW` でのリロードはしない**（タブ増殖 → GPU jank）。

### qctl で個別デバイスを触る

```bash
python scripts/quest/qctl.py check --serial 2G0YC1ZF890864 --cdp 9224   # 2台目
python scripts/quest/qctl.py nav http://localhost:8001/hand --cdp 9224
```

`--serial` を渡すと qctl が先に `adb -s <serial> forward tcp:<cdp> …` を張ってから接続する（2台目を単独で扱える）。`--serial` 無しは従来どおり（後方互換）。

## ケーブルを抜いて使う（adb over Wi-Fi。2026-08-07 実機確認）

**USB を抜くと `adb reverse` が消えて、ヘッドセットから PC の :8001 に届かなくなる**（指データが1件も来ない形で出る）。
装着して動き回るなら転送を Wi-Fi 側へ移す。`adb reverse` は Wi-Fi トランスポートでも張れるので、
**localhost:8001 の HTTPS 不要トリックはそのまま使える**。

```bash
ADB="C:/Users/kouga/AppData/Local/Programs/SideQuest/resources/app.asar.unpacked/build/platform-tools/adb.exe"
S=2G0YC1ZF7S06BW
"$ADB" -s $S shell "ip -f inet addr show wlan0"      # IP を読む（例 192.168.11.42）
"$ADB" -s $S tcpip 5555                              # adbd を TCP モードで再起動（USB 側は一瞬切れる）
"$ADB" connect 192.168.11.42:5555
"$ADB" -s 192.168.11.42:5555 reverse tcp:8001 tcp:8001
"$ADB" -s 192.168.11.42:5555 forward tcp:9224 localabstract:chrome_devtools_remote
```

- **順番が大事**: USB が生きているうちに tcpip → connect → reverse まで通し、**確認してから抜く**。
- ⚠ **USB を抜くと Wi-Fi 側に張った reverse も消える**（2026-08-07 実測）。先に Wi-Fi へ移してあっても
  抜いた直後は `reverse --list` が空になるので、**抜いたあとにもう一度張り直す**:
  `"$ADB" -s <ip>:5555 reverse tcp:8001 tcp:8001`。張り直せば以降はケーブル無しで維持される。
- 疎通確認に `adb shell curl` / `toybox wget` は使えない（Quest に無い）。**PC 側で
  `Get-NetTCPConnection -LocalPort 8001` を見る**（トンネル経由なので相手は 127.0.0.1 に見える）か、
  `hand_launch` の戻りが `already_native` になることで前面動作を確認する。
- ヘッドセットが再起動・長時間スリープすると TCP の adb は切れる。切れたら USB で挿し直して同じ手順。
- `adb devices` に USB と Wi-Fi の**同じ実機が2行**出る。`launch-hand` は両方に投げるが実害はない。

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
