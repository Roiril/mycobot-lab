# ロボットハンド 開発引き継ぎ

Hiwonder（LewanSoul）5本指ロボットハンドを制御するサブシステム。
cogni-storage 側のセッションで開封〜基本制御確立まで完了し、ここ mycobot-lab へ引き継ぐ。

---

## ⚠ v2 (2026-06-29): Arduino Uno → **ATOM Lite + 8Servos Unit** へ移行（現行構成）

配線小型化のため、Arduino Uno + ブレッドボードを撤去し、半田なし構成に置き換えた。
**以下より下の「Arduino Uno / D3..D10 / 外部6V / arduino:avr:uno」の記述は v1（旧）**。
ハード詳細・配線は当時の記録として残すが、現行は本節が優先。

| 項目 | v2 現行 |
|---|---|
| 制御 MCU | **M5Stack ATOM Lite（ESP32-PICO-D4）** + **M5 8Servos Unit**（I2C 0x25） |
| 接続 | ATOM の Grove(SDA=G26 / SCL=G32) → 8Servos。ATOM は USB-C 給電 |
| USB シリアル | **FTDI（VID 0403 / "USB Serial Port"）**。⚠ WCH ではない。動的 ≈**COM9** |
| 指→ch | 親=CH0 / 人=CH1 / 中=CH2 / 薬=CH3 / 小=CH4（各 `G/V/S` の並びに挿す） |
| サーボ電源 | 8Servos のオレンジ端子台 **5V/G**（外部供給、実測5V）。HV(9-24V)は不使用 |
| ファーム | [hand_control_atom/hand_control_atom.ino](hand_control_atom/hand_control_atom.ino)（旧Uno版とシリアル**完全互換**） |
| ライブラリ | `M5Unit-8Servo`（GitHub限定・`Documents/Arduino/libraries/` に手動clone）。API: `servo.begin(&Wire,26,32)` → `setAllPinMode(SERVO_CTL_MODE)` → `setServoPulse(ch, us)` |
| ビルド/書込 | `arduino-cli compile --upload -p COM9 --fqbn m5stack:esp32:m5stack_atom hand/hand_control_atom`。**書き込みは直挿し推奨**（ハブだと自動リセット/データが乱れ失敗しうる。検出はハブでも可） |
| ドライバ | [hand_driver.py](hand_driver.py) が FTDI(0403) を自動検出（CH9102/CH343 は除外）。プロトコル・定数は v1 と同じ |

> ポート特定で詰まったら: `esptool.exe --port COMx chip-id` で実体確認（読むだけ・安全）。`ESP32-PICO-D4` と出れば ATOM。詳細メモリ [[atom-hand-controller-port]]。

### ⚠ 「COM が見える」≠「MCU が生きている」（2026-07-17 事故）

シリアル用の **FTDI アダプタは ATOM とは別体・別給電**（ATOM=USB-C / FTDI=自前 USB）。
そのため **ATOM が無給電でも COM ポートは正常に開く**。`hand_driver` / `server.py` は
`connected: true` を返し続け、teleop の `t` コマンドは ACK 無し設計なので、実機が
1mm も動いていないのに UI 上は全て正常に見える（今回これで空回しした）。

- **切り分けは [scripts/hand_diag.py](../scripts/hand_diag.py)**（一次トリアージ）:
  `python scripts/hand_diag.py`。ポート自動検出 → DTR → boot banner → `tspd` ACK →
  `open` ACK の順に検査し、`FTDI は見える/MCU 沈黙 → ATOM の USB-C 給電を確認` まで
  人間可読に判定する。読むだけでは動かない `tspd` ACK が生存判定の本命
  （`open` はサーボが動くので、動かしたくなければ `--no-open`）。
- **注意: ATOM は DTR で自動リセットしない**（Arduino Uno と違う）。よって boot banner が
  空でも死亡ではない。生存判定は banner ではなく `tspd` ACK で行う。
- `hand_driver.HandDriver` は接続時に `ping()`（= `tspd` の ACK 検証）で `mcu_alive` を
  判定し、`status()` に載せる。死んでいても接続は維持（後から給電される場合があるため）。
  UI（`/hand`）のバッジは 3 状態: **接続**(緑) / **MCU応答なし**(赤, connected だが
  mcu_alive=false) / **なし**(present=false)。

### LED の意味（ATOM Lite 本体の RGB, G27）

ファーム（`hand_control_atom.ino`）が状態を LED で示す。**LED が点いている = この MCU は
実際に走っている**（PC 側の「connected」と違い誤魔化せない一次サイン）:

- **緑（点灯）** = boot 完了・アイドル（生きている）
- **青（点滅）** = シリアル受信中。最後の受信から 3 秒で緑に戻る
- **消灯** = 無給電 or ファーム未起動 → ATOM の USB-C 給電を確認

> 実装はコア内蔵の `neopixelWrite()`（追加ライブラリ不要）。書き込みは ATOM の USB-C を
> PC に直挿しして `arduino-cli upload -p COMx --fqbn m5stack:esp32:m5stack_atom
> hand/hand_control_atom`（FTDI 経由では焼けない）。

---

## 別シュビーへの指示（最初に読む）

おまえ（mycobot-lab の シュビー）への依頼。前任シュビーがロボットハンドを箱から出して、
**PC から 5 本の指を独立＋協調制御できる状態**まで持っていった。その続きを頼む。

- **現状**: Arduino Uno にスケッチ書き込み済み。シリアル経由で `open` / `close` / `<指> <µs>` が動く。電源・配線・指マッピング・安全可動域すべて確定済み（このドキュメントに全部ある）。
- **このハンドの研究目的**: 「ロボットハンドが共同作業相手として成立するか」。myCobot アーム研究と同じ系譜。最終的には上位ロジック（LLM・タイミング制御・センサ連動）から手を動かして、人との共同作業を作る。
- **完了済み**（2026-06-03 セッション）:
  1. ✅ **Python ドライバ** — [hand/hand_driver.py](hand_driver.py)。pyserial ラッパ（`HandDriver`/`VirtualHand`）。Arduino 自動検出（CH9102=アームは除外）、正規化曲げ `set_bends([0..1]*5)`、`set_fingers_us`、`open/close/neutral`。
  2. ✅ **server.py 統合** — `--hand-port`/`--no-hand`、endpoint `/hand/status`・`/hand/fingers`（throttle 40Hz）・`/hand/preset`。offline は VirtualHand。
  3. ✅ **Quest ハンドトラッキング → ロボットハンド** — [scripts/ui.html](../scripts/ui.html) WebXR に `ctrlMode='hand'` 追加。右手の各指の曲げ角→robot hand 各指へ。engage=左手ピンチ（右手の指が信号源なので右ピンチは使えない）。指キャリブ（開き/握りを記録）付き。
  4. ✅ **firmware non-blocking teleop** — `t u0..u4` コマンド（上の表）。
  5. ✅ **UI 手動パネル** — ハンド手動操作（5指スライダ+開く/握る/中立）。アームと視覚的に分離（teal アクセント）。
- **次にやること候補**（ユーザーと相談）:
  1. **実機 + Quest 検証** — ファーム再書き込み→Arduino 接続→ドライバ疎通→Quest で指マッピング/キャリブ調整。曲げ抽出の curl 閾値（calibOpen/calibClosed）は実機で詰める。
  2. **名前付きジェスチャ** — `point`/`peace`/`grip`/`thumbsup` 等のプリセット。
  3. **可動域の微調整** — 各指 open/close を自然な握り形に。
  4. **電源強化** — 6V/5A 電源へ（後述）。
- **作業前の確認**: ハードを動かす前に、想定の動き（どの指・どの位置・所要時間）を 1 行宣言してから実行（mycobot-lab の応答スタイルに準拠）。
- **安全**: アームと違い自己干渉・床干渉のリスクは低いが、サーボのストール（突き当て保持）だけ注意。可動域クランプを外さないこと。

このハンドは **myCobot アームとは別系統**（別マイコン・別 COM ポート・別電源）。混同しないこと。

---

## ハードウェア構成

| 項目 | 内容 |
|---|---|
| ハンド | Hiwonder / LewanSoul 5本指ロボットハンド（"Secondary Development" 版。**制御マイコンは付属しない**） |
| サーボ | **LFD-01 ×5**（PWMデジタルサーボ）。動作電圧 4.8〜6V、パルス 500〜2500µs、可動 0〜180°、ストール電流 700mA、無負荷 60mA、ストール保護内蔵 |
| 制御マイコン | **Arduino Uno**（COM ポートは動的。前回は **COM10**。`arduino-cli board list` で再確認すること） |
| 電源 | サーボ用に**外部 6V**（後述）。Arduino 本体は USB 給電 |
| 付属の白い基板 | **Hiwonder Digital Servo Tester**（サーボ1個を手動で回す確認用。**マイコンではない**。協調制御には使えない） |

配線図: [wiring.html](wiring.html)（ブラウザで開く）

## 指 ↔ 番号 ↔ ピン マッピング（確定）

| Finger | 指 | Arduino ピン（信号/オレンジ線） |
|---|---|---|
| 0 | 親指 | D3 |
| 1 | 人差し指 | D5 |
| 2 | 中指 | D6 |
| 3 | 薬指 | D9 |
| 4 | 小指 | D10 |

- 赤線（＋）×5 → 外部 6V ＋レールにまとめる
- 茶線（GND）×5 ＋ **Arduino の GND** → 外部 6V −レールにまとめる（★**共通 GND 必須**）
- **サーボの電源を Arduino の 5V から取らない**（5本同時で最大3.5A、レギュレータが焼ける）

## 電源で得た知見（重要）

セットアップで一番ハマった所。2 つの別問題があった。

1. **電圧不足だった**: 当初 単3×2本（=3V）で駆動しており、サーボ定格 4.8〜6V を下回っていた。
   → 力が出ない・動作不安定・テスターの電圧計ちらつき の原因。
   **単3×4本（=6V）に変えて解決**。直列の本数が電圧を決める（2本=3V、4本=6V）。
2. **電流スパイクで瞬間的な電圧降下**: 大きい/速い動きで他の指が一瞬ピクッと動く。
   AA 電池は内部抵抗が高く、電流スパイクで電圧が落ちる。
   → **スケッチ側の滑らか補間**（小刻みステップ）で di/dt を抑えて緩和。6V にしてからは滑らか OFF でも実用上問題なし。

**電源の選択肢**:
- 現状: 単3×4本（6V）。動くが、5本同時の激しい握りでは電流不足になりうる。
- 手元にある **6V 2A の AC アダプタ（バレルプラグ）** が候補。ただし**極性（センター＋/−）未確認**。使う前にテスターで「中心ピン=赤、外筒=黒」で +6V を確認すること。バレル→端子台 変換が必要。
- 研究で本格運用するなら **6V / 5A 安定化電源**（または同等 AC アダプタ）が本命。
- 緩和策として電源レールに**電解コンデンサ 1000〜2200µF / 耐圧16V以上**（極性注意、＋脚→6V＋、−脚→GND）。

> 注意: 6V 電源では**ストールしてもメーターはちらつかない**（電源が電流を出しきれて電圧が落ちないため）。「ちらつかない＝安全」ではない。真の可動限界は「指がそれ以上動かなくなった所」で判断する。

## ファームウェア（スケッチ）

[hand_control/hand_control.ino](hand_control/hand_control.ino) — Arduino Uno 用。

- ビルド対象 FQBN: `arduino:avr:uno`
- 依存: `Servo` ライブラリ（1.3.0、インストール済み）
- 機能: 各指独立制御、滑らか補間移動、指ごとの安全範囲クランプ、open/close プリセット

### シリアルプロトコル（9600 baud / 改行 = Newline）

| コマンド | 動作 |
|---|---|
| `<finger> <us>` | 指を指定µsへ（例 `0 2000`）。範囲外は自動クランプ。**blocking ランプ** |
| `open` | 全指オープン（blocking） |
| `close` | 全指クローズ（握る、blocking） |
| `n` | 全指ニュートラル（1500µs、blocking） |
| `spd <step>` | blocking 補間ステップµs変更 |
| `t <u0> <u1> <u2> <u3> <u4>` | **TELEOP: 全5指のターゲットを一括設定。non-blocking**。loop() がシリアル読みをブロックせずランプ追従するので ~25Hz ストリームでも詰まらない。**ack 無し**（バス節約）。ハンドトラッキング/連続操作はこれを使う |
| `tspd <us> <ms>` | teleop ランプ調整（us/step, ms/step）。小さいほど穏やかだが追従が遅い |

> ⚠ blocking 系（open/close/<f> <us>）は 1 コマンドで最大 1.5 秒ブロックするのでライブ追従には不向き。連続操作は必ず `t` を使う。`t` 実行後は curUs↔tgtUs が同期され blocking 系とも競合しない。

### 確定済みパラメータ（スケッチ内）

```
finger:        0(親) 1     2     3     4
MIN_US     =   1000  600   600   600   600
MAX_US     =   2000  2400  2400  2400  2400
OPEN_US    =   2000  2400  2400  2400  2400
CLOSE_US   =   1000  600   600   600   600
STEP_US=8  STEP_MS=12  (spd 8 相当)
```

親指は 6V で 1000〜2000 を全域スイープして無負荷・無ストール確認済み。
指1〜4 は open/close（600/2400）で動作確認済み。ストールが出る指があればここを内側に詰める。

## ビルド & 書き込み（Windows）

`arduino-cli` は PATH に無い。**Arduino IDE 同梱版**を使う:

```
$cli = "C:\Users\kouga\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$sketch = "C:\Users\kouga\Projects\Robotics\mycobot-lab\hand\hand_control"

# ポート確認
& $cli board list

# コンパイル → 書き込み（COM は実際の値に）
& $cli compile --fqbn arduino:avr:uno $sketch
& $cli upload -p COM10 --fqbn arduino:avr:uno $sketch
```

AVR コア 1.8.7 インストール済み。Servo ライブラリは `& $cli lib install Servo` で導入済み。

## シリアルから動かす（PowerShell の例）

pyserial 版を書くまでの暫定。`System.IO.Ports.SerialPort` で叩ける:

```powershell
$p = New-Object System.IO.Ports.SerialPort('COM10',9600)
$p.NewLine = "`n"; $p.DtrEnable = $true   # DTR で Arduino が自動リセットされる
$p.Open()
Start-Sleep -Milliseconds 2300            # リセット+ブート待ち（必須）
$null = $p.ReadExisting()                 # "ready..." を読み捨て
$p.WriteLine('n')      ; Start-Sleep -Milliseconds 1500
$p.WriteLine('open')   ; Start-Sleep -Milliseconds 3500
$p.WriteLine('close')  ; Start-Sleep -Milliseconds 3500
$p.Close()
```

ポイント:
- 開いた直後に Arduino がリセットされるので **約2.3秒待ってから**コマンドを送る。
- 改行は `\n`（`NewLine="`n"`）。スケッチは `readStringUntil('\n')`。

## 既知の状態・未解決

- COM ポートは USB の挿し直しで変わりうる。固定書きせず `board list` で確認。
- 6V 2A アダプタの極性未確認（使うならテスターで要確認）。
- 5本同時の強い握りは電流的に未検証（現状は単3×4本）。本運用前に電源強化を検討。
- 上位統合（Python / mycobot-lab スタックとの結合）は未着手。次の主タスク候補。

## 出自

このサブシステムは `cogni-storage/scripts/hand/` で開発し、本ディレクトリへコピーした。
以後の正本は **mycobot-lab/hand/** とする。
研究文脈（ロボットハンド＝共同作業相手）の経緯は、ユーザーの研究メモ（robothand-partner）参照。
