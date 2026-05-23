---
name: hardware
description: myCobot 320-M5 の接続・動作確認済み構成と既知事象
metadata:
  type: project
---

# ハードウェア構成と動作確認済み事実

## 基本

- 機種: Elephant Robotics myCobot 320-M5（6-DoF, DC 24V 120W, シリアル番号 ER32001202300050）
- 制御 MCU: M5Stack Basic（土台） + M5Atom（先端）
- 接続: USB Type-C → CH9102 USB-Serial（VID 1A86 / PID 55D4）
- 動作確認済みボーレート: **115200**
- 動作確認済み応答フレーム: `FE FE 02 02 FA`（version 要求）→ `FE FE 03 02 2A FA`（version=4.2）

## 起動シーケンス（再現性ある手順）

1. STOP ボタン解除（時計回り）
2. 電源 ON → M5 画面起動
3. M5 メニュー → `Transponder` → `USB UART` → OK
4. 画面が `Connect test / Atom: ok` 表示になる ← **これが Transponder 動作中の画面そのもの**（一見診断画面に見えるが正解）
5. PC 側で `python scripts/check.py` 実行 → version / angles が取れれば疎通

**Why:** 2026-05-23 の初回接続で苦戦の末に確立した手順。Atom 通信不良・物理ポート問題・モード誤認の 3 つで詰まり、最終的に通った経緯あり。
**How to apply:** 接続できない時、まずこの手順を上から順に確認する。詳しい切り分けは [.agent/rules/connection-troubleshooting.md](../../.agent/rules/connection-troubleshooting.md) 参照。

## 過去の躓きポイント（初回接続）

### 物理 USB ポート/ケーブル問題で詰まった（最重要）

- 最初は COM11（CH9102 SN: `5626035067`）に接続。CH9102 は Windows に認識される（`Get-PnpDevice` で見える、`Serial.open()` 成功）が、何を送っても無応答だった
- 別の USB-C ポートに挿し直したら COM12（SN: `56E3004757`、**別チップ**）に切り替わり、即正常応答
- **教訓**: デバイスマネージャに見える ≠ データ通信が成立する。CH9102 ↔ M5 内部 UART / USB ケーブルのデータ線が部分的に死んでいる可能性がある
- **対処**: 無応答が続いたら最優先で**ケーブル交換** or **別物理ポート**に挿し直す

### COM ポート番号は動的

- USB の挿し場所/抜き差しで Windows が COM 番号を再割当する
- ポート固定書き禁止。`serial.tools.list_ports` で CH9102 を探す（`src/arm/client.py` の `find_port()`）

### `Atom: no` から `Atom: ok` への復活

- 当初 `Atom: no` で出た
- 対処: 電源サイクル後に `Atom: ok` に復活（接点・グレイブケーブルが原因と推定）
- 公式 FAQ では「Atom を軽く押し込んで接点を整える」も有効と記載

### pymycobot の応答取りこぼし

- `power_on()` が `-1` を返すことがある
- ただし直後の `is_power_on()` は `1` を返す = 実際は ON 完了
- **`is_power_on()` を真実とする**

### `send_angles` 直後の `get_angles` は移動途中値

- 動作完了を待つには `time.sleep()` か `is_moving()` ポーリングが必要
- `Arm.move()` は `_estimate_duration()` で速度から推定 sleep を入れている

### PermissionError(13) の連鎖

- スクリプトを連打すると COM が前回プロセスに掴まれていてエラー
- 各実行の間に 1-2 秒空ける、または `Serial.close()` を確実に呼ぶ

## 環境

- 日付: 2026-05-23
- OS: Windows 11 Home 10.0.26200
- Python: 3.10.6
- pymycobot: 3.x（`pip install pymycobot` で取得）
- pyserial: 3.5
