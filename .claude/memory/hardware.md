---
name: hardware
description: myCobot 320-M5 の接続・動作確認済み構成と既知事象
metadata:
  type: project
---

# ハードウェア構成と既知事象

- 機種: Elephant Robotics myCobot 320-M5（6-DoF, DC 24V 120W）
- 制御 MCU: M5Stack Basic + M5Atom（先端）
- 接続: USB Type-C → CH9102 USB-Serial（VID 1A86 / PID 55D4）
- 動作確認済みボーレート: **115200**
- 起動シーケンス: STOP 解除 → 電源 ON → M5 メニュー `Transponder` → `USB UART` → OK → 画面が `Connect test / Atom: ok` になれば PC 制御可

**Why:** 2026-05-23 に初回接続で疎通させた実績。再現性のあるシーケンスとして残す。
**How to apply:** 接続トラブル時はまずこのシーケンスから確認。CLAUDE.md §起動手順を参照。

## 過去の躓きポイント

- COM ポート番号は USB ハブの位置/抜き差しで変わるため、`serial.tools.list_ports` で CH9102 を動的に探す（`src/arm/client.py` の `find_port()`）
- 別の物理 USB ポートに挿し直したら復活した事例あり（CH9102 シリアル番号が変わる）。最初に試した USB-C ポート/ケーブル経路の信頼性に注意
- `power_on()` の戻り値が -1 でも `is_power_on()` は 1 を返すケースあり。`is_power_on()` を真実とする
