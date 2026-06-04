---
name: mycobot-firmware-quirks
description: myCobot 320-M5 ファームウェアの非自明な挙動（J5 latched lock / get_servo_currents が torque を反映しない件）
metadata:
  type: project
---

# myCobot 320-M5 ファームウェアの非自明な挙動

## J5 latched lock（押下後の復旧不能ラッチ）

ユーザーが手でアームを押した直後、firmware overload protection で特定 servo（特に J5）が「enable=1 / 電流 0 でも動かない」ラッチ状態に入ることがある。

- `clear_error_information()`、`focus_servo(n)`、`power_on()` のいずれも**効かない**
- Python 側からの復旧手段は無い
- **唯一の復旧手段: M5 本体の電源ボタンで再起動**

**Why:** 2026-05 頃に押下イベント後の挙動を切り分けて判明。ファームウェアの保護機構が Python から到達不能な状態を作る。
**How to apply:** デモ中・テスト中にアームが「電源 ON のはずなのに無反応」になったら、Python 側で粘らず即 M5 再起動を案内する。clear_error_information を 10 回叩くより速い。

## get_servo_currents() は torque を反映しない

`get_servo_currents()` の戻り値は torque/外力に応じた電流値**ではない**。

- 人が手で押し込んでも 24mA 程度で頭打ち
- Python 側で実装する `CurrentMonitor` ベースの collision 検出器は**実質機能しない**
- firmware 内蔵の overload protection のほうが先に走る（→ 上記 J5 latch に繋がる）

**Why:** 衝突検知を Python 側で作ろうとして数値を観察した結果。SDK のドキュメント通りに見える API だが、実態はアイドル電流のテレメトリに近い。
**How to apply:** 衝突回避を実装するなら、電流ではなく「target vs actual angle の乖離」や「is_moving のタイムアウト」で検出する設計に倒す。現在の `src/arm/current_monitor.py` は信用しない。

## 関連

- 接続・起動シーケンスの躓きは [[hardware]] 参照
- 安全規約は `.agent/rules/safety.md`
- 接続トラブル切り分けは `.agent/rules/connection-troubleshooting.md`
