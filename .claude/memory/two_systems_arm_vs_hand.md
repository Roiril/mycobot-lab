---
name: two_systems_arm_vs_hand
description: このプロジェクトはアーム(myCobot)とハンド(Hiwonder 5指)の二系統。別MCU・別COM・別電源・別プロトコル。混同禁止
metadata:
  type: project
---

mycobot-lab には**物理的に独立した 2 つのロボット**がある。「ロボットを動かして」と言われたらどちらか必ず確認する。

| | 🦾 アーム | ✋ ハンド |
|---|---|---|
| 機種 | myCobot 320-M5（6-DoF） | Hiwonder 5本指（LFD-01×5サーボ） |
| MCU | M5Stack Basic + M5Atom | Arduino Uno |
| ボーレート | 115200 | 9600 |
| COM | 動的 ≈COM12 | 動的 ≈COM10（**別ポート**） |
| 電源 | DC 24V 本体 | 外部 6V（サーボ用・別系統） |
| 制御 | pymycobot / HTTP `:8000` | 生シリアル `<f> <us>`/`open`/`close` |
| コード | `src/arm/` `scripts/server.py` | `hand/` |
| 正本ドキュメント | CLAUDE.md / .agent/rules/ | `hand/HANDOFF.md` |

**「ロボットハンド」は指のハンドを指す。アーム先端ではない**（アームにエンドエフェクタは未装着）。

**Why:** ユーザーが過去に取り違えを警戒して明示要請（2026-06-03）。ポート・電源・プロトコルが全部違うので、混同すると 9600 のハンドに 115200 で繋ぐ等の事故になる。

**How to apply:** 動作指示が来たら対象を一意化。アーム=[[hardware]] / [[mycobot_firmware_quirks]]、ハンド=hand/HANDOFF.md を参照。VR teleop はアーム=手首pose、ハンド=各指の曲がり、と入力源が別。
