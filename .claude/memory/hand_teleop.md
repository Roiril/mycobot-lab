---
name: hand_teleop
description: ✋ハンドの Quest 指追従 + Python ドライバ + server endpoint + UI 手動パネルの構成（2026-06-03 構築）
metadata:
  type: project
---

Quest ハンドトラッキングの各指の曲げ→ロボットハンド各指、を実装した（[[two_systems_arm_vs_hand]] のハンド側）。

**経路**: Quest(ブラウザ) → server.py プロキシ → Arduino。ハンドは Arduino 別シリアルなのでブラウザから直接は触れず、必ず server 経由。

**構成要素**:
- `hand/hand_driver.py` — `HandDriver`(実機)/`VirtualHand`。`set_bends([0..1]*5)`（0=開/伸展, 1=閉/屈曲）を各指 OPEN_US↔CLOSE_US で線形マップ。Arduino 自動検出は CH9102（=アーム）を除外。
- `scripts/server.py` — `HAND` グローバル、`--hand-port`/`--no-hand`、`/hand/status`・`/hand/fingers`（throttle 40Hz, latest-wins）・`/hand/preset`。offline=VirtualHand。
- firmware `t u0..u4` = non-blocking teleop（blocking の open/close は ~1.5s 詰まるのでライブ不可）。
- `scripts/ui.html` — VR `ctrlMode='hand'`。右手の指 curl 角→bend。**クラッチ無し・キャリブ無し**（固定閾値）。右手が見えてる間ずっと常時追従。手動パネル（5指スライダ）。アームは blue / ハンドは teal で視覚分離。
  - **ハンドモードVRではアームを完全非表示・非移動**: `_xrStart` の hand 枝で `armCurrent.group.visible=false`（注意: armCurrent は buildArm が返す `{group,...}` ラッパ。`.visible` ではなく `.group.visible` を触る—ここで一度ハマった）。`/move` も実機ポーリングもスキップ。`_xrOnEnd` で group.visible を戻す。

**指 curl の出し方**: WebXR 関節 [metacarpal, proximal, tip] の (proximal-metacarpal) と (tip-proximal) の成す角（rad, 0=伸展）。固定 `hand.calibOpen`/`calibClosed`（rad, 親指は閾値低め）で 0..1 正規化、clamp。キャリブUI・engage(左ピンチ)・localStorage永続は一度実装したが「面倒」で全廃（2026-06-03）。

**Why:** 既存の Quest→アーム teleop 資産を流用し、入力源だけ手首pose→指曲げに替えた。

**How to apply:** offline 検証は `python scripts/server.py --offline` + ブラウザ（VirtualHand）。実機+Quest は `--offline --real-hand`（仮想アーム+実ハンド）or launch.json の `hand-quest`。Quest 開発ループ/反映は **docs/QUEST_DEV.md** + `/quest-reload`（`scripts/quest/qctl.py`）。

**運用の罠（実機で踏んだ）:**
- 「動かない」第一容疑は**外部6V電源**（ソフト全部正常でもサーボ電源落ちで動かない。Arduino は USB 給電で生きるので紛らわしい）。
- **ハンド USB 抜き差し→サーバ再起動が必須**（自動再接続しない。古いハンドル掴んだままだと `t` は ack 無しで `connected:True` 誤表示・書き込み届かず）。
- `cur_us` 変化≠実機到達（ドライバ内部状態）。実機到達は ack ありコマンド（open/close/n）で確認。
- 親指 teleop は `invert[0]=true`（curl 方向が他指と逆）。teleop ランプはドライバが接続時 `tspd 40 6` 送出（既定25/8より速い）。
