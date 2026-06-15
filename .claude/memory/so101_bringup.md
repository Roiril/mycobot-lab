---
name: so101_bringup
description: SO-101 は server.py の /so101/* + ui.html SO-101 タブに統合済み・2つの Python env 使い分け・Seeed XIAO ボードは CH343P 内蔵でファーム不要
metadata:
  type: project
---

SO-101（第2アーム、5-DoF follower / 12V / STS3215）の開発基盤メモ（2026-06-09 構築）。
詳細手順は [hardware/SO101_BRINGUP.md](../../hardware/SO101_BRINGUP.md) が正本。

## 2つの Python env（混同するな）
- **`.venv-so101`（Python 3.12, uv 製）**: lerobot 0.5.1 + torch 2.10。実機ドライバ・
  lerobot CLI 専用。`.venv-so101\Scripts\lerobot-find-port.exe` 等。3.12 ランチャは壊れてて
  C:\Python312 は空 → uv の 3.12 を使う。
- **既定の Python 3.10**: mujoco 3.9 が入っている。**オフラインUIサーバ／MuJoCo sim はこちら**で動く
  （lerobot 不要なため）。

## 操作UI（2026-06-10 に server.py へ統合済み — Phase 2 完了）
- **旧 `so101_server.py` / `so101.html`（:8011 別サーバ）は廃止**。今は `scripts/server.py` の
  `So101Subsystem`（/so101/state|jog|ik|home|release|frame.png）+ ui.html の **SO-101 タブ**。
- driver 選択: `--so101-driver {sim,mock,real,off}`（default sim）。real は `--so101-port COMx` 必須。
- **lazy-init**: SO-101 タブを初めて開いた時に MuJoCo をロード（~2s/500MB）。アームだけ使う
  セッションはコスト 0。
- MuJoCo offscreen GL はスレッド親和 → driver 構築も render も `So101Subsystem._gl`
  （単一スレッド executor）上で実行する。ThreadingHTTPServer の handler スレッドから直接
  render を呼ばないこと。
- ドライバ: sim=MuJoCo実形状+PNGレンダ / mock / real=lerobot。brain は `So101Controller`
  （safety検証付き）。`hub.py`(HubBase ラッパ) は作っていない＝不要だったため。

## Seeed Bus Servo Driver Board for XIAO v1.0（重要な訂正）
- **オンボードに CH343P（USB-シリアル）内蔵**。USB-C→CH343P→Logic→サーボバスの経路あり。
- lerobot は **USB モード**で使う → **XIAO へのファーム書き込み不要、XIAO 自体も不要**。
  （当初「XIAO にパススルーファーム要」と誤案内した。データシート回路図で訂正済み）
- モード切替は**ハンダジャンパ**: USB=未ハンダ（新品既定）/ UART=ハンダ。
- Win11 は CH343 を標準認識しがち。ダメなら WCH CH343SER ドライバ。

## 実機ブリングアップ完了（2026-06-10）
- **電源は 12V 必須**（STS3215 12V版）。⚠ 5V/4A アダプタを誤接続して長時間ハマった：
  軽負荷の手首は動くが**土台(ID1)・肩(ID2)が「Enable=1・電流0・Status=0 で全く動かない」**＝
  過負荷ラッチに見える症状になる。サーボの `Present_Voltage`（0.1V単位）を読めば一発判別
  （正常 ≈120=12V / 異常 ≈49=4.9V）。サーボ無故障。**動かない時はまず電源電圧を疑う**。
- キャリブの homing 符号: STS3215 は **Present = raw − Homing_Offset**。off = u − 2048（unsigned）。
  lerobot の `set_half_turn_homings` は一部関節を符号付きで読み offset>2047 で落ちるので自前計算。
- wrist_roll の「多回転カウント」は homing 後に残ることがあり、**12V/サーボ電源の入れ直しで解消**。
- 実機操作: `python scripts/server.py --offline --no-hand --so101-driver real --so101-port COM13`
  を **`.venv-so101` で**（lerobot 必須なため 3.10 env では real が動かない。pymycobot/mujoco を
  .venv-so101 に追加済み）。校正ファイルは id=`so101_follower`（無印）で保存（driver 既定と一致）。
- ABORT は `/so101/abort`（lock 外・認証不要・ペーシングループが should_abort で中断）。
  グローバル `/abort`・Esc も SO-101 を止める。

## 既知の課題
- **`profile.SAFETY["floor_z_mm"]` が暫定 0** のまま。実機が机に載った状態で先端が z<0 になり
  「床に近接」で全モーション拒否される。実機の実測で floor_z を適正値（負方向）に校正要。
- `tests/test_so101_ik.py` の数値IKが**乱数無シードで非決定的**（毎回 4〜9/40 が no solution）。
  別タスクで切り出し済み。実機制御信頼性に直結。[[dual_impl_single_source]] と同種のドリフト注意。
