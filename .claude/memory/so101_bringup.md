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

## 既知の課題
- `tests/test_so101_ik.py` の数値IKが**乱数無シードで非決定的**（毎回 4〜9/40 が no solution）。
  別タスクで切り出し済み。実機制御信頼性に直結。[[dual_impl_single_source]] と同種のドリフト注意。
