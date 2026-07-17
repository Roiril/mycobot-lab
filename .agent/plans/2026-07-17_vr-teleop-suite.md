# VR テレオペ統合（SO-101 リーダー追従 + Quest ハンドトラッキング）

2026-07-17。オーケストレーション: シュビー（設計）+ opus サブエージェント3体（実装）。

## ゴール

1. SO-101 フォロワーがリーダーへ**滑らかに低遅延で**追従する（フィルタ + 高周期化）
2. Quest 3 ×2台のハンドトラッキングで Hiwonder 5指ハンドをリアルタイム操作
3. 両 Quest の App Library に**分かりやすい名前のアプリ**（ロボットハンド操作）を入れる
4. ワンコマンドで全系統を起動できる

## 実機の前提（2026-07-17 実測）

- COM13 / COM14 = CH343 ×2（SO-101 フォロワー / リーダー。既定: leader=COM14, follower=COM13）
- COM9 = FTDI（ハンド ATOM Lite、9600 baud）
- Quest 3 ×2: adb serial `2G0YC1ZF7S06BW`, `2G0YC1ZF890864`
- 電源: フォロワー 12V5A / リーダー 5V4A / ハンド 5V4A（外部給電済み）
- Android SDK: `C:\Program Files\Unity\Hub\Editor\2022.3.62f2\Editor\Data\PlaybackEngines\AndroidPlayer\SDK`（build-tools 34.0.0）+ 同 `OpenJDK`

## アーキテクチャ（3プロセス並列 + Quest 2台）

```
[SO-101 リーダー COM14] ──┐
                          ├─ cockpit :8013 (.venv-so101)  ← teleop エンジン（本計画 A）
[SO-101 フォロワー COM13] ─┘

[Quest 3 ×2] ── WebXR hand-tracking ── adb reverse tcp:8001 ──> server.py :8001
                                        (--offline --real-hand, Py3.10)
                                                 └── COM9 → ATOM Lite → 5指サーボ（`t` コマンド）

home :8010 = ランチャー（既存。カード追加）
```

- WebXR secure context は従来どおり **adb reverse による localhost 化**で満たす（HTTPS 不要）
- CDP は Quest ごとに forward ポートを分ける（9223 / 9224）
- Quest 上の入口 = 自前ビルドの**ランチャーAPK**（ラベル「ロボットハンド操作」）が
  `com.oculus.browser` へ VIEW intent（http://localhost:8001/hand）を投げて終了する 2D アプリ

## ワークストリーム

### A. SO-101 teleop 滑らか化（opus）
- `src/robots/so101/teleop_engine.py` 新設: One-Euro フィルタ + 60Hz ループ + last-goal 方式
- `so101_cockpit_server.py` / `so101_teleop.py` をエンジン共用に一本化（定数の重複排除）
- 安全機構は不変: 関節別 TORQUE_LIMITS・先行書込・段階トルクON・MAX_STEP クランプ・ConnectionError 復旧
- 純ロジックは unittest（lerobot 遅延 import で Py3.10 でもテスト可能に）

### B. Quest 配備 + ハンド入口（opus）
- `scripts/hand.html`: タイトル「ロボットハンド操作」+ Web App Manifest + アイコン
- `scripts/quest/deploy_hand.py`: 2台一括 setup（reverse/forward/タブ再利用 nav）。qctl の新タブ禁止規約を継承
- `scripts/quest/qctl.py`: 複数デバイス対応（--serial / --cdp-port）
- `scripts/teleop_all.ps1`: cockpit + hand server + Quest 配備のワンコマンド起動
- `home_server.py` にカード追加、docs 更新

### C. ランチャーAPK（opus）
- `scripts/quest/apk/`: Java 1ファイル + PowerShell ビルドスクリプト（javac→d8→aapt2→zipalign→apksigner、全部 Unity 同梱ツール）
- ラベル「ロボットハンド操作」、package `jp.mycobotlab.handteleop`
- 両 Quest へ adb install し `pm list packages` で確認（起動はしない — reverse 未設定でタブが無駄に開くため）

## 検証（シュビー実施）

1. unittest 一式
2. cockpit 起動 → teleop ON → リーダー手動でなくても静置追従の位置ログで周期・ジッタ確認 → OFF
3. hand server 起動 → 合成 POST /hand/fingers で実サーボ動作確認
4. deploy_hand.py 実行 → 両 Quest のタブが localhost:8001/hand を向くこと（CDP で確認）
5. 実際のハンドトラッキング操作はユーザーの装着テストで最終確認

## 決めたこと（変更する場合はここを更新）

- アプリ名 =「ロボットハンド操作」（App Library の提供元不明カテゴリに出る）
- SO-101 teleop は Quest 非依存（PC 内で完結）。Quest はハンド専用
- 2台の Quest が同時に /hand/fingers を叩いた場合は latest-wins（排他はしない。docs に明記）
- teleop 自動ONはしない（cockpit のトグルで明示的に開始 — 安全のため）
