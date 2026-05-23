# mycobot-lab

Elephant Robotics **myCobot 320-M5**（6-DoF 卓上協働アーム）を Python から制御する研究用プロジェクト。pymycobot 経由で USB-Serial（M5Stack Basic の Transponder 経由）で通信する。

## ハードウェア

- 機種: myCobot 320-M5（DC 24V 120W）
- 制御 MCU: M5Stack Basic（土台） + M5Atom（先端）
- 接続: USB Type-C → CH9102 USB-Serial → M5Stack Basic
- ボーレート: **115200**（USB UART Transponder モード時）
- COM ポート: Windows 上では動的割当（プロジェクトコードは `serial.tools.list_ports` で自動検出）

## 起動手順（必須）

1. アーム土台の **STOP（緊急停止）ボタン**が解除されていること（時計回りで解除）。
2. 電源 ON → M5 画面が起動。
3. M5 メニュー → **`Transponder`** → **`USB UART`** で OK。
4. 画面が **`Connect test / Atom: ok`** 表示になることを確認（これが Transponder 動作中の表示）。
5. Python から接続可能。

## スタック

- Python 3.10+
- `pymycobot` — Elephant Robotics 公式 SDK
- `pyserial` — シリアル通信
- 依存は [requirements.txt](requirements.txt)

## ファイル構成

| パス | 役割 |
|---|---|
| `src/arm/client.py` | `MyCobot320` のラッパ（接続・電源・安全停止）|
| `src/arm/poses.py` | 名前付き姿勢（home, ready, rest 等）|
| `src/main.py` | エントリポイント |
| `scripts/check.py` | 状態確認（角度・電源・バージョン）|
| `scripts/move.py` | 基本動作テスト |
| `scripts/sweep.py` | 診断: ボーレート探索 |

## 起動コマンド

```bash
pip install -r requirements.txt
python scripts/check.py    # 接続確認
python scripts/move.py     # 基本動作デモ
```

## コーディング規約

- **安全第一**: [.agent/rules/safety.md](.agent/rules/safety.md) を必読。新規モーション追加前に必ず参照。
- **速度**: 動作速度は 50 以下を既定とする（公式 1-100 範囲、過大値は危険）。
- **姿勢ハードコード禁止**: 再利用する姿勢は `src/arm/poses.py` に名前で登録。マジックナンバーをスクリプトに散らさない。
- **接続管理**: スクリプト終了時は必ず `mc.release_all_servos()` または安全姿勢へ復帰してから切断（脱力で落下を防ぐ場合は復帰優先）。
- **COM 自動検出**: ポート固定はしない。`src/arm/client.py` の `find_port()` を使う。

## 禁止事項

- 周囲のクリアランスを確認せずに `send_angles` / `send_coords` を実行しない。
- 速度 > 80 を使わない（必要な時は人がそばにいる時だけ）。
- 緊急停止ボタンに手が届かない位置で動作させない。
- M5 ファームウェアを許可なく書き換えない（myStudio 操作はユーザー確認必須）。

## 応答スタイル

- 端的・論理的・必要最低限。
- ハードを動かす操作の前に、想定の動き（移動先・速度・所要時間）を 1 行で宣言してから実行。

## Claude Code ハーネス (.claude/)

- **[memory/](.claude/memory/)** — 自動メモリ
- **[commands/](.claude/commands/)** — プロジェクト固有スラッシュコマンド
- **[hooks/](.claude/hooks/)** — プロジェクト固有 hook
- **[settings.json](.claude/settings.json)** — 権限・hook
- **[settings.local.json](.claude/settings.local.json)** — ローカル許可（コミット対象外）

## 共有ハーネス (.agent/)

- [安全規約](.agent/rules/safety.md) — **モーション追加前に必読**
- [接続トラブルシュート](.agent/rules/connection-troubleshooting.md) — **接続できない時はまずこれ**。初回セットアップで詰まった全パターンと正解の対応付け
- [プロトコルメモ](.agent/rules/mycobot-protocol.md) — シリアルプロトコルと既知事象
- [計画](.agent/plans/) — `YYYY-MM-DD_<slug>.md`
- [タスク](.agent/tasks/) — チェックリスト

## 既知の事象

- 起動直後に COM ポート番号が変わることがある（USB ハブ位置やケーブル抜き差しで）。スクリプトは `list_ports` で再検出するため固定書きしないこと。
- Transponder に入ってもサーボ電源が入るとは限らない。`mc.power_on()` を明示的に呼ぶ。`is_power_on()` で確認。
- `power_on()` の戻り値が -1 でも `is_power_on()` が 1 を返すケースあり（ACK 取りこぼし）— `is_power_on()` を真実とする。
