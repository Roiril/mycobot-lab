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
| `src/arm/constants.py` | 全モジュール共通の定数（speed cap、安全マージン、ツール長等）|
| `src/arm/kinematics.py` | Modified DH FK（リンク位置・ツール先端）|
| `src/arm/safety.py` | 関節限界・床干渉・自己干渉チェック（純関数）|
| `src/arm/planner.py` | 関節空間経路計画 + 各 waypoint 検証 |
| `src/arm/hub.py` | `HubBase` ABC + 実機 `Hub` / `VirtualHub` |
| `src/arm/client.py` | `MyCobot320` のラッパ（接続・電源・close）|
| `src/arm/poses.py` | 名前付き姿勢 |
| `scripts/server.py` | HTTP サーバ（JSON API + UI 配信）|
| `scripts/ui.html` | three.js 3D 操作 UI（関節スライダ + drag gizmo + IK）|
| `scripts/check.py` | 状態確認（角度・電源・バージョン）|
| `scripts/move.py` | 基本動作テスト |
| `scripts/sweep.py` | 診断: ボーレート探索 |
| `tests/` | safety / kinematics / planner の単体テスト |

## 起動コマンド

```bash
pip install -r requirements.txt
python scripts/server.py            # 実機、loopback のみ
python scripts/server.py --offline  # 仮想アーム（UI 開発用）
python -m unittest discover tests   # 単体テスト
```

ブラウザで http://localhost:8000/ を開いて操作。

LAN 公開する場合は `--bind 0.0.0.0 --token <secret>` 必須。

## コーディング規約

- **安全第一**: [.agent/rules/safety.md](.agent/rules/safety.md) を必読。新規モーション追加前に必ず参照。
- **速度**: `MAX_SPEED=40` を上限（`src/arm/constants.py`）。
- **姿勢ハードコード禁止**: 再利用姿勢は `src/arm/poses.py` に。マジックナンバーは `constants.py` に集約。
- **接続管理**: `Hub.shutdown()` がカメラ + シリアル両方を確実に閉じる。スクリプト終了時は必ず安全姿勢へ復帰してから切断。
- **COM 自動検出**: ポート固定はしない。`src/arm/client.py` の `find_port()` を使う。
- **safety/kinematics は純関数**: ハード依存無し → テスト容易。`/move` は HUB を介してのみ。

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
