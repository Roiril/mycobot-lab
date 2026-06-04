# mycobot-lab

Elephant Robotics **myCobot 320-M5**（6-DoF 卓上協働アーム）を Python から制御する研究用プロジェクト。pymycobot 経由で USB-Serial（M5Stack Basic の Transponder 経由）で通信する。

## ⚠ 二系統あり — アーム と ハンド を混同するな

このプロジェクトには **物理的に独立した 2 つのロボット** がある。別マイコン・別 COM ポート・別電源・別プロトコル。「動かして」と言われたら **どちらの話か必ず確認**し、コード・ポート・電源を取り違えないこと。

| | 🦾 **アーム** | ✋ **ハンド** |
|---|---|---|
| 機種 | myCobot 320-M5（6-DoF アーム） | Hiwonder 5本指ハンド（LFD-01×5） |
| 制御 MCU | M5Stack Basic + M5Atom | **Arduino Uno** |
| 接続 | CH9102 USB-Serial | Arduino USB シリアル |
| ボーレート | **115200** | **9600** |
| COM | 動的（≈COM12） | 動的（≈COM10）※別ポート |
| 電源 | DC 24V 120W（本体） | **外部 6V**（サーボ用、別系統） |
| SDK/制御 | pymycobot | 生シリアル（`<f> <us>` / `open` / `close`） |
| コード | `src/arm/` `scripts/server.py` | `hand/`（firmware + 今後 Python ドライバ）|
| プロトコル | [.agent/rules/mycobot-protocol.md](.agent/rules/mycobot-protocol.md) | [hand/HANDOFF.md](hand/HANDOFF.md) |

- **「ロボットハンド」= 指のハンド**（アームの先端ではない）。アームには現状エンドエフェクタは付いていない。
- ハンドのファーム書き込み（arduino-cli）・配線・電源は [hand/HANDOFF.md](hand/HANDOFF.md) が正本。
- 詳細メモリ: [memory/two_systems_arm_vs_hand.md](.claude/memory/two_systems_arm_vs_hand.md)

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

## AI エージェント向けドキュメント

- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — 設計思想・モジュール構造・抽象化。新機能を追加する前に読む
- **[docs/AGENT_API.md](docs/AGENT_API.md)** — AI が arm を制御する API リファレンス。動作 verb・Pose ポリシー・エラーコード・retry_hints
- **[docs/VR_TELEOP.md](docs/VR_TELEOP.md)** — WebXR 遠隔操作（アーム/✋ハンド）の挙動仕様・チューニング値
- **[docs/QUEST_DEV.md](docs/QUEST_DEV.md)** — Quest 実機開発ループ（bring-up・反映手順・環境固有値・ハマりどころ）。VR を触る前に読む。ツールは `scripts/quest/qctl.py`、反映は `/quest-reload`

## 共有ハーネス (.agent/)

- [安全規約](.agent/rules/safety.md) — **モーション追加前に必読**
- [UIデザイン規約](.agent/rules/design.md) — **ui.html / CSS を触る前に必読**（VSCode トークン体系・フラット規律）
- [接続トラブルシュート](.agent/rules/connection-troubleshooting.md) — **接続できない時はまずこれ**。初回セットアップで詰まった全パターンと正解の対応付け
- [プロトコルメモ](.agent/rules/mycobot-protocol.md) — シリアルプロトコルと既知事象
- [計画](.agent/plans/) — `YYYY-MM-DD_<slug>.md`
- [タスク](.agent/tasks/) — チェックリスト

## 既知の事象

- 起動直後に COM ポート番号が変わることがある（USB ハブ位置やケーブル抜き差しで）。スクリプトは `list_ports` で再検出するため固定書きしないこと。
- Transponder に入ってもサーボ電源が入るとは限らない。`mc.power_on()` を明示的に呼ぶ。`is_power_on()` で確認。
- `power_on()` の戻り値が -1 でも `is_power_on()` が 1 を返すケースあり（ACK 取りこぼし）— `is_power_on()` を真実とする。
- **押下イベント後の servo latch**: ユーザーが手で押した直後、firmware overload protection で特定 servo が「enable=1 で電流 0 でも動かない」ラッチ状態になることがある。`clear_error_information()`, `focus_servo(n)`, `power_on()` どれも効かない。**復旧は M5 本体の電源ボタン再起動のみ**。詳細は [memory/mycobot_firmware_quirks.md](.claude/memory/mycobot_firmware_quirks.md)
- `get_servo_currents()` は torque 反映の電流値ではない（押されても 24mA 止まり）。Python 側 CurrentMonitor は collision 検出器として実質機能しない — firmware 内蔵保護のほうが先に走る。
