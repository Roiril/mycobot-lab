---
name: so101-operate
description: SO-101（第3のロボット / 5-DoF lerobot follower アーム）を実機で動かす・キャリブする・繋がらない/動かない時に切り分ける知識ハブ。「SO-101 動かして」「SO-101 起動」「校正したい」「サーボが動かない」「実機ジョグ」「so101 繋がらない」等で入る。myCobot(robot-action) とは別系統 — env・サーバ・トラブルシュートが全く違う。
---

# SO-101 Operate — 実機運用 + キャリブ + 切り分けハブ

SO-101 は myCobot とは**完全に別系統**（lerobot / 別 venv / 別サーバ起動 / 別 COM）。
基盤メモリ [memory/so101_bringup.md](../../memory/so101_bringup.md) と
手順正本 [hardware/SO101_BRINGUP.md](../../../hardware/SO101_BRINGUP.md) が前提。ここは
**実機を動かす時の判断フローと、今回(2026-06-10)実際に踏んだ落とし穴＋診断コマンド**をまとめる。

## 0. 現状（2026-06-10 時点）

- **実機が動く状態に到達済み**。組み立て→ID設定→キャリブ→`/so101/*` API + ui.html SO-101 タブ
  でジョグ/HOME/IK/脱力/ABORT/速度、すべて実機検証済み。
- 残TODO: ① `profile.SAFETY["floor_z_mm"]` 暫定0の実測校正（垂れ姿勢で全拒否される）
  ② 12V **5A** 電源（今 2A だと土台/肩 全速同時で sag の可能性）③ IK テスト非決定性
  ④ キャリブ系 script の `so101_tools/` 整理。

## 1. ⚠ env を間違えるな（最頻ハマり）

| 用途 | env | 理由 |
|---|---|---|
| **実機 real ドライバ・lerobot CLI・キャリブ・診断** | **`.venv-so101`**（Python 3.12） | lerobot 必須。3.10 には無い |
| sim/mock の UI 開発・MuJoCo | 既定 Python 3.10 | mujoco はこちら |

`server.py --so101-driver real` は **`.venv-so101` で起動**（lerobot 必須）。pymycobot/mujoco も
.venv-so101 に追加済みなので real でも myCobot 系 import は通る。

UTF-8 強制（cp932 文字化け防止）: コマンド前に `PYTHONIOENCODING=utf-8`。

## 2. 実機を起動して動かす

```powershell
# COM は CH343（VID 1A86 / PID 55D3）。固定書きせず毎回確認:
.venv-so101\Scripts\python.exe -c "import serial.tools.list_ports as lp; [print(p.device,p.description) for p in lp.comports() if p.vid==0x1A86 and p.pid==0x55D3]"

# 実機サーバ（launch.json に so101-real あり / COM は実測値に）
.venv-so101\Scripts\python.exe scripts\server.py --offline --no-hand --so101-driver real --so101-port COM13
```
→ ブラウザ http://localhost:8001/ → **SO-101 タブ**。スライダ=ジョグ / HOME / 脱力 / ABORT / 速度。
3D ビューは実機角度をミラーした MuJoCo デジタルツイン。

API（同一サーバ）: `/so101/state`（torque/moving/stale 含む）, `/so101/jog`（angles or gripper のみ）,
`/so101/ik`, `/so101/home`, `/so101/release`, **`/so101/abort`**（lock外・認証不要・即停止）, `/so101/ping`。
全 verb は speed(deg/s) 受け付け、非有限値は弾く。

## 3. 動かす前の鉄則（毎回）

1. **台座を固定/支える**。土台未固定だと肩トルクが本体を傾けるのに食われ、動かない/危険。
2. **動作宣言**（移動先・速度・所要時間を1行）してから実行（CLAUDE.md 規約）。
3. real 接続時は torque ON で**現在姿勢を保持**（boot で自動 HOME はしない設計）。脱力は `/so101/release`。
4. **困ったら ABORT**（`/so101/abort`・Esc・グローバル ABORT すべて SO-101 を止める）。

## 4. キャリブレーション（GUI が正本）

純正 `lerobot-calibrate` は対話式で扱いづらいので、**ライブ進捗 GUI** を使う：
```powershell
.venv-so101\Scripts\python.exe scripts\so101_calib_server.py --port COM13   # → http://localhost:8012/
```
フロー: ①中立姿勢で「中立をセット」(homing) → ②「記録開始」で全関節を端まで振る(バーが緑) → ③「保存」。
保存 API が通信エラーで落ちたら、GUI の値を吸い出して `scripts/so101_save_cal.py`（バス通信なし）で書く。
校正ファイルは `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json`
（id=`so101_follower` 無印 — 実機ドライバ既定と一致させること。ズレると読まれない）。

## 5. 🔴 診断プレイブック（「読めるが動かない/繋がらない」時）

**まず必ず電源電圧を疑う**（今回最大のハマり）。サーバを止めて COM を空け、`.venv-so101` で:

```python
# (A) バススキャン: 何番が応答するか（配線断・ID確認）
from lerobot.motors.feetech import FeetechMotorsBus
print(sorted(i for ids in FeetechMotorsBus.scan_port('COM13').values() for i in ids))

# (B) 電圧 + 生コマンド移動テスト（controller/calibration を全バイパス）
from lerobot.motors import Motor, MotorNormMode
import time
M={'pan':Motor(1,'sts3215',MotorNormMode.DEGREES),'lift':Motor(2,'sts3215',MotorNormMode.DEGREES)}
b=FeetechMotorsBus('COM13',M); b.connect()
for n in M:
    v=b.read('Present_Voltage',n,normalize=False)      # 0.1V単位: 120=12V 正常 / 49=4.9V 異常
    b.write('Torque_Enable',n,1)
    p0=b.read('Present_Position',n,normalize=False)
    b.write('Goal_Velocity',n,400,normalize=False); b.write('Goal_Position',n,p0+200,normalize=False)
    time.sleep(0.9)
    print(n,'V=',v/10,'moved=',b.read('Present_Position',n,normalize=False)-p0)
b.disconnect()
```

判定:
- **scan で ID 欠け** → そのサーボの配線断（コネクタ挿し直し）。今回 ID2 が抜けて固まった事例あり。
- **電圧 ≈4.9V** → **12V が届いてない**（アダプタが5V品だった / ネジ端子緩み / 極性）。軽負荷の手首は
  動くが土台/肩が「Enable=1・Current=0・Status=0 で全く動かない」過負荷ラッチ風症状になる。
  → 12V を確保すれば直る（サーボは無故障）。**STS3215 は 12V版。5V/7.4V品ではない。**
- **生コマンドで動くが UI で動かない** → safety 拒否（多くは「床に近接」= floor_z 問題、§6）。
- **生コマンドでも動かない & 12V 来てる & 温度正常** → 真の overload latch。**12V 電源入れ直し**で復旧
  （[memory/mycobot_firmware_quirks.md] の latch と同種）。

## 6. floor_z（暫定0）で全モーション拒否される

実機が机に載った状態だと FK 上 tip が z<0 になり「ツール先端が床に近接」で**全 jog/home が拒否**される。
暫定回避: `/so101/release` → 手でアームを起こす（tip を base より上に）→ HOME。
恒久対応: `src/robots/so101/profile.py` の `SAFETY["floor_z_mm"]` を実機の机面実測値（負方向）に校正。

## 7. キャリブの符号・多回転の罠（再校正時）

- STS3215 は **Present = raw − Homing_Offset**。中立を 2048 にする offset = `(u - 2048)`（u=unsigned raw）。
  lerobot の `set_half_turn_homings` は一部関節を符号付きで読み offset>±2047 で落ちる → 自前計算
  （`so101_calib_server.py` の do_home が実装済み）。
- **wrist_roll の多回転カウント**が homing 後に残ると 0°指令で半回転する。→ **12V/サーボ電源を入れ直す**と
  エンコーダがリセットされて直る。組立で horn 向きを間違えた時もこの順で。

## 8. ツール一覧（`.venv-so101` で実行）

| script | 役割 |
|---|---|
| `scripts/so101_calib_server.py` + `so101_calib.html` | ライブ進捗キャリブ GUI（:8012） |
| `scripts/so101_save_cal.py` | 記録値を**バス通信なし**で校正ファイルに直接書く（保存失敗時の保険） |
| `scripts/so101_calibrate.py` | 2段 CLI キャリブ（GUI に上位互換・整理候補） |
| lerobot CLI | `lerobot-find-port` / `lerobot-setup-motors`（組立時の ID 設定・各1回） |

## 関連
- 基盤/学び: [memory/so101_bringup.md](../../memory/so101_bringup.md)
- 手順正本: [hardware/SO101_BRINGUP.md](../../../hardware/SO101_BRINGUP.md)
- 統合計画/フェーズ: [.agent/plans/2026-06-04_so101-integration.md](../../../.agent/plans/2026-06-04_so101-integration.md)
- firmware latch 既知事象: [memory/mycobot_firmware_quirks.md](../../memory/mycobot_firmware_quirks.md)
