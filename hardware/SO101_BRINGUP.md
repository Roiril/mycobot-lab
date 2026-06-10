# SO-101 ブリングアップ手順（follower / 12V）

このリポジトリの SO-101 を実機で動かすまでの正本。組み立て後 → 通電 → lerobot
キャリブ → 本リポジトリ接続、の順。**物理＝ユーザー / ソフト＝シュビー** の分担で
進める。ハードは myCobot（24V/115200）とは完全別系統 — 電源・ポートを取り違えない。

---

## 0. 確定している前提

- 機種: **SO-101 follower**（STS3215 ×6 / 5-DoF + グリッパ、**12V** 版）
- 駆動ボード: **Seeed Bus Servo Driver Board for XIAO v1.0**
- ソフト env: `.venv-so101`（Python 3.12 / **lerobot 0.5.1 + torch 2.10.0+cpu** 導入済み）

---

## 1. 駆動ボードの設定（重要な事実）

データシートの回路ブロック図で確認済み。**ボードはオンボードに CH343P（USB-シリアル
変換）を内蔵**しており、`USB-C → CH343P → Logic → SCS Servo` の経路を持つ。

- **lerobot は USB モードで使う**（PC から CH343P 経由でサーボバスを直接叩く）。
- **XIAO へのファーム書き込みは不要。USB モードでは XIAO モジュール自体が不要**
  （XIAO は UART/組み込み単体動作モード用。今回は使わない）。
- **モード切替はハンダジャンパ**:
  - **USB モード（今回これ）= ジャンパパッド未ハンダ**（新品ボードの既定）
  - UART モード = パッドをハンダブリッジ
  - → 新品なら何もしなくて良い。誰かが UART 用にハンダ済みなら外す。
- 電源: **2P 3.5mm スクリュー端子に 12V**（STS3215 の定格に合わせる）。3A 以上供給可能な
  PSU。サーボは 2.5mm 3P 端子へデイジーチェーン。

### Windows ドライバ（CH343）

- Windows 11 は CH343 を標準で認識することが多い（挿すだけで COM 付与）。
- 認識されない場合のみ WCH の **CH343SER** ドライバを入れる:
  https://www.wch-ic.com/downloads/CH343SER_EXE.html
- 認識されれば `lerobot-find-port` に COMx として現れる（myCobot とは別番号）。

---

## 2. 配線チェックリスト（通電前・ユーザー）

1. サーボ 6個が ID1..6 で組み上がっている（→ 手順 4 で ID 付与）。
2. 12V PSU → ボードのスクリュー端子（極性確認）。
3. サーボバス → ボードの 3P 端子。
4. USB-C → ボード → PC。
5. ジャンパパッド = 未ハンダ（USB モード）。
6. **先に 12V、次に USB** の順で投入。全関節を手でゆっくり全可動域 → 干渉/ケーブル噛みゼロ。

---

## 3. ソフト env（導入済み・シュビー担当）

```powershell
# 専用 venv（Python 3.12）。lerobot 0.5.1 + torch 2.10 導入済み
.venv-so101\Scripts\python.exe -c "from lerobot.robots.so_follower import SO101Follower; print('ok')"
```

検証済みの事実:
- `SO101Follower` / `SO101FollowerConfig` / `FeetechMotorsBus` import OK
- 本リポジトリの `profile.JOINT_NAMES` + `GRIPPER_NAME` が lerobot の `action_features`
  （`shoulder_pan.pos` … `gripper.pos`）と**完全一致** → `send_action` のキーがそのまま通る
- `src/robots/so101/driver.py` の lazy import パス（`lerobot.robots.so_follower`）は 0.5.1 と一致

---

## 4. lerobot CLI ブリングアップ（実機接続後）

> CLI は venv の `.venv-so101\Scripts\` 配下。各コマンドは**対話式**（指示に従い Enter）。
> モーターID設定は**機械組み立ての前**にやるのが正解（中央位置基準を取るため）。

```powershell
# (1) COM ポート特定: 挿抜の差分で確定
.venv-so101\Scripts\lerobot-find-port.exe

# (2) モーターID設定（1個ずつ接続して ID1..6 を付与・中央位置センタリング）
.venv-so101\Scripts\lerobot-setup-motors.exe `
  --robot.type=so101_follower --robot.port=COMx

# (3) キャリブレーション（各関節を min→max まで手で動かす）
.venv-so101\Scripts\lerobot-calibrate.exe `
  --robot.type=so101_follower --robot.port=COMx --robot.id=so101_follower_01
```

正準関節順（本リポジトリと一致）:
`shoulder_pan(1) shoulder_lift(2) elbow_flex(3) wrist_flex(4) wrist_roll(5) gripper(6)`

---

## 5. 本リポジトリへの接続（TODO・要実装）

- [ ] `SO101Hub`（HubBase 実装、lerobot lazy ラップ）+ `VirtualSO101Hub`
- [ ] `scripts/server.py` を `--robot {mycobot|so101}` で profile 切替（myCobot を壊さない）
- [ ] `scripts/ui.html` を `num_joints` 駆動に（現状 J1-J6 ハードコード）

→ それまでは **MuJoCo 仮想アーム**で動作確認できる:
```powershell
cd src; python -m robots.so101.sim.demo_sim   # demo.gif + demo_final.png 生成
```

詳細計画: [.agent/plans/2026-06-04_so101-integration.md](../.agent/plans/2026-06-04_so101-integration.md)

## 参考

- ボード Wiki: https://wiki.seeedstudio.com/bus_servo_driver_board/
- SO-101 docs: https://huggingface.co/docs/lerobot/so101
- 組み立て: https://huggingface.co/docs/lerobot/so101 （写真・動画が正本）
