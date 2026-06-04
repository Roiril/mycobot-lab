# SO-101 統合計画 — 多アーム化リファクタ

作成: 2026-06-04 / 状態: 計画（ハードウェア未着）

## 確定事項（2026-06-04 ユーザー判断）
- **SO-101 follower は 12V 版**（STS3215 ~30kg·cm、PSU は 12V）
- **模倣学習は当面やらない** → **Track B（LeRobot IL / WSL2）は丸ごと延期**。
  leader アーム不要。placo IK / TorchCodec / evdev の Windows 摩擦は当面無視でよい。
- env: **lerobot を optional 依存 + lazy import**（推奨どおり）
- ディレクトリ大移動 **許可**（`src/arm/`→`src/robots/mycobot/` 等）
- vision: **手首カメラ運用を流用**（SO-101 で hand-eye 再キャリブ）
- IK: 当面 **native Windows 上の position-only numeric**。placo は後回し（学習着手時に再検討）

myCobot 320 専用の本リポジトリに Hugging Face **SO-101** を第2アームとして追加する。
myCobot を一切壊さず、UI / vision / VR teleop / spatial_memory / サーバを共有基盤として
両アームで使い回せる構成にする。

---

## 0. 推奨方針（結論）

**案A（このリポジトリを多アーム化）＋ デュアルトラック運用**を採る。

- **Track A — カスタムスタック駆動（最優先・低リスク・native Windows 可）**
  `SO101Follower` / `FeetechMotorsBus` を新しい `HubBase` バックエンドでラップし、
  **既存の UI・VR teleop・vision・spatial_memory・gestures をそのまま SO-101 に流用**する。
  これが「案A の旨味」。三章で構築する。

- **Track B — LeRobot ネイティブ模倣学習（別トラック・WSL2/Linux）**
  teleop(leader→follower) → データセット記録 → ACT/拡散ポリシー学習 → 推論 の
  LeRobot 標準パイプライン。**再実装しない**。CLI をそのまま使い、薄いグルーと
  ドキュメントだけ用意する。SO-101 本来の価値はここ。

理由: SO-101 は制御 SDK・DoF・電圧・ボーレート全てが myCobot と別物で制御層は共有不可。
だが vision/UI/VR は calibration.json でパラメータ化済みで流用できる。
かつ SO-101 の native ワークフローは模倣学習で、これを潰すのは損失。両取りする。

---

## 1. アーキテクチャ — 共有層 vs アーム固有層

調査で確定した分類（`file:line` は主要な結合点）。

### 流用できる（アーム非依存・パラメータ化済み）
| モジュール | 備考 |
|---|---|
| `scripts/server.py` の状態系・vision系・memory系ルート | `/angles` `/coords` `/power` `/perceive` `/memory` 等は `HubBase` 経由のみ |
| `src/arm/vision/*`（camera, detector, localizer, transforms） | hand-eye は `calibration.json` のパラメータ。変更ほぼ不要 |
| `src/arm/spatial_memory.py` | J1 セクタ式だが汎用 |
| `src/arm/planner.py` / `path_cartesian.py` / `ik_path.py` | 関節数非依存（IK はコールバック注入） |
| `scripts/ui.html` の CSS / パネル骨格 / 3D 描画（URDF は `/kinematics` から取得） | 関節数ループのみ要修正 |

### アーム固有（書き直し or 別実装）
| モジュール | 結合点 | SO-101 対応 |
|---|---|---|
| `src/arm/client.py` | pymycobot 全面依存 | `src/robots/so101/driver.py` を新規（lerobot ラップ） |
| `src/arm/kinematics.py` | `URDF_LINKS`/`DH` を 6-DoF でモジュール global、`len(angles)!=6`（109）、`range(6)`（114） | URDF から SO-101 リンク構成を作り config 化 |
| `src/arm/safety.py` | `len!=6`（46）、`range(6)`（50）、衝突ペア・クリアランス定数が myCobot 寸法（31-32, 77-83） | SO-101 ジオメトリで再構築 |
| `src/arm/ik_numeric.py` | 6×6 DLS（FK が 6 関節前提） | FK を URDF 長でパラメータ化 → 5-DoF は位置優先 IK |
| `src/arm/ik_gpu.py` / `ik_policy.py` | 6-DoF 姿勢選好をハードコード | reach-grid 用。MVP 不要、後回し |
| `src/arm/poses.py` / `gestures.py` | 全て 6-tuple、J5/J6 前提の所作 | 5-DoF 版を作る（wrist roll 無で不可な所作あり） |
| `src/arm/constants.py` | `HOME_ANGLES`（67）/`JOINT_LIMITS`（44-51） | RobotConfig 化 |
| `src/arm/hub.py`（HubBase は良い ABC、実装に 6 ハードコード） | `range(6)`（142-146, 413, 415, 544-546） | `num_joints()` 追加・ループ汎用化 |
| `server.py` import 鎖（35-58）・`/kinematics`（451-476）・`/solve_ik`・`/check` | 6-DoF 前提 | RobotConfig 注入 |
| `ui.html` | J1-J6 パネル（287-310）、`range(6)` ループ（705-725）、VR hand→J1/J2/J5/J6 マップ（456, 467） | `num_joints` 駆動に |

### 目標ディレクトリ構成（Phase 1 で移行）
```
src/
  core/                 # アーム非依存
    hub_base.py         # HubBase（num_joints/grasp など抽象）
    planner.py  path_cartesian.py  ik_path.py
    ik_numeric.py       # config 受け取りに一般化
    spatial_memory.py
    robot_config.py     # ★新規: RobotConfig dataclass
  vision/               # arm/ から昇格（非依存）
    camera.py detector.py localizer.py transforms.py
  robots/
    mycobot/            # 既存 myCobot 固有
      driver.py(=旧client) kinematics.py safety.py poses.py
      gestures.py ik_policy.py constants.py hub.py(MyCobotHub)
    so101/              # ★新規
      driver.py         # lerobot ラップ（lazy import）
      kinematics.py     # URDF 由来
      safety.py poses.py constants.py hub.py(SO101Hub)
      profile.urdf      # TheRobotStudio から取得
scripts/
  server.py             # --robot {mycobot|so101} で profile 選択
  ui.html
```
> 補足: 1 プロセス＝1 アーム運用（同時駆動なし）。よって config は **起動時に確定** すれば良く、
> 全シグネチャに config を流す必要はない。Hub にぶら下げて注入する。

---

## 2. RobotConfig 抽象（設計の中核）

```python
@dataclass(frozen=True)
class RobotConfig:
    name: str                       # "mycobot320" | "so101"
    num_joints: int                 # 6 | 5（アーム自由度。グリッパ除く）
    urdf_links: list                # 親→子の剛体変換（FK 用）
    joint_limits: list[tuple]       # 各関節の (min,max) deg
    home_angles: list[float]
    tool_length_mm: float
    # safety
    link_radius_mm: float; self_clearance_mm: float; floor_z_mm: float
    collision_pairs: list[tuple]
    # gripper
    gripper: GripperSpec | None     # 開閉の単位（myCobot=state, so101=0-100）
```
- `kinematics` / `safety` / `ik_numeric` を「モジュール global 参照」から「config 引数」へ。
- `/kinematics` エンドポイントは選択中 profile の config を返す → UI は関節数を動的化。

---

## 3. Track A 実装 — SO101Hub（カスタムスタック駆動）

`SO101Hub(HubBase)` が lerobot を内包。要点（調査で確定した API）:

- 接続: `SO101Follower(SO101FollowerConfig(port="COMx", id=...))` → `connect(calibrate=False)`
- 読み: `robot.get_observation()` → `{"shoulder_pan.pos": deg, ..., "gripper.pos": 0-100}`
- 書き: `robot.send_action({...})`。または `robot.bus`（`FeetechMotorsBus`）で
  `sync_read("Present_Position")` / `sync_write("Goal_Position", {...})`
- トルク: `bus.enable_torque()` / `bus.disable_torque()`（= `release()`）
- **単位**: 関節は度（`use_degrees=True` 既定）、**グリッパは 0–100**。raw tick(0–4095) も `normalize=False` で可
- **ボーレート 1,000,000**、低電圧 7.4/12V（myCobot の 115200/24V を持ち込まない）

HubBase 実装の対応:
| HubBase メソッド | SO101Hub 実装 |
|---|---|
| `angles()` | `get_observation()` の関節 5 値を順序リスト化 |
| `send_angles_and_wait()` | dict 化して `send_action` → `Present_Position` で readback ポーリング |
| `power_ok()` | bus 接続・torque 状態 |
| `release()` | `disable_torque()` |
| `set_gripper(flag, speed)` | `gripper.pos` に 0/100 を書く |
| `solve_ik()` | placo IK が使えれば利用、無理なら position-only numeric |
| `live_coords()` | FK（URDF）or placo FK |
| `get_servo_diagnostics()` | Feetech レジスタ（電流/温度/電圧）読み |
| `frame_jpeg()` | 既存カメラ流用（手首カメラ再キャリブ要） |

- **dict↔順序リスト変換は 1 箇所に集約**（`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll` の正準順）。
- **VirtualSO101Hub** を同時に作る（ハード未着でも UI 開発を回すため。URDF から FK で擬似 readback）。

---

## 4. 環境・依存戦略（要・人間判断）

lerobot は **Python ≥3.12 / PyTorch ≥2.10** と重い。現リポは 3.10。
**推奨: lerobot を optional 依存にし lazy import。SO-101 用に専用 env を切る。**

- `requirements.txt` は触らず、`requirements-so101.txt`（or extras）を追加。
- `src/robots/so101/driver.py` は **モジュール冒頭で lerobot を import しない**。
  `SO101Hub.__init__` 内で遅延 import → `--robot mycobot` では PyTorch を一切ロードしない。
- Track A（位置制御）は **native Windows で動く見込み**（pyserial + Feetech SDK）。
- Track B（記録/学習/placo IK）は **WSL2(Ubuntu) or Linux** を推奨（TorchCodec/evdev/placo の Windows 摩擦回避）。
- `placo` の native Windows ビルド可否は **未確認** → ハード到着時に検証。不可なら IK は
  WSL 側 or 自前 position-only に倒す。

---

## 5. フェーズ計画（リスク順・早期検証重視）

### Phase 0 — ハード未着でできる準備（進行中）
- [x] SO-101 URDF を TheRobotStudio/SO-ARM100 から取得 → `src/robots/so101/so101_new_calib.urdf`
- [x] URDF から link 変換 / joint_limits / tool 変換を抽出 → `src/robots/so101/profile.py`
      （`tests/test_so101_kinematics.py` が URDF と突き合わせ検証、10 件緑）
- [x] SO-101 FK モジュール `kinematics.py`（URDF 規約準拠、profile 駆動）
- [x] SO-101 position-only 数値 IK `ik.py`（DLS、方位アラインシード + 乱数リスタート）
      FK 往復検証 40/40 緑（`tests/test_so101_ik.py`）。fast-fail 球 = 実測 max リーチ 546mm + 余裕で 560mm
- [x] `requirements-so101.txt` 追加（optional・別 env、lazy import 方針）
- [ ] 専用 env（Python ≥3.12）で `lerobot[feetech]` install 検証（import まで）← ローカル env 構築要
- [ ] `VirtualSO101Hub` 実装 → オフライン UI 確認（※ HubBase 共有層に触れるため Phase 1/2 と一体）

### Phase 1 — ディレクトリ移行（挙動変更なし・テスト緑維持）
- [ ] `src/core/` `src/vision/` `src/robots/mycobot/` へ移動
- [ ] import パス修正、`python -m unittest discover tests` で回帰確認
- [ ] myCobot 実機で `python scripts/server.py` が従来通り動くこと

### Phase 2 — 6-DoF 前提のパラメータ化
- [ ] `RobotConfig` 注入で kinematics/safety/ik_numeric の `range(6)` を一掃
- [ ] `/kinematics` を profile 駆動に、`ui.html` を `num_joints` ループ化
- [ ] VirtualSO101Hub + offline server で UI が 5-DoF アームを正しく描画・FK 表示

### Phase 3 — SO101Hub 本実装（Track A）
- [ ] `SO101Hub` を lerobot ラップで実装（lazy import）
- [x] SO-101 safety（`safety.py`：関節限界=URDF 厳密 / 床 / 自己干渉=暫定値、8 テスト緑）
      ※自己干渉クリアランスは実機で要再計測（profile.SAFETY、現状は誤検知<7%の保守値）
- [x] position-only IK 経路を確立（`ik.py`、placo は学習着手時に後付け）
- [ ] poses・gestures（5-DoF 版）

### Phase 4 — 実機ブリングアップ（ハード到着後）
- [ ] `lerobot-find-port` → `lerobot-setup-motors` → `lerobot-calibrate`（CLI、各 1 回）
- [ ] SO101Hub 経由で read/write/release 確認 → UI 手動ジョグ
- [ ] 手首カメラ再マウント → hand-eye 再キャリブ（`calibration.json` に SO-101 セクション）
- [ ] reach-grid 再計算（`scripts/reachable_grid.py` を profile 対応に）

### Phase 5 — LeRobot ネイティブ IL（Track B・WSL2）【延期】
> 模倣学習は当面やらない判断のため凍結。着手時に解凍する。
- [ ] （延期）WSL2 で leader+follower teleop
- [ ] （延期）`lerobot-record` → `lerobot-train`（ACT）→ `lerobot-rollout`
- [ ] （延期）`docs/SO101_LEROBOT.md`

### Phase 6 — ドキュメント/ハーネス更新
- [ ] `CLAUDE.md`: 「2 系統（アーム/ハンド）」→「3 系統（myCobot/SO-101/ハンド）」に改訂
- [ ] `.claude/memory/` に SO-101 ハードウェア・quirks メモ追加
- [ ] `robot-action` skill を多アーム対応に（どのアームか確認するフロー）
- [ ] リポジトリ改名（`mycobot-lab`→`robot-lab` 等）は最後に検討（git remote 影響）

---

## 6. 判断点（2026-06-04 解決済み）

1. ~~env 戦略~~ → **optional+lazy import** で確定。
2. ~~Track B の範囲~~ → **当面やらない**。Phase 5 凍結。
3. ~~ディレクトリ改名~~ → **許可**。
4. ~~vision 流用~~ → **手首カメラ流用**（hand-eye 再キャリブ）。
5. ~~電圧 SKU~~ → **12V 版**で確定。
6. **placo IK の Windows 可否**: 学習着手まで不要。当面は position-only numeric で進める。

### 残る確認点（ハード到着時）
- SO-101 の COM ポート番号・Feetech ドライバ board のジャンパ位置（B チャンネル）
- 手首カメラの SO-101 マウント方法（hand-eye 行列の再取得が必要）

---

## 7. 後回し（MVP に不要）
- `ik_gpu.py` の SO-101 対応（reach-grid 高速化は後）
- 高度な 5-DoF 姿勢選好ポリシー
- 2 アーム同時駆動（現状 1 プロセス 1 アームで十分）

## 参考 URL
- SO-101 docs: https://huggingface.co/docs/lerobot/main/en/so101
- LeRobot install: https://huggingface.co/docs/lerobot/installation
- motor bus API: https://huggingface.co/docs/lerobot/en/integrate_hardware
- 模倣学習: https://huggingface.co/docs/lerobot/en/il_robots
- ハード/URDF/BOM: https://github.com/TheRobotStudio/SO-ARM100
- so_follower.py: https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py
- kinematics(placo): https://github.com/huggingface/lerobot/blob/main/src/lerobot/model/kinematics.py
- SO-101 URDF(mirror): https://huggingface.co/haixuantao/dora-bambot/blob/main/URDF/so101.urdf
