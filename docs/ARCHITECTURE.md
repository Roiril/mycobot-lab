# Architecture — mycobot-lab

> AI エージェントが本プロジェクトを変更・拡張する前に読むドキュメント。
> 設計思想・主要抽象・依存方向・「なぜそうなっているか」を解説する。

## 1. 設計思想（最重要・優先順位順）

### 1.1 安全第一、二度と緩めない

24V 120W の卓上アームは人を怪我させる・物を壊す。ソフトの全層で **多重防護** を敷く：

| 層 | 防御内容 |
|---|---|
| 入力検証 | NaN/inf/型 / 速度 cap / 関節限界 |
| 計画 | `check_angles()` で全 waypoint を FK 評価（床干渉・自己干渉・関節限界）|
| 実行 | `motion_lock` で同時 motion 排他、各 waypoint 前に `power_ok()` |
| 監視 | `CurrentMonitor` daemon（10Hz）が過電流 → `abort_flag` |
| 復帰 | `abort_flag` 検知で waypoint loop が即停止 |

**緩めるなら同時に強める。** 例：cartesian モードを追加する時、firmware IK 任せにせず joint-space waypoint に compile し直して既存の安全層を全部通す。

### 1.2 「目標位置」ではなく「目標 pose」

位置 (x,y,z) だけで指定すると IK が自由に姿勢を選び、人間の直感と乖離する（手首固定で下から接近する等）。**位置 + 姿勢ポリシー**を必ず指定する。

| Pose kind | 意味 | デフォルト用途 |
|---|---|---|
| `extend_toward` | tool 軸が target 方向 | 指差し・自然な reach |
| `align_tool` | tool 軸が approach の反対 | 上から/横から把持 |
| `preserve` | 現在姿勢維持 | HOME 復帰、軽微な位置調整 |
| `explicit` | rxyz/quat 直接指定 | 細かい制御 |
| `any` | IK 任意（opt-in） | 探索・debug 用 |

### 1.3 関節空間が真実、cartesian は補助

ロボット制御の最終出力は **関節角度** だけ。cartesian 入力は内部で IK → 関節 waypoint に変換して、joint-space 経路計画と safety check を通してから実行する。`send_coords`（firmware の直接 cartesian 移動）は使わない — IK 解が予測不能で safety check を bypass するから。

### 1.4 AI 呼び出しを一級市民として設計

人間 UI と並び（または上位に）AI エージェントが呼ぶ。API 設計の原則：

- **stateless calls + rich state echo**：全 response に現在角度・tip・通電状態を含める
- **structured errors + retry_hints**：失敗時に code/message/diagnostics/retry_hints を返す
- **dry_run defaults vary by risk**：interaction は default dry_run=true、move は false
- **discoverable**：`/kinematics` エンドポイントが DH・限界・grasp 定数を配信し UI/AI が boot 時に取得

### 1.5 offline でも本物に近い動作

`VirtualHub` は単なる stub ではない：数値 IK が動き、過電流の fault injection 可（`VHUB_FAULT=overcurrent`）、UI フル動作。**実機なしで設計・テスト・review を完結できる** ことを意図的に守る。

## 2. モジュール構成

### 2.1 ディレクトリ map

```
src/arm/
├── constants.py          # 全モジュール共通の定数（限界、速度、寸法、閾値）
├── kinematics.py         # Modified DH 順運動学（pure）
├── safety.py             # 関節限界・床・自己干渉（pure, FK を使う）
├── planner.py            # 関節空間 waypoint 経路 + 各 step 検証
├── path_cartesian.py     # cartesian 直線/lift-translate-lower + quaternion slerp
├── ik_path.py            # cartesian → joint chain（seed-IK 連続性、wrap-aware）
├── ik_numeric.py         # DLS Jacobian IK + multi-seed retry + tool-frame relaxation
├── pose_resolver.py      # Pose union → (rx,ry,rz) 解決
├── current_monitor.py    # サーボ電流監視 daemon
├── client.py             # pymycobot 薄ラッパ（接続・電源・close）
├── hub.py                # HubBase ABC + Hub (実機) / VirtualHub (offline)
└── poses.py              # 名前付き姿勢（HOME 等）

scripts/
├── server.py             # HTTP サーバ（HTTP only、ロジックは arm/ に委譲）
├── ui.html               # three.js + vanilla JS、単一ファイル
├── check.py / move.py    # 古い手動 CLI（保守）
└── sweep.py              # 接続診断

tests/
└── test_*.py             # safety, kinematics, planner, ik_*, pose_resolver, grasp
                          # 全 pure function + offline E2E（実機不要で 56 件 pass）
```

### 2.2 依存方向（重要）

```
                ┌── safety.py ───┐
                │                │
kinematics.py ──┼── planner.py ──┼── hub.py ──── client.py (pymycobot)
                │                │      │
                └── ik_*.py ─────┘      │
                                        │
pose_resolver.py ──────────────────────/│
                                        │
                  path_cartesian.py ───/
                                        │
                                    server.py
                                        │
                                    ui.html
```

ルール：
- `kinematics.py` は **pure**（外部依存ゼロ、numpy 含めず純 Python）。すべての層の土台。
- `safety.py` は kinematics に依存するが **pure**。テストで全網羅できる。
- `hub.py` 以下が初めてハードウェア・状態を持つ。
- `server.py` は HTTP transport のみ。判断は `arm/` 内に。
- `ui.html` は server の JSON API しか触らない。

**逆向きの import は禁止**（例：kinematics が hub を import する等）。循環参照と test 不能を生む。

## 3. 主要抽象

### 3.1 HubBase（ABC）

ハード制御と仮想実装のインターフェース。テストでは VirtualHub、本番は Hub。

```python
class HubBase(abc.ABC):
    monitor_enabled: bool
    motion_lock: threading.Lock      # whole motion sequence の排他
    io_lock: threading.Lock          # 個別シリアル read/write の排他
    abort_flag: threading.Event      # 全実行 loop が読む割り込み旗

    angles() -> list[float] | None
    power_ok() -> bool
    send_angles_and_wait(angles, speed) -> (reached, actual_angles)
    solve_ik(coords6, seed) -> angles | None             # firmware → 数値 fallback
    solve_ik_with_mode(coords3_or_6, seed) -> (angles, mode)
        # mode: "full"|"firmware"|"relaxed_roll"|"position_only"|"failed"
    live_coords() -> coords6 | None                       # 現在 tip pose
    home_blocking(speed) -> None                          # validated path で HOME へ
    release() -> None                                     # サーボ脱力（落下注意）
    get_currents() -> list[int] | None                    # mA, per joint
    frame_jpeg() -> bytes | None                          # camera frame
    shutdown() -> None
```

### 3.2 Pose（discriminated union）

```python
{kind: "extend_toward", target: [x,y,z], roll_deg?: float}
{kind: "align_tool", approach: "+x|-x|+y|-y|+z|-z" or [vx,vy,vz], roll_deg?: float}
{kind: "preserve"}
{kind: "explicit", euler_xyz: [r,p,y] or quat: [w,x,y,z]}
{kind: "any"}        # opt-in only
```

`pose_resolver.resolve_pose(pose, position, current_angles) -> (rx,ry,rz) | None`。
`None` を返す = position-only IK で良い（preserve / any / extend_toward の距離不足時）。

### 3.3 Planner レイヤ

```
[cartesian入力]──path_cartesian──>[(x,y,z,rx,ry,rz) waypoints]
                                          │
                                  ik_path.plan_ik_path
                                  (seed-IK + 連続性検証 + 細分化)
                                          ▼
                                  [joint waypoints]
                                          │
                                  planner.plan_and_validate
                                  (各 step に check_angles)
                                          ▼
                              hub.send_angles_and_wait × N
```

8° の関節 step を上限に分割、各 waypoint で safety check、各 step で abort_flag 監視。

### 3.4 Safety check_angles

```python
check_angles(angles) -> (ok, msg, bad_joints)
```

検証内容：
1. 関節限界 (JOINT_LIMITS) 内か
2. 全リンク（J0-J6 + ツール）が床 + LINK_RADIUS のクリアランス確保
3. 非隣接リンク対が SELF_CLEARANCE 未満で接近していないか

`bad_joints` は 1-based の問題関節 index リスト。UI/AI が該当 joint をハイライトできる。

### 3.5 Current Monitor

```python
CurrentMonitor(read_currents_fn, on_overcurrent_fn,
               threshold_mA=1500 or 800 (safe-mode),
               poll_hz=10, sustained=3)
```

10Hz でサーボ電流取得、3 連続で閾値超 → `abort_flag.set()`。
`.calibrated` ファイルが project root に無ければ自動的に safe-mode（800mA）。

### 3.6 IK 戦略

```
1. firmware solve_inv_kinematics      (高速、不安定)
       ↓ fail
2. 数値 IK with full orientation, multi-seed retry
       ↓ fail
3. 数値 IK with tool-frame roll relaxation (±15-45°)
       ↓ fail
4. 数値 IK position-only             (姿勢諦め、警告付き)
       ↓ fail
5. SOLVER_NONCONVERGENT
```

`solve_ik_with_mode` がこれを実行し、どの段階で成功したか `mode` で返す。
UI/AI は mode を見て「要求姿勢が honored されたか」を判断する。

## 4. 状態と並行性

### 4.1 ロック階層

```
motion_lock (粗) > io_lock (細)
```

`motion_lock`：1 つの motion sequence（複数 waypoint）全体を排他。`/move` `/home` `/grasp_sequence` が取得。
`io_lock`：個別シリアルコマンドを排他（angles 読み、send_angles、get_currents）。

**逆順獲得は禁止**（deadlock リスク）。常に motion_lock → io_lock の順。

### 4.2 abort_flag の semantic

set される条件：
- ユーザーが `/abort` 押下（UI: Esc キー / ABORT ボタン）
- CurrentMonitor が過電流検知
- send_angles_and_wait 内で power_ok() が False になった
- `/release` が motion 中（abort 先発で waypoint loop を止めてから release）

読まれる場所：
- waypoint loop の各 iteration 冒頭
- send_angles_and_wait の poll 内（100ms 毎）
- home_blocking の各 step

clear されるタイミング：
- 各 `/move` `/home` `/grasp_sequence` の開始時

### 4.3 drift detection

`expected_current` パラメータで「クライアントが想定する現在角」を送る。実機角と 3° 超ズレで 409 拒否。
意図：UI の preview と実機が乖離した状態で APPLY されるのを防ぐ（ユーザーがブラウザ放置中に物理的にぶつけられた等）。

## 5. UI（scripts/ui.html）の構造

単一ファイル three.js アプリ。

### 5.1 3D オブジェクト

| Object | 色 | 役割 |
|---|---|---|
| `armCurrent` | 青不透明 | 実機状態（/angles ポール、600ms）|
| `armTarget` | 橙半透明 | IK プレビュー（state.target から FK）|
| `goalSphere` | 黄 | ユーザーの drag target、ゴール位置 |
| `targetObj` | 橙ワイヤフレーム | 把持対象物体（半径スライダで size 調整）|
| `approachArrow` / `liftArrow` | 橙↓ / シアン↑ | 接近・引き上げ経路 |
| `gridHelper` / `axesHelper` | グレー / RGB | スケール参照 |

### 5.2 State machine

```js
state = {
  current: [j1..j6],          // poll 由来、信頼できる
  target:  [j1..j6],          // ユーザー目標（slider/IK 結果）
  power: bool,
  busy: bool,                 // motion 実行中
  ikFailed: bool,             // 最後の IK 試行が失敗
  userTouched: bool,          // ユーザーが target を操作したか
  busyWatchdog,               // 120s タイムアウト
  lastDriftWarn,              // drift 警告 rate limit
}
posePolicy = 'extend_toward' | 'align_+z' | ... | 'preserve'
```

precedence ルール：
- スライダ操作 → state.target 上書き、pose policy active 解除
- ゴール球ドラッグ → policy で IK → state.target 上書き
- IK ボタン / cartesian execute → 同上

### 5.3 ボタン階層

| Tier | 例 | スタイル |
|---|---|---|
| 主操作 | APPLY | 緑、フル幅、大 |
| 主操作（破壊的）| ABORT | 赤、フル幅、大、常時表示 |
| 副操作 | HOME / Sync / cartesian execute | 青、コンパクト |
| 破壊的副操作 | 脱力 | 赤、確認ダイアログ必須 |

## 6. API 設計の原則（AI 呼出し前提）

詳細は [AGENT_API.md](AGENT_API.md) 参照。要点：

- **3 verb**: `set_joints` / `move_tip` / `interact`（grasp/place 等）
- **stateless + state echo**
- **structured errors with retry_hints**
- **dry_run defaults by risk**
- **single source of truth**: `/kinematics` で DH・限界・定数配信、UI/AI が boot 時取得

## 7. 拡張時のチェックリスト

新機能を追加する時に確認：

- [ ] `kinematics.py` に依存追加が無いか（追加するなら pure であること）
- [ ] `safety.py` の `check_angles` で新しい配置パターンが弾かれないか
- [ ] 新 endpoint は `motion_lock` を取るか（実行系の場合）
- [ ] エラー応答は `{code, message, diagnostics, retry_hints}` 構造化されているか
- [ ] offline モード（VirtualHub）でも動作するか
- [ ] ユニットテストを追加したか（特に safety 系）
- [ ] `/kinematics` に新定数を追加する場合 UI も更新したか
- [ ] CLAUDE.md / docs の説明が古くなっていないか

## 8. 既知の制限と注意

- **DH パラメータの tool 方向が approximate**：J6 frame z 軸に沿って TOOL_LENGTH を伸ばしているが、HOME で実測すると ±30mm 程度ずれる。`FK_TOOL_SLOP` で FLOOR_Z に含めて補正している。実機での tool tip 真値は `mc.get_coords()` 由来 (`live_coords`) を使うこと。
- **firmware IK は不安定**：失敗時の動作が不定（-1 を返す、別解を返す、freeze）。常に `solve_with_retries` のフォールバック前提で運用。
- **myCobot 320 には力センサ無し**：「触れた瞬間に止める」はできない。CurrentMonitor 300ms 遅延前提。
- **gripper actuation は未実装**：Phase 3。`grasp` は approach + lift のみで、tip は object surface + clearance で停止する。
- **接近方向は top-down のみ**：`/grasp_sequence` は approach="+z" 固定。横/前は将来拡張。

## 9. 参考

- [AGENT_API.md](AGENT_API.md) — AI エージェント向け API リファレンス
- [.agent/rules/safety.md](../.agent/rules/safety.md) — 物理動作の安全規約
- [.agent/rules/connection-troubleshooting.md](../.agent/rules/connection-troubleshooting.md) — 接続トラブルシュート
- [CLAUDE.md](../CLAUDE.md) — プロジェクト規約とエージェント指示
