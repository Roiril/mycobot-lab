# AI Agent API Reference — mycobot-lab

> AI エージェントが myCobot 320 を制御するための API リファレンス。
> 設計思想は [ARCHITECTURE.md](ARCHITECTURE.md) を参照。

## 0. 開始前チェックリスト

任意の motion を起こす前に **必ず**確認：

```
1. /power           → サーボ通電? (false なら作業中断、人間に依頼)
2. /angles          → 現在位置取得（後の expected_current に渡す）
3. /currents        → 監視 ON 確認、過電流発生なし
4. /kinematics      → 定数取得（初回 boot 時 1 回でよい、キャッシュ可）
```

これを怠ると、stale state / 通電ミス / 監視 OFF で実行する事故が起きる。

## 1. 3 つの主要動作 verb

エージェントが選ぶべき motion API は **3 種類** のみ：

| Verb | 用途 | 主要パラメータ |
|---|---|---|
| `POST /move`            | 関節角度直接指定（HOME 復帰・校正・undo） | `angles[6]`, `speed`, `expected_current` |
| `POST /move_cartesian`  | tip を XYZ へ移動（pose ポリシー込み） | `x,y,z,rx?,ry?,rz?`, `mode`, `speed`, `expected_current` |
| `POST /grasp_sequence`  | 物体相対の多段把持モーション | `x,y,z, radius`, `approach_offset?`, `lift_offset?`, `speed` |

**ガイドライン：**
- 「とりあえず HOME に戻したい」 → `/home` ショートカット使用
- 「特定の関節姿勢にしたい」（calibration, undo） → `/move`
- 「tip をここに置きたい」 → `/move_cartesian` または下記 IK プレビュー経由
- 「物体を掴みたい」 → `/grasp_sequence`（current は approach + lift のみ、grip 未動作）

## 2. 推奨ワークフロー：preview-then-commit

衝突や姿勢不可を実機で発生させる前に、必ず IK preview で検証する：

```
[1] state = await GET /angles            # 信頼できる現状
[2] ik   = await POST /solve_ik {        # この pose で IK 可解か?
        x, y, z,
        pose: {kind:"extend_toward", target:[x,y,z]}  # または他のポリシー
    }
[3] if not ik.ok:
        # ik.error.code を見て retry_hints を試す
        ...
[4] # ok なら ik.angles を /move に渡す
    res = await POST /move {
        angles: ik.angles, speed: 20,
        expected_current: state
    }
[5] # 失敗パターンに応じてリカバリ（後述）
```

**なぜ preview 必須:**
- `/move_cartesian` を直接呼ぶと、cartesian IK + waypoint 計画が一気に走り、失敗時に「どこで」失敗したか分かりにくい
- preview なら orientation 不可・位置不可・safety 違反を切り分けて検知できる
- `expected_current` でユーザーが裏で動かしたケースを検出可能

## 3. Pose ポリシー（手首向きの指定）

`/solve_ik` `/move_cartesian` に渡す `pose` オブジェクト。**毎回必ず明示**：

```jsonc
// 自然な指差し reach（推奨デフォルト）
{"kind": "extend_toward", "target": [x, y, z]}

// 上から把持（tool 軸が下を向く）
{"kind": "align_tool", "approach": "+z"}

// 横から把持（approach 軸の反対側に tool が向く）
{"kind": "align_tool", "approach": "+x"}    // +x 側から → tool 軸 -x
{"kind": "align_tool", "approach": [1,1,0]} // 任意ベクトル可

// 任意 roll 追加（approach 軸まわりの回転、deg）
{"kind": "align_tool", "approach": "+z", "roll_deg": 30}

// 現在の手首向きを維持（軽い位置調整用）
{"kind": "preserve"}

// 既存の rx, ry, rz を直接（pymycobot 互換）
{"kind": "explicit", "euler_xyz": [rx, ry, rz]}
{"kind": "explicit", "quat": [w, x, y, z]}

// IK 任せ（opt-in、結果が予測不能なので推奨せず）
{"kind": "any"}
```

**選び方：**
- 何もしないなら `extend_toward` がベース（人間の自然な reach）
- 物体把持なら `align_tool` で approach 明示
- 微調整（5-30mm の位置補正）なら `preserve` で十分

## 4. エラー応答と retry_hints

`/solve_ik` 失敗時の構造化エラー：

```jsonc
{
  "ok": false,
  "ikMode": "failed",  // または "position_only" 等
  "error": {
    "code": "OUT_OF_REACH",  // 下記コード一覧参照
    "message": "位置 (900, 0, 500) はアーム到達範囲外",
    "diagnostics": {
      "distance_from_base_mm": 1029.6,
      "approx_max_reach_mm": 380
    },
    "retry_hints": [
      {"action": "move_closer", "patch": null,
       "rationale": "R=1030mm がアーム reach (~380mm) を超えている"}
    ]
  }
}
```

### 4.1 エラーコード一覧

| Code | 意味 | 推奨対応 |
|---|---|---|
| `OUT_OF_REACH` | 位置が物理的に到達不可 | 目標を base 寄りに、より近い座標へ |
| `ORIENTATION_INFEASIBLE` | 位置は OK だが姿勢不可 | `pose: preserve` に切替 / 別 approach axis 試行 |
| `SAFETY_VIOLATION` | IK 解が check_angles で NG（床/限界/自己干渉）| 目標を上方に / `pose: preserve` / approach を変える |
| `SOLVER_NONCONVERGENT` | 数値 IK が収束しなかった（稀） | seed を変えて再試行（軽く現在角を動かす）|

### 4.2 retry_hints の使い方

エージェントは `retry_hints[]` を順に試す。各 hint は：

```jsonc
{
  "action": "use_preserve_pose",
  "patch": {"pose": {"kind": "preserve"}},
  "rationale": "姿勢を捨てて位置だけ到達"
}
```

元 request に `patch` をマージして再 POST する。3-5 試行で諦め、人間にエスカレートする。

### 4.3 ikMode の解釈

`/solve_ik` 成功時も `ikMode` を確認：

| Mode | 意味 | 信頼度 |
|---|---|---|
| `firmware` | firmware IK で要求姿勢 100% 達成 | 高 |
| `full` | 数値 IK で要求姿勢達成 | 高 |
| `relaxed_roll` | 姿勢を ±15-45° 緩和して達成 | 中（把持精度に影響あり）|
| `position_only` | 姿勢を諦め位置のみ達成 | 低（姿勢無視）|

**重要：** `position_only` で帰ってきた場合、要求した手首向きは反映されていない。把持シーケンスでは fatal、reach motion なら許容可。

## 5. 動作 sequence の例

### 5.1 単純な tip 移動（最頻出パターン）

```jsonc
// 1. 現状取得
GET  /angles → {"angles": [0,0,-90,0,0,0], ...}
GET  /power  → {"ok": true}

// 2. IK preview（指差しで自然な reach）
POST /solve_ik {
  "x": 200, "y": -100, "z": 250,
  "pose": {"kind": "extend_toward", "target": [200,-100,250]}
}
→ {"ok": true, "angles": [25, -10, -75, ...], "ikMode": "firmware"}

// 3. 実行（drift 検知あり）
POST /move {
  "angles": [25, -10, -75, ...],
  "speed": 20,
  "expected_current": [0, 0, -90, 0, 0, 0]
}
→ {"angles": [25.1, -9.8, -74.9, ...], "elapsed": 2.4,
   "peakCurrents": [80, 120, 110, 90, 80, 85], "monitorEnabled": true}
```

### 5.2 物体把持（grasp_sequence）

```jsonc
// 1. 状態取得 (省略)

// 2. 物体表面到達性確認
POST /solve_ik {
  "x": 180, "y": 50, "z": 80,         // 物体中心
  "pose": {"kind": "align_tool", "approach": "+z"}
}
→ {"ok": true, "angles": [...], "ikMode": "full"}

// 3. 把持シーケンス実行（pre-grasp → approach → lift, slow）
POST /grasp_sequence {
  "x": 180, "y": 50, "z": 80,
  "radius": 25,                        // 物体半径 → tip 停止位置に影響
  "speed": 15,
  "expected_current": [...]
}
→ {
    "stages": ["pre-grasp", "approach", "lift"],
    "nJointWp": 11,
    "elapsed": 6.2,
    "graspZ": 110,                     // tip 実際の停止 z（z + radius + clearance）
    "peakCurrents": [...]
  }
```

### 5.3 エラーからのリカバリ

```jsonc
// 試行 1: 自然な指差し
POST /solve_ik {x: 50, y: 0, z: 100, pose: {kind: "extend_toward", target: [50,0,100]}}
→ {"ok": false, "error": {
     "code": "ORIENTATION_INFEASIBLE",
     "retry_hints": [
       {"action": "use_preserve_pose", "patch": {"pose": {"kind":"preserve"}}},
       {"action": "use_align_top", "patch": {"pose": {"kind":"align_tool","approach":"+z"}}}
     ]
   }}

// 試行 2: retry_hint[0] を適用
POST /solve_ik {x: 50, y: 0, z: 100, pose: {"kind":"preserve"}}
→ {"ok": true, "angles": [...], "ikMode": "position_only"}
   // 姿勢諦めで成功 → reach motion なら OK、把持なら諦める
```

### 5.4 中断と復帰

```jsonc
// 何か変だと思ったら即 abort
POST /abort → {"ok": true}
   // 進行中の motion を即停止（次の waypoint 前で break）

// 安全姿勢へ復帰
POST /home   → {"angles": [0,0,-90,0,0,0], "elapsed": 3.2,
                "monitorReenabled": true}
```

### 5.5 監視 OFF を要する高度作業（debug 用、推奨せず）

```jsonc
POST /monitor {"enabled": false}
   → {"warning": "過電流監視 OFF — 衝突しても自動停止しません"}
// ... 試験的な motion ...
POST /monitor {"enabled": true}
// /home でも auto re-enable される（防御）
```

## 6. 全エンドポイント一覧

### 状態取得（GET、安全）

| Path | Returns |
|---|---|
| `/angles` | `{angles:[6], offline:bool}` |
| `/coords` | `{coords:[xyz,rxryrz] or null, angles:[6]}` |
| `/power` | `{ok:bool}` |
| `/currents` | `{currents:[mA×6], monitor_enabled, threshold_mA, poll_hz, sustained_polls}` |
| `/fk?angles=a1,..,a6` | `{joints:[(x,y,z)×7], tip:[x,y,z]}` |
| `/kinematics` | `{dh, joint_limits, tool_length, floor_z, link_radius, grasp_*, target_radius_*, gripper_present}` |
| `/frame.jpg` | カメラ JPEG（end-effector 取付想定）|

### 計画（POST、無害な計算のみ）

| Path | Body | Returns |
|---|---|---|
| `/check` | `{angles:[6]}` | `{ok, msg, badJoints:[1-based]}` |
| `/solve_ik` | `{x,y,z, pose?:{...}, rx?,ry?,rz?}` | `{ok, angles?, msg, badJoints, resolvedOrientation, achievedOrientation, ikMode, error?}` |

### 実行（POST、副作用あり）

| Path | Body | Returns | Lock |
|---|---|---|---|
| `/move` | `{angles, speed, expected_current?}` | `{angles, nWaypoints, elapsed, peakCurrents, monitorEnabled}` | motion_lock |
| `/move_cartesian` | `{x,y,z,rx?,ry?,rz?, mode:auto\|linear\|lift, speed, expected_current?}` | `{angles, nCartWp, nJointWp, elapsed, peakCurrents}` | motion_lock |
| `/grasp_sequence` | `{x,y,z, radius, approach_offset?, lift_offset?, speed, expected_current?}` | `{angles, stages, nJointWp, elapsed, graspZ, peakCurrents}` | motion_lock |
| `/home` | `{}` | `{angles, elapsed, monitorReenabled}` | motion_lock |
| `/abort` | `{}` | `{ok:true}` | (lock 取らない、即実行)|
| `/release` | `{force?:bool}` | `{ok, warning}` | motion_lock または abort 先発 |
| `/monitor` | `{enabled:bool}` | `{ok, enabled, warning?}` | (lock 不要) |

### HTTP コード semantics

| Code | 意味 |
|---|---|
| 200 | 成功（または ok:false の structured error）|
| 400 | リクエスト不正（型違反、範囲外、必須欠如）|
| 409 | motion_lock 競合 / drift 検出（再 preview して再送）|
| 422 | 計画段階で安全 NG（target または waypoint が check_angles fail）|
| 499 | 実行中に abort（ユーザー or over-current）|
| 503 | 通電 NG / shutdown / readback timeout（ハード状態問題）|
| 500 | 想定外例外 |

## 7. 並行性とロック

- **複数 motion 同時不可**：`motion_lock` で排他。busy 時は 409 即返却。
- **/abort は即時**：lock 取らないので競合時も実行可能。
- **/release は abort 先発**：force=true で motion 中でも強制可（落下注意）。
- **state 取得系は同時可**：io_lock で個別 serial op を直列化。

## 8. 速度・タイミング

| 設定 | デフォルト | 上限 | 備考 |
|---|---|---|---|
| `MAX_SPEED` | - | 40 | server.py 起動時に `--max-speed` で上書き可、最大 80 |
| `/move` `/move_cartesian` 速度 | 20 | 40 | クライアントが指定 |
| `/grasp_sequence` 速度 | 15 (`GRASP_APPROACH_SPEED_DEFAULT`) | 25 (`GRASP_APPROACH_SPEED_MAX`) | 把持は意図的に slow |
| WAYPOINT_TIMEOUT | 4 秒 | - | 1 waypoint 到達待ち |
| WAYPOINT_TOLERANCE | 2 deg | - | 到達判定の許容 |

motion 全体の elapsed 目安：
- `/move` 30°移動: ~3 秒
- `/move_cartesian` 100mm 直線: ~5 秒
- `/grasp_sequence` (200mm 上空→ object → 100mm 引上げ): ~10 秒

## 9. 認証

- デフォルト bind は `127.0.0.1`（loopback のみ）→ 無認証
- LAN 公開時は **`--bind 0.0.0.0 --token <secret>` 必須**（refusing で起動拒否）
- write endpoint（/move /home /release /abort）は `X-Auth-Token: <secret>` header 必須

## 10. 「やってはいけない」一覧

- ❌ `/move` を `expected_current` 無しで連続呼出し（stale state で誤動作）
- ❌ `pose: {kind:"any"}` をデフォルトに（IK 任意 = 予測不能）
- ❌ `/monitor` を OFF にしたまま放置（次の /home まで OFF のまま、ただし auto re-enable あり）
- ❌ `/release` を motion 中に force なしで複数回（必ず /abort 先）
- ❌ ikMode を確認せず `angles` を信用（position_only かもしれない）
- ❌ `/grasp_sequence` で grasp 動作期待（current は approach + lift のみ、tip は object 表面 +5mm で停止）
- ❌ `expected_current` を `state.target` から作る（→ 実機との乖離で常に 409、`/angles` から取る）

## 11. AI が呼ぶ前に唱える 1 文

> 「目標位置はここ、姿勢は (preserve または 〇〇)、approach は (なし または 〇〇)、終了動作は (なし または grasp/place)」

これを言語化できないなら、まだ `/solve_ik` で preview すべきタイミング。

## 8. Vision API (Phase 1)

視覚で物体を見つけて world 座標に投影する read-only API。motion を起こさないので動的安全性とは独立。

### 8.1 エンドポイント

| Path | Method | Returns |
|---|---|---|
| `/cameras` | GET | `{cameras:[{id,role,resolution,calibrated,placeholder}], workspace:{table_z_mm,table_z_uncertainty_mm}}` |
| `/frame.jpg?cam=<id>` | GET | JPEG。`cam` 省略時は wrist（後方互換）。不明 id は 404 + JSON |
| `/perceive` | POST | `{ok, objects?, error?, vlm_latency_ms, recommended_speed?}` |

### 8.2 `/perceive` request / response

**Request:**
```jsonc
{
  "query": "コップ",                  // required
  "cameras": ["wrist"],               // optional, default = 全 calibrated
  "use_table_plane": true,            // optional, default true
  "confidence_threshold": 0.5,        // optional, default 0.5
  "consensus": false,                 // Phase 2 で実装、Phase 1 は受信のみ
  "refine": false                     // Phase 2、Phase 1 は ignore
}
```

**Success response:**
```jsonc
{
  "ok": true,
  "objects": [
    {
      "label": "コップ",
      "world_xyz_mm": [200.0, 0.0, 30.0],
      "radius_mm": 25.0,
      "confidence": 0.83,
      "depth_uncertainty_mm": 18.5,
      "source_cam": "wrist",
      "frame_id": "wrist_20260524_103045_123",
      "bbox_px": [320, 240, 80, 80],
      "estimated_size_class": "medium"
    }
  ],
  "vlm_latency_ms": 1840.0,
  "elapsed_ms": 1920.5,
  "recommended_speed": 20,             // confidence>=0.8→20, 0.5-0.8→10, <0.5→5
  "consensus_used": false
}
```

### 8.3 エラーコード

`/solve_ik` と同じ `{code, message, diagnostics, retry_hints[]}` 構造化エラー：

| Code | 意味 | 主な retry_hints |
|---|---|---|
| `OBJECT_NOT_FOUND` | detection 0 件 | observe_from_another_angle / narrow_query / use_different_camera |
| `LOW_CONFIDENCE` | top 候補 confidence < threshold | observe_from_another_angle / lower_confidence_threshold |
| `MULTIPLE_AMBIGUOUS` | top-2 差 < 0.15 かつ両者 > threshold | narrow_query / lower_confidence_threshold |
| `OCCLUDED` | bbox 端 5% 内 or 面積 < 100px | observe_from_another_angle / zoom_in |
| `DEPTH_UNCERTAIN` | depth_uncertainty > 50mm | observe_from_overhead / refine_with_known_object_height |
| `VLM_API_ERROR` | Anthropic API 例外 | retry_after_delay / fallback_to_fixture |
| `CALIBRATION_MISSING` | calibration.json 無し or 対象カメラ無し | configure_camera |

各 hint は `{action, patch, rationale}` 構造で、`/solve_ik` の retry_hints と同じ運用：元 request に `patch` をマージして再 POST する。

### 8.4 AI 呼出しシーケンス（grasp の前段）

```jsonc
// 1. 観察位置に移動（人間が事前に置いた observe_pose、または HOME）
POST /move {angles: OBSERVE_ANGLES, speed: 25, expected_current: [...]}

// 2. 視覚で物体を探す
POST /perceive {query: "コップ", confidence_threshold: 0.6}
→ {ok: true, objects: [{world_xyz_mm: [200,0,30], radius_mm: 25, ...}],
   recommended_speed: 20}

// 3. IK preview（把持姿勢で到達可確認）
POST /solve_ik {
  x: 200, y: 0, z: 30,
  pose: {kind: "align_tool", approach: "+z"}
}
→ {ok: true, ikMode: "firmware", angles: [...]}

// 4. 把持シーケンス（perceive の recommended_speed を採用）
POST /grasp_sequence {
  x: 200, y: 0, z: 30, radius: 25,
  speed: 20,
  expected_current: [...]
}
```

### 8.5 注意 (Phase 1 制限)

- **placeholder calibration**: `data/calibration.json` の初期値は仮値。intrinsics と hand_eye_T_ee_cam を `scripts/calibrate_intrinsics.py` で校正するまで、world_xyz_mm は信用できない（実機で +20-50mm ズレうる）。
- **手首カメラ 1 台のみ**: overhead カメラ等は calibration.json に追加可能だが Phase 1 は wrist 推奨。
- **VLM レイテンシ**: 1-3 秒。連続 perceive は避ける。
- **consensus は未実装**: 2 回連続検出 + 一致確認は Phase 2。
- **refine は未実装**: 検出後の zoom-in 再撮影は Phase 2。

## 12. 参考

- [ARCHITECTURE.md](ARCHITECTURE.md) — 設計思想と内部構造
- [.agent/rules/safety.md](../.agent/rules/safety.md) — 物理動作の安全規約
- [CLAUDE.md](../CLAUDE.md) — エージェント全般指示
