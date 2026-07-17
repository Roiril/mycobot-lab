# VR ハンドトラッキング遠隔操作 仕様

WebXR（Meta Quest 系）で手を使って myCobot 320 をリアルタイム操作する機能の仕様。
実装は [scripts/ui.html](../scripts/ui.html) の `xr.*` / `_xr*` 群、サーバ側は [scripts/server.py](../scripts/server.py) の `/jog`。

> 本書は「現在の挙動」を記述する生きた仕様。チューニング調整の基準点。最終更新: 2026-06-01。

---

## 1. 概要

- **目的**: HMD を被り、右手の動きでアーム先端（または関節）を相対操作する。
- **必須機能**: `immersive-vr` + `hand-tracking` + `local-floor`（`_xrStart` の `requiredFeatures`）。
- **対応**: Quest Browser 等 WebXR ハンドトラッキング対応ブラウザのみ。`navigator.xr` 無しは即エラー。
- **操作主体**: **右手のみ**。左手は可視化（手骨スフィア）だけで制御に使わない。

---

## 2. セッションのライフサイクル

### 開始（`_xrStart`）
1. `immersive-vr` セッション要求 → three.js `renderer.xr` に接続。
2. scene を **mm→m スケール（0.001）** + Z-up→Y-up 回転で VR 空間に配置。
3. 実機を **開始姿勢 `XR_START_POSE = [90, 15, -100, -15, -50, 0]`** へ移動（`/move`, speed=`XR_START_SPEED=30`）。
   - この姿勢は「手首が特異点から離れ（J5=-50）、先端が中央寄り（r_xy≈313mm, z≈468mm）で全方向に到達余裕がある」ように選定（`/solve_ik` 実測で ±60mm 全6方向到達確認済み）。
4. **初フレームで HMD 基準にアーム基部を再配置**（`xr._needPlace`）。HMD 前方＝アーム base +Y 方向に合わせ、目線の `XR_VIEW_DROP_M=0.45m` 下・`XR_VIEW_FWD_M=0.6m` 前に基部を置く。
5. 実機現在角を `/real_angles` から **10Hz** でポーリングし、不透明シアンのゴーストで表示（`_xrRealArmPoll`）。
6. cue ステップ protocol は **既定 OFF**（`xr._dbgOn=false`）。ピンチは素直な追従 ON/OFF トグル。

### 終了（`_xrOnEnd`）
- engage 解除、ゴースト非表示、scene スケール/回転を元に戻す、HUD 非表示、テレメトリ flush。

---

## 3. 制御モード（2 種）

UI パネル「制御方式」で切替。`xr.ctrlMode` に保持（localStorage 永続）。**既定 = `follow`**。

| モード | ボタン | 方式 | 制御対象 |
|---|---|---|---|
| `follow` | IK追従 | レート制御（速度積分）+ クライアント IK | 先端の **位置**（向きは既定 OFF）|
| `joint` | 直接関節 | 手姿勢→関節へ直接マップ | **J1/J2/J5/J6**（J3/J4 固定）|

毎フレーム `_updateXR(frame)` から、engage 中のみ該当モードの更新関数を呼ぶ。

---

## 4. クラッチ（engage）

手の動きを常時アームに流すのではなく、**ピンチでトグルする ON/OFF クラッチ**式。

- **ピンチ検出**: 右手の thumb-tip↔index-finger-tip 距離。ヒステリシス `PINCH_ENTER_M=0.025`（25mm で握り）/ `PINCH_EXIT_M=0.035`（35mm で離し）。
- **トグル**: ピンチ立ち上がりで `engaged` を反転（`_xrSetEngaged`）。デバウンス 600ms。
- **アーミング**: VR 開始直後の握りっぱなしで即追従しないよう、一度指を離すまで無効（`_pinchArmed`）。
- **engage 時**: 現在の手 pose を「中立点」として捕捉（follow=`rateCenterPos/Quat` と目標先端=実機 FK、joint=`jointAnchor`）。1 engage = 1 **trial**（テレメトリ単位）。
- **トラッキング消失で自動フリーズ**: engage 中に右手が `XR_TRACK_LOST_MS=600ms` 見えなくなったら自動的に engage 解除（瞬断は無視）。

ゴースト色: 🟢緑=追従中 / マゼンタ=停止。

---

## 5. follow モード（IK追従）制御則

`_xrUpdateGoalFromHand`。**手の位置=速度指令のレート制御**（1:1 位置追従ではない）。

### 5.1 速度係数
1. 手の中立点からのズレ → `_xrViewerDeltaToArm` で arm base 系 delta（mm）に変換 → `dispMm`。
2. デッドゾーン+expo カーブ `_rateCurve(disp, deadzone, full)` → 速度係数 `spd`（0..1）。
   - `disp ≤ XR_RATE_DEADZONE_M(0.05m=50mm)` → `spd=0`（手ブレ殺し）。
   - `XR_RATE_FULL_POS_M(0.20m=200mm)` で `spd=1`。
   - `spd = ((disp-dead)/(full-dead))^XR_RATE_EXPO`、`XR_RATE_EXPO=2.5`（中央緩く端速い）。

### 5.2 目標の積分
- `doPos` 時、目標先端を **方向維持で積分**: `target += dir * (XR_RATE_MAX_POS_MMS(140) * gain * spd * dt)` mm/s。
- 向き（rot）は **既定 OFF**（`XR_RATE_ORIENT_ENABLED=false`）。有効時は位置と**排他**（優勢な方のみ動かす）。
- 目標の向き `smoothQuat` は engage 時の実機先端向きで固定。

### 5.3 到達範囲の扱い
- **reach グリッドへのスナップクランプは既定 OFF**（`XR_REACH_CLAMP_ENABLED=false`）。
  - 旧実装は疎な baked 点へスナップしてノコギリ波・無追従の原因だった → 撤去。
- **IK 失敗時は目標を前フレームの到達可能値へロールバック**（`rateTargetPos` を巻き戻す）。
  - これにより目標が可動域外へ暴走せず、腕は境界で滑らかに停止、手を戻せば即再開する。

### 5.4 IK
- クライアント側 `solveIKTeleop(pos, rpy, seed)`（DLS、warm-start=前フレーム解）。
- 段階フォールバック: **full（位置+向き）→ relaxed（向き許容）→ position-only**。
- `xr.ikMode` = `full / relaxed / position / failed`。ゴースト色も連動（緑/黄/橙/赤）。

---

## 6. joint モード（直接関節）制御則

`_xrUpdateJointsFromHand`。手の姿勢を 4 関節に直接マップ（**4-DoF、J3/J4 は engage 時の値で固定**）。

| 関節 | 入力 | ゲイン |
|---|---|---|
| J1 | HMD→手 ベクトルの水平方位角 delta | `gain × XR_SHOULDER_GAIN(2.8)` |
| J2 | 同・垂直角 delta | `gain × XR_SHOULDER_GAIN(2.8)` |
| J5 | 手首の anchor-local ピッチ | `gain` |
| J6 | 手首の anchor-local ヨー | `gain × XR_WRIST_YAW_GAIN(2.2)` |

- 360° アンラップ（±180°境界のジャンプ除去）＋ **EMA スムージング**（`XR_JOINT_SMOOTH_ALPHA=0.35`）。
- 送信 ~30Hz（`XR_JOINT_INTERVAL_MS=33`）、1 送信あたり最大 `XR_JOINT_MAX_STEP_DEG=4.0°`。
- 各関節は `LIMITS` でクランプ。符号は UI チェックボックス `xr.signs`（J1/J2/J5/J6 反転、localStorage 永続）。

---

## 7. 座標マッピング（`_xrViewerDeltaToArm`）

viewer（視点）系の手 delta → arm base 系 delta。
- viewer: +X 右 / +Y 上 / +Z 後（手前）
- arm base: +X 右 / +Y 奥 / +Z 上
- 既定マップ: `arm_x = hand_x`, `arm_y = -hand_z`, `arm_z = hand_y`
- **yaw キャリブ** `xr.calibYaw`（rad）: viewer +Y 軸まわりに手 delta を回してからマップ。「キャリブ」ボタンで HMD 前方を arm +Y に合わせる。既定 0。

---

## 8. 送信経路と安全

- **両モードとも、クライアントで関節角を算出 → `/jog` に `{angles, speed}` を POST**（follow も先端目標を手元 IK で角度化してから送る）。
- 送信は **単一 in-flight + 保留枠 1（latest-wins）**（`_xrSendJog`）。応答待ち中は最新目標のみ保持。
- サーバ `/jog`（[server.py](../scripts/server.py)）:
  - **40Hz 上限**（25ms 最小間隔、超過は `THROTTLED`）。
  - 計画移動 `/move` 実行中は `MOVING` で拒否。
  - **安全は床干渉 + 関節限界のみ**チェック（自己干渉はユーザー目視管理）。NG は `SAFETY` / `IK_FAIL`。
  - 非ブロッキング送信。
- `speed` は UI スライダ `xr.sendSpeed`（5–100, 既定 60）。サーバ `--max-speed` でクランプ。

> ⚠ `_xrSendJogPos`（座標を /jog に送る経路）は現在**デッドコード**。実経路は `_xrSendJog`(角度) のみ。

---

## 9. チューニング可能パラメータ一覧（現行値）

| 定数 / 変数 | 値 | 意味 |
|---|---|---|
| `XR_START_POSE` | `[90,15,-100,-15,-50,0]` | VR 開始姿勢（非特異・中央寄り）|
| `XR_START_SPEED` | 30 | 開始姿勢への移動速度 |
| `PINCH_ENTER_M` / `PINCH_EXIT_M` | 0.025 / 0.035 | ピンチ握り/離し閾値（m）|
| `XR_TRACK_LOST_MS` | 600 | 自動フリーズまでのトラッキング消失時間 |
| **follow** | | |
| `XR_RATE_DEADZONE_M` | 0.05 | 位置デッドゾーン（m）|
| `XR_RATE_FULL_POS_M` | 0.20 | 最大速度に達する手ズレ（m）|
| `XR_RATE_MAX_POS_MMS` | 140 | 最大速度（mm/s）|
| `XR_RATE_EXPO` | 2.5 | 入力カーブ指数 |
| `XR_RATE_ORIENT_ENABLED` | false | 向きレート（既定 OFF）|
| `XR_REACH_CLAMP_ENABLED` | false | reach スナップクランプ（既定 OFF）|
| **joint** | | |
| `XR_SHOULDER_GAIN` | 2.8 | 肩角→J1/J2 倍率 |
| `XR_WRIST_YAW_GAIN` | 2.2 | 手ヨー→J6 倍率 |
| `XR_JOINT_SMOOTH_ALPHA` | 0.35 | EMA 係数 |
| `XR_JOINT_MAX_STEP_DEG` | 4.0 | 1送信あたり最大関節移動（°）|
| **共通** | | |
| `xr.gain` | 1.0 (UI 0.3–2.0) | 速度/回転倍率 |
| `xr.sendSpeed` | 60 (UI 5–100) | firmware 速度 |
| `xr.calibYaw` | 0 | 手→arm yaw 補正（rad）|
| **配置** | | |
| `XR_VIEW_FWD_M` / `XR_VIEW_DROP_M` | 0.6 / 0.45 | 基部を目線の前/下に置く量 |
| `XR_VIEW_YAW_RAD` | π/2 | アーム模型の見え方回転 |

---

## 10. テレメトリ & HUD

- **テレメトリ**: 1 engage = 1 trial。`/clientlog`（[server.py](../scripts/server.py)）へ JSONL で送信、`data/client_logs/vr_session.jsonl` に永続化。
  - `trial_start`（全チューニング値スナップショット）/ `~15Hz サンプル`（手入力 hv・絶対手座標 hw・マップ後 arm・spd・目標・IKモード・実機角・先端FK）/ `trial_end`（要約: dwell% / ikFail% / 手移動範囲 / 先端移動距離）。
  - console.* / 未捕捉エラーもミラー。
- **HUD**（VR 内, 視界下）: trial 番号・engage 状態・ライブの手ズレ/速度係数/IKモード/先端移動量。「手は動いてるのに速度0」等の失敗をその場で可視化。

---

## 11. 既知の課題 / 調整中の項目

- **デッドゾーン 50mm が大きめ**: 自然な手の動きの初動が吸われる。即応性を上げるなら縮める候補（dwell% を見て判断）。
- **follow はレート制御**: 「掴んで動かす」直感とズレる可能性。位置 1:1 マップは別途検討の余地。
- **向きレート OFF**: 先端の向きは engage 時固定。向き操作が要るなら有効化＋排他ロジック調整。
- **joint は 4-DoF**: J3/J4 固定で先端を任意位置へ置けない。
- **ハンドトラッキングのドロップアウト**: 手を FOV 端へ伸ばすと飛ぶ（hw のフレーム間ワープで検出可）。
- **デッドコード** `_xrSendJogPos` の整理。

---

## 12. ✋ ハンド専用ページ（/hand）と同時運用

上記 1〜11 は 🦾アーム（myCobot）の teleop。**ハンド（Hiwonder 5指）は別系統**で、軽量ページ
[scripts/hand.html](../scripts/hand.html)（three.js 無し）が `/hand` で配信される。右手5指の curl を
0..1 に正規化して `/hand/fingers` に POST（サーバ側 40Hz スロットル・latest-wins）。curl 計算・calib は
ui.html と同じ。PWA 対応（`<title>ロボットハンド操作` + manifest + アイコン。App Library に載る）。配備は
[QUEST_DEV.md](QUEST_DEV.md) の「2台運用」を参照。

### SO-101 teleop と同時運用

ハンド teleop と **SO-101 リーダー/フォロワー追従は独立に並行できる**。両者は別プロセス・別ハード：

- SO-101 teleop は cockpit（:8013, `.venv-so101`）内で完結し **Quest 非依存**（PC 内の COM13/COM14 のみ）。
- ハンドは hand server（:8001, `--offline --real-hand`）が COM9 の実5指ハンドを駆動。Quest の WebXR から操作。
- 同時起動はワンコマンド [scripts/teleop_all.ps1](../scripts/teleop_all.ps1)（cockpit + hand + home を上げ、Quest 2台へ `/hand` を配備）。**SO-101 follow は自動 ON にしない** — 安全のためコックピットのトグルで明示的に開始する。

### 2台同時装着（Quest ×2）

2台の Quest が同じ hand server（同じ `adb reverse tcp:8001`）を共有する。したがって
**`/hand/fingers` は latest-wins（排他制御なし）**：両機が同時に指を動かすと、後に届いた POST の指値で
実ハンドが上書きされる（40Hz スロットルは共有）。1人が操作し他方は観戦、あるいは交互操作を想定した運用。
同一 PC・同一 COM9・単一の物理ハンドを2台で奪い合う構図なので、**同時に別々の指形を送っても合成はされない**。
