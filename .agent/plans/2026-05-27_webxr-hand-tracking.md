# WebXR ハンドトラッキングでアームを相対操作

## 方針転換 (2026-05-29): 直接関節マッピングを追加

当初は「手 → goalSphere → IK（6DOF follow）」一本だったが、より直感的な
**直接関節クラッチ**モードを追加（UI トグルで切替、デフォルト=直接関節）。
IK を介さず手首 pose を関節へ相対マップする：

| 入力（右手 pinch クラッチ中の相対量） | 関節 |
|---|---|
| 手ロール（手のひらを縦軸でひねる, anchor-local Euler Z）| J1 |
| 手の上下移動（手首 Y × heightScale °/m）| J2 |
| 手ピッチ（手の向き, Euler X）| J5 |
| 手ヨー（手首回転, Euler Y）| J6 |

- J3/J4 は anchor 固定。全て pinch 開始時の `state.current` からの相対。
- 送信は既存 `/jog {angles, speed}` に速度制限ストリーム（max 2.5°/送信, ~25Hz）。
- **符号・軸はヘッドセット実機でしか確定できない** → UI に J1/J2/J5/J6 符号反転
  チェックボックス＋回転GAIN＋J2高さ°/m スライダを出し、被って詰める前提。
- 実装は `scripts/ui.html` の WebXR セクション（`xr.ctrlMode==='joint'` 分岐、
  `_xrUpdateJointsFromHand`）。IK追従コードは温存。
- prefs: `vrCtrlMode` / `vrHeightScale` / `vrSigns` を localStorage に追加。

オフライン検証で確認済み: ブート・コントロール描画・モードトグル・disabled 連動・
prefs 保存。マッピングの正しさ（符号/軸/感度）は次回 Quest 実機で要調整。

## ゴール

VR HMD（Quest 系想定）の WebXR hand tracking で取った「手の動き」を
`goalSphere`（既存の 6DOF target）にマップし、既存の **手首追従モード** 経由で
アームを動かす。実機を動かすのは最終段階。それまでは 3D viewer 内で完結。

## 既存資産（流用前提）

- `scripts/ui.html` の **手首追従モード**（`posingMode === 'follow'`）
  - `goalSphere` の position / quaternion を直接書き換えれば `buildPoseSpec` が
    explicit pose を組んで `/solve_ik` に流す既存パイプが動く
  - つまり「`goalSphere` を WebXR の手で動かす」だけ作ればアーム側は無改造
- `data/reachable_grid.json` — reachable space clamp に使える

## 設計の核（3 つだけ覚えれば良い）

### 1. クラッチ式の相対追従（必須）

手の絶対座標で goalSphere を駆動しない。**ピンチ中だけ delta を積分する**。

```
pinch_start:
  anchor_hand   = currentHandPose      # 手側の原点
  anchor_target = goalSphere.pose      # アーム側の原点
pinch_hold (per frame):
  delta_hand    = currentHandPose - anchor_hand   # camera 空間
  delta_world   = R_calib * delta_hand            # アーム base 空間へ
  goalSphere.pos = anchor_target.pos + delta_world.pos * GAIN
  goalSphere.rot = delta_world.rot * anchor_target.rot
pinch_release:
  freeze (goalSphere そのまま、次の pinch まで動かさない)
```

- `GAIN` は position 1.0 / rotation 1.0 開始、UI スライダで 0.3〜2.0 可変に
- pinch は `XRHand` の thumb-tip と index-tip の距離 < 25mm 判定で十分

### 2. キャリブレーション（座標系合わせ）

WebXR の手は「ビューア座標」、アームは「base 座標」。**1 回のキャリブ姿勢**で
回転行列 `R_calib` を取る。

- UI に「キャリブ」ボタン
- 押下時の HMD 前方ベクトルを「アームの +X（前方）」に揃える yaw 補正のみで実用十分
- pitch/roll は HMD 装着姿勢に依存するので無視（ユーザーが正面を向いている前提）
- localStorage に保存（versioned key、既存規約に従う）

### 3. レート整合と reachable clamp

- WebXR は 72–90Hz。**サーバ送信は 20Hz にデシメート**
- goalSphere 位置に **EMA smoothing**（α=0.3 程度）
- 送信前に `reachable_grid` で clamp（範囲外なら最近傍点に丸めて、UI で赤く表示）
- サーバ側 `/solve_ik` は既存のまま。**新しい endpoint は作らない**

## 実装フェーズ

### Phase 1: WebXR session + 手の可視化のみ（実機なし）

- `scripts/ui.html` に「VR」ボタン（`navigator.xr.requestSession('immersive-vr', {requiredFeatures:['hand-tracking']})`）
- XR frame loop で両手 25 joints を取得し、three.js で小球として描画
- pinch 判定の動作確認だけ（goalSphere はまだ動かさない）
- **完了条件**: HMD 内で自分の手が見える + pinch でログが出る

### Phase 2: クラッチ式 goalSphere 駆動（オフライン、実機なし）

- `python scripts/server.py --offline` 前提
- 右手 pinch で goalSphere を相対操作 → 既存 follow モードで IK が解けて仮想アームが追従
- キャリブボタン実装
- GAIN スライダ、smoothing、reachable clamp
- **完了条件**: HMD 内で「手を動かす → 仮想アームの tip が同じ向きに動く」が直感的に成立

### Phase 3: 実機接続

- 速度上限を `MAX_SPEED=40` の半分（20）に一時的に絞って試運転
- 緊急停止ボタンに人が必ず手を添える（[.agent/rules/safety.md](.agent/rules/safety.md)）
- 問題なければ 40 まで戻す

## やらないこと（明示）

- 両手協調（左手で何か別操作、等）→ 将来。今は右手の pinch + delta のみ
- 手の指 pose をアーム手首の roll に rich にマップ → 過剰。pinch クラッチ + delta だけ
- 新しいサーバ endpoint → 不要。`/solve_ik` で完結
- HMD なし（ブラウザだけ）での WebXR emulator 対応 → 後回し、Quest 実機で開発

## 想定ハマりどころ

- **Quest Browser でしか動かない可能性**: PC + Link 経由の Chrome は hand-tracking 不安定。
  最初に Quest Browser から `http://<PC IP>:8000` を開けるか確認（`--bind 0.0.0.0` 必須、
  LAN 公開なので `--token` も付ける — CLAUDE.md の規約通り）
- **HTTPS 必須**: WebXR は localhost 以外は HTTPS。LAN で Quest から繋ぐなら mkcert 等で
  自己署名 → Quest 側で証明書受け入れ。あるいは `adb reverse` で localhost にする
- **左右の手の id が frame 毎に入れ替わる**: `XRInputSource.handedness` で毎フレーム判定
- **pinch chattering**: 距離閾値にヒステリシス（enter 25mm / exit 35mm）

## 参考: 既存 UI のフック点

- [scripts/ui.html:1081](scripts/ui.html:1081) — `goalSphere` 定義
- [scripts/ui.html:1572](scripts/ui.html:1572) — `posingMode` state
- [scripts/ui.html:1589](scripts/ui.html:1589) — `buildPoseSpec`（follow 時の explicit pose 組立）
- [scripts/ui.html:1646](scripts/ui.html:1646) — `/solve_ik` 呼び出し

新規コードは ui.html の末尾に WebXR セクションを追加する形で良い。
規模が大きくなりそうなら `scripts/webxr.js` に分離。
