---
name: robot-action
description: ユーザーが myCobot 320 に物理動作（指さす・お辞儀・振る・うなずく・特定の物に face する・カメラで見る・物を取る等）をさせたい時の知識ハブ。「○○して」「あれを指さして」「ユーザーに向かって挨拶」「あの本見せて」「右側何ある？」等のリクエストを HTTP API + spatial_memory + gestures に落とすための判断フロー、典型パターン、ハマりどころ集。survey-environment（四方見回し）など下位スキルはここから派生。
---

# Robot Action — ロボットアーム + カメラ + UI 動作の判断ハブ

ユーザーが「○○して」とアームに何かさせる時、このスキルから入る。

詳細な API リファレンスは [docs/AGENT_API.md](../../../docs/AGENT_API.md)、設計は [docs/ARCHITECTURE.md](../../../docs/ARCHITECTURE.md)。ここでは **経験で踏んだ落とし穴** と **どの API を選ぶかの判断軸** をまとめる。

## 0. 動作前の必須チェック（毎回 / 1回で OK）

1. **アームが緊急停止中ではないか確認**（停止中は接続試行禁止 → memory: feedback_estop_no_connect）
2. **サーバ起動** — `http://localhost:8000/power` が `{"ok": true}` を返すか
3. **PermissionError(13) on COM12** → 古い python プロセス kill：
   ```powershell
   Get-Process python* | Where-Object { $_.StartTime -lt (Get-Date).Date } | Stop-Process -Force
   ```
4. **動作開始前に予告**: 「想定の動き（移動先・速度・所要時間）を1行で宣言してから実行」（CLAUDE.md 規約）

## 1. リクエストを 4 カテゴリに分類

ユーザーの依頼を以下に分類して API を選ぶ：

| カテゴリ | 例 | 主 API |
|---|---|---|
| **A. 表現動作（gesture）** | お辞儀して / 振って / うなずいて / こっち向いて | `/gesture` |
| **B. 指す・向ける** | あれ指して / モニター指して / ユーザーの方向いて | `/gesture` (`point_at` / `face`) |
| **C. 観察・記憶** | 周りを見て / 何がある？ / 写真撮って / メモして | `/observe` + `/memory/annotate` + `/frame.jpg` |
| **D. 物理操作（把持）** | あれ取って / ここに置いて | `/grasp_sequence` ([AGENT_API.md](../../../docs/AGENT_API.md) §3 参照) |

複合（「あれを見て指してお辞儀して」等）は `/gesture` の **list 形式で連鎖**できる。

## 2. /gesture チートシート

```bash
# 単発
curl -X POST localhost:8000/gesture -H "Content-Type: application/json" \
  -d '{"kind":"bow","direction":"left","return_home":true}'

# 連鎖（1回の motion_lock で実行、間に他の動作が割り込まない）
curl -X POST localhost:8000/gesture -H "Content-Type: application/json" \
  -d '[
    {"kind":"point_at","target_xyz":[0,2000,300]},
    {"kind":"bow","direction":"front"},
    {"kind":"home"}
  ]'
```

| kind | params | 意味 |
|---|---|---|
| `face` | `direction`, `upright?` | 方向を向く（カメラ水平） |
| `bow`  | `direction`, `depth_deg=25`, `hold_s=0.5` | お辞儀（face→lean→return） |
| `nod`  | `direction`, `times=2` | うなずき（小さく反復） |
| `wave` | `direction`, `times=3` | 手を振る挨拶 |
| `point_at` | `target_xyz=[x,y,z]` or `label` | 指差し（腕全体を伸ばす） |
| `home` | — | HOME 復帰 |

**`direction` の値**: `"back"`(J1=0) / `"right"`(J1=90) / `"front"`(J1=165) / `"left"`(J1=-90) または数値 J1°。

## 3. base frame 座標系（取り違え注意）

ユーザーから見た方向と base 座標軸を混同するな：

- **+X = right** (J1=+90°)
- **-X = left**  (J1=-90°)
- **+Y = front** (J1=±180°、実装は 165°)
- **-Y = back**  (J1=0°)
- **+Z = up**（床は z=0）

「奥のディスプレイを 2m 先で指して」と言われたら、front 方向なら `[0, 2000, 300]`。`[2000, 0, 300]` だと right 方向を指す（実機で踏んだ）。

## 4. spatial_memory 経由で「あれ」を指す

ユーザーが言う「あの本棚」「奥のディスプレイ」を解決するには：

1. `GET /memory` で全 sector 一覧、各 sector の `objects[]` を見る
2. label がユーザーの言葉に一致／類似する物体を見つけて `position_mm` を取得
3. `/gesture` に `{"kind":"point_at","target_xyz":[...]}` で渡す

label による自動解決も `/gesture` 側にある (`point_at` で `target_xyz` 無し `label` ありの spec はサーバが memory lookup する)。ただし日本語 label を curl heredoc で送ると **UTF-8 が壊れる**ので：
- **Bash の heredoc で日本語ラベル**を送る場合は `target_xyz` を直接指定するか、Python スクリプト経由で投げる
- ラテン文字だけなら問題なし

該当する観察データが無ければ、まず `survey-environment` skill で見回す。

## 5. 観察したい / 記録したい

**「四方見回し + 記録」** → `survey-environment` skill を呼ぶ（このスキルから委譲）。

**「特定方向だけ見たい」** → `/observe {direction}` 単発。返ってきた `frame_full_path` を `Read` で開けばシュビーが見て応答できる。

**「今のフレーム見せて」** → `/frame.jpg` を curl で保存して `Read`。

**「忘れて」** → `/memory/clear`。

## 6. ハマりどころ

### 日本語 label の UTF-8 壊れ
- Bash heredoc に日本語入れると壊れる場合がある
- 対処: 日本語ラベルは annotate API 経由（JSON body 全体を heredoc）なら通る。問題は `/gesture` body に日本語混ぜた時。`label` フィールドを抜くか英数字に

### `/observe` の "single-shot 到達タイムアウト" は false positive のことがある
- 大きな J1 回転（例: 165° → -90°）で誤発火
- `/angles` で目標近傍に居れば再 `/observe` で成功
- 詳細: [[survey-environment]]

### カメラ正立は J1=90°（right）でしか成立しない
- 他方向は画像 90° 横倒し → annotate / Read 時はカメラから見て何が映ってるかを書く
- 詳細: [[camera-upright-calibration]]

### サーボ latch（押下後）
- 「電源 ON のはずなのに動かない」→ M5 本体再起動以外で復旧不能
- 詳細: [[mycobot-firmware-quirks]]

### 動作完了後は HOME 戻し（または `return_home: true`）
- 不自然な姿勢で放置するとユーザーが手で動かして latch を踏みがち

## 7. 関連

- API 詳細: [docs/AGENT_API.md](../../../docs/AGENT_API.md)
- 設計思想: [docs/ARCHITECTURE.md](../../../docs/ARCHITECTURE.md)
- 安全規約: [.agent/rules/safety.md](../../../.agent/rules/safety.md)
- 下位スキル: `survey-environment`（四方見回し）
- 関連メモリ: [[hardware]], [[mycobot-firmware-quirks]], [[camera-upright-calibration]]
