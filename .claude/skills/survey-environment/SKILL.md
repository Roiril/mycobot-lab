---
name: survey-environment
description: アームの wrist カメラで四方（back/right/front/left）を見回し、各方向の観察内容を spatial_memory（UI）に記録する。使うタイミング：「四方見て」「環境確認して」「周りを見回して」「spatial memory を更新」「部屋の様子を記録」等、ユーザーが空間認識を更新したい時。HOME 戻しまで自動で行う。
---

# 環境を見回して spatial_memory に書き込むワークフロー

「四方を見回して空間記憶を更新する」一連の流れ。実機で確立済み（2026-05-25）。

## 全体の流れ

1. **接続確認**（必要なら）
2. **サーバ起動**（既に起動してるか確認、無ければ background で起動）
3. **4方向ループ**（back → right → front → left の順）
   - `POST /observe {direction}` でアームを動かしフレーム保存 + sector entry 作成
   - `Read` で保存された JPEG を視覚確認
   - `POST /memory/annotate` で description + objects を書き込み
4. **HOME 戻し** (`POST /home`)
5. **検証**: `GET /memory` で 4 sector とも observer=シュビー、stale=False になっていることを確認

## 詳細

### 1. 接続確認

`scripts/check.py` で version/angles が取れることを確認。`PermissionError(13)` で COM が掴まれていたら、古い python プロセスを kill する：

```powershell
Get-Process python* -ErrorAction SilentlyContinue | Where-Object { $_.StartTime -lt (Get-Date).Date } | Stop-Process -Force
```

それでも掴まれてたら**今日起動分の python も**全て落とす（自分が起動した server.py プロセスは後で再起動）。

### 2. サーバ起動

```bash
python scripts/server.py > /tmp/server.log 2>&1 &
sleep 3
curl -s http://localhost:8000/power  # → {"ok": true}
```

### 3. 4方向ループ

各方向で順に：

```bash
# (a) 観察姿勢に動かしてフレーム保存
curl -s -X POST http://localhost:8000/observe \
  -H "Content-Type: application/json" \
  -d '{"direction":"back"}'
# → {"ok":true, "frame_full_path":"...\\observe_YYYYMMDD_HHMMSS_back.jpg", "memory_sector":"back", ...}
```

**direction → J1 対応**:
| direction | J1 (deg) | カメラ向き |
|---|---|---|
| back  |   0 | -Y（アーム背面） |
| right |  90 | +X（アーム右） |
| front | 165 | +Y 寄り（180 は J1 limit のため 165） |
| left  | -90 | -X（アーム左） |

`/observe` は HOME-like 姿勢（J2=0, J3=-90, J4=0, J5=0, J6=CAMERA_UPRIGHT_J6_DEG）で J1 だけ振る。カメラは床と水平。

```bash
# (b) 保存された JPEG を Read で視覚確認
# frame_full_path をそのまま Read tool に渡す
```

frame_full_path（絶対パス）を `Read` ツールに渡して画像を見る。

```bash
# (c) 観察内容を spatial_memory に書き込む
curl -s -X POST http://localhost:8000/memory/annotate \
  -H "Content-Type: application/json" \
  -d @- <<'EOF'
{
  "sector": "back",
  "description": "<シュビーが画像から書き起こした2-4行の自然文>",
  "observer": "シュビー",
  "objects": [
    {"label": "<物体名>", "position_mm": [x, y, z], "note": "<任意>"},
    ...
  ]
}
EOF
```

**objects の position_mm の決め方**: base 座標系での **大まかな推定**でよい。距離感は画像から目測 + 「アームから見て前方 700mm」「右奥 300mm」のように相対値で。正確な 3D 復元は phase 2 で perceive() がやる。

**base frame の XY 軸定義（絶対に取り違えるな）**:
- **+X = right** (J1=+90° 方向)
- **-X = left**  (J1=-90° 方向)
- **+Y = front** (J1=±180° 方向、実装は 165°)
- **-Y = back**  (J1=0° 方向)
- **+Z = up**

sector 名（front/back/right/left）と座標軸 (X/Y) を **混同しない**こと。例:
- front sector の「奥のディスプレイ 2000mm」は `[0, 2000, 300]`（**+Y方向**）が正解。`[2000, 0, 300]` だと right 方向を指す座標になり、point_at が別の方を向く（実機で踏んだ）
- right sector の「奥のモニター 700mm」は `[700, 0, 200]`（**+X方向**）

annotate 直後に `point_at target_xyz=<その物体の座標>` を投げて、想定通りの方向に腕が向くかをサニティチェックすると安全。

**description の書き方**:
- 視界の構図（手前/中央/奥/天井/床）を 1 文目
- 主要オブジェクト 3-5 個を読みやすく列挙
- 文字情報（モニター画面・ラベル・ポスター）があれば引用すると後で役立つ
- 「ユーザー本人」が映った場合は「シュビーから見て○○方向、近距離」のように相対位置で書く

### 4. HOME 戻し

```bash
curl -s -X POST http://localhost:8000/home
```

### 5. 検証

```bash
curl -s http://localhost:8000/memory | python -c "
import sys, json
d = json.load(sys.stdin)
for sec, e in d['sectors'].items():
    print(f\"{sec:6s} | obs={e['observer']:8s} | n_obj={len(e.get('objects',[]))} | stale={e['stale']} | desc={e['description'][:40]}\")"
```

4 sector が `observer=シュビー / stale=False` ならOK。UI（http://localhost:8000/）でも確認可能。

## ハマりどころ（実機で踏んだ）

### `/observe` の "single-shot 到達タイムアウト" は false positive のことがある

- 大きな J1 回転（例: front J1=165° → left J1=-90°）の後、tolerance 判定が誤発火
- 直後に `/angles` を取って目標近傍に居れば、もう一度 `/observe` を呼ぶだけで成功
- 失敗時の戻り値は `memory_sector: None` + `error: "single-shot 到達タイムアウト"`

### CAMERA_UPRIGHT_J6_DEG=90 は J1=90° でしか画像が正立しない

- back/front/left で撮ると画像は 90° 横倒しになる（カメラのキャリブレーションズレ）
- **annotate するときは画像の回転を考慮して『カメラから見て何が映っているか』を書き起こす**
- 詳細: [[camera-upright-calibration]]

### `/observe` は frame_full_path を返すので Read に渡す

- `frame_path` は相対、`frame_full_path` は絶対。Read tool には絶対パスが必要

### サーバ停止忘れに注意

- 作業完了後、自分が起動した server.py を kill しないと、次回 COM12 を掴んだままになる
- ただし**ユーザーが別途使う可能性もある**ので、勝手に kill せず最後にユーザーに「サーバ落とす？」と聞くのが安全

## 関連メモリ

- [[hardware]] — 接続トラブル時の正攻法
- [[mycobot-firmware-quirks]] — 押下後の servo latch（M5 再起動以外復旧不可）
- [[camera-upright-calibration]] — 画像 roll が J1 に依存する件
