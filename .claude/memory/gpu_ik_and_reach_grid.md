---
name: gpu-ik-and-reach-grid
description: reach 点群の IK ベイク設計。GPU バッチ IK は FP64 必須、J1 回転対称で計算量激減、reach 点に角度ベイクで実行時 IK ゼロ。生成器の使い分けと再生成手順
metadata:
  type: technique
---

reach 点群（クリック/jog の到達点）への joint 角度ベイク + GPU 生成のノウハウ。

## 設計の核

- **角度ベイク**：reach 点に IK 結果（6 角度）を保存しておき、実行時は IK を回さず baked を直送り。
  クリック/jog/ドラッグの体感が劇的に軽くなる（実行時 IK の SOLVER_NONCONVERGENT も消える）。
- **J1 回転対称**：J1 は base Z 軸の純回転で、下流は J1 値に非依存。
  → (r, 0, z) で 1 回解けば、任意方位 θ は `[J1_solved + θ, J2..J6]` で得られる。
  IK 問題数が N_r·N_z·N_θ → N_r·N_z（数十倍減）、かつ全 θ で J2-J6 が**完全同一**＝真の花弁。

## GPU IK のハマりどころ

- **FP64 必須**。FP32 だと DLS が 2mm tol に収束せず成功率が激落ち（25% 程度）。
  `ik_gpu.DTYPE = torch.float64`。4090 でも FP64 で十分速い（数千点/秒）。
- **複数 seed → score 最良を採用するが、safety NG の解は飛ばす**。
  best 1 個だけ見て safety NG だと落ちるので、score 昇順に全 seed を見て最初の safety OK を選ぶ。
- PyTorch は CUDA 版を別 index から入れる：
  `pip install --index-url https://download.pytorch.org/whl/cu128 torch`
  （sandbox が外部 index を弾くので `dangerouslyDisableSandbox` か手動実行が要る）

## 生成器の使い分け（重複整理済み）

| スクリプト | エンジン | 用途 |
|---|---|---|
| `reachable_grid_petal.py` | GPU・J1 対称 | **現行・第一選択**（~1.5 秒 / 約 2 万点）|
| `reachable_grid.py` | CPU(numpy) | torch 無し環境の fallback（遅い、~18 分）|

※ cylindrical GPU 版（reachable_grid_gpu.py）は petal に置換して削除済み。

前提データ：`data/reachable_rz.json`（FK 包絡。`scripts/reachable_rz.py` で生成）が必要。

## 再生成手順

```bash
python scripts/reachable_grid_petal.py --angles 36 --z-step 15 --r-step 15
```
**TOOL_LENGTH や URDF を変えたら必ず再生成**（baked 角度が古い幾何前提のままになる）。

## バックグラウンド実行の罠

長時間スクリプトを background task で回すと **stdout がブロックバッファされて出力ファイルが空のまま**進む。
進捗を見たいなら `python -u`（unbuffered）で起動する。
