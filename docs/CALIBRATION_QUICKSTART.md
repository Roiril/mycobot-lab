# CALIBRATION QUICKSTART

今夜の Vision Phase 1 実機初使用手順。所要 30-60 分（カメラの動きが慣れれば 20 分）。

すべて `mycobot-lab/` 直下から実行する想定。

---

## 0. 前提

- myCobot 320-M5 + wrist USB カメラ装着済み
- チェッカーボード（推奨: 9×6 内角、25mm 角、A4 印刷で実測スケール確認、または市販 OpenCV calib board）
- 机面が水平、base flange 上面の Z=0 が決まっている

---

## 1. ハード起動
- アーム土台の **STOP（緊急停止）ボタン** を時計回りで解除。
- 電源 ON → M5 画面 → **Transponder → USB UART** → `Connect test / Atom: ok` を待つ。
- 成功判定: 画面が `Atom: ok` 表示で停止。

## 2. サーバ起動と preflight 確認
```powershell
python scripts/server.py --cam 3
```
- stderr に preflight サマリが出る。`[!]` / `[X]` の行をメモ。
- 成功判定: 少なくとも `[OK] arm hub: real (port=COM…)` が出る。vision の警告は今から潰す。

## 3. フレーム確認
- ブラウザ http://localhost:8000/ を開く。
- 右下 PIP カメラに **机が映っている** こと。
- 成功判定: 机面とチェッカーボードを置いたとき盤面が見える。

## 4. チェッカーボード撮影 (15-20 枚)
- 盤を持って机上に置き、毎回 **角度・位置・距離を変えて** PIP 右上の **「Capture Frame」** をクリック。
- 視野の四隅に盤の角が来るパターンも 2-3 枚混ぜる。
- 成功判定: ボタン横の表示が `20 枚 (data/calib_images/wrist/…)` まで進む。

代替（CLI 撮影）:
```powershell
# サーバを止めてから:
python scripts/capture_calib_frame.py --cam wrist --count 20 --auto --interval 1.5
```

## 5. intrinsics 校正
```powershell
python scripts/calibrate_intrinsics.py --cam wrist --rows 6 --cols 9 --square-mm 25
```
- 成功判定: 最終行 `SUMMARY: cam=wrist | 12/20 images | RMS=0.43px | placeholder=true | …`
  - **RMS < 1.0 px** が必須。1.0 超なら撮り直し（ブレ・ピンボケが多い）。
  - この時点ではまだ `placeholder=true` (hand-eye 未指定)。

## 6. 机面 Z を実測
- base flange 上面から机面までの距離をメジャーで測る (mm)。机が下にあれば負の値。
```powershell
python scripts/calibrate_intrinsics.py --cam wrist --table-z-mm -15.0
```
- (intrinsics は前回値が保たれる)
- 成功判定: `SUMMARY: … table_z_mm=-15.0`

## 7. hand-eye 実測 (T_ee_cam)
- J6 中心からカメラレンズ中心までの (x, y, z) を実測、向きを決める (docs/AGENT_API.md §8.10 参照)。
- 例: ツール先端から +x 30mm、光軸は EE +z 方向と同じ:
```powershell
python scripts/calibrate_intrinsics.py --cam wrist --hand-eye 30,0,0,0,0,0
```
- 成功判定: `SUMMARY: … placeholder=false`

## 8. calibration.json 最終確認
- `data/calibration.json` をエディタで開く。
  - `cameras.wrist.placeholder` が `false`
  - `workspace.table_z_mm` が実測値
  - `cameras.wrist.intrinsics.K` が更新されている

## 9. API キー設定 + サーバ再起動
```powershell
$env:ANTHROPIC_API_KEY = "sk-ant-..."
python scripts/server.py --cam 3
```
- 成功判定: preflight サマリが **全 `[OK]`**。`vision: ANTHROPIC_API_KEY=set | wrist calibrated | table_z=-15.0` が出る。

## 10. OBSERVE 進入経路の素振り
- 机から **被写体を全部退ける**。
- UI で `HOME` → 関節を OBSERVE pose (例: `[0, -30, -60, -30, 0, 0]`) に手動セットし `APPLY`。
- 成功判定: 何にもぶつからずに到達。

## 11. 被写体を配置
- base から半径 200-300mm、ツール直下より **150mm 以上クリアランス** を確保した位置に被写体を置く（例: 赤いコップ）。

## 12. /perceive 試打
```powershell
Invoke-RestMethod -Method Post -Uri http://localhost:8000/perceive `
  -ContentType 'application/json' `
  -Body '{"query":"赤いコップ"}'
```
- 成功判定: `objects[0].world_xyz_mm` が机上の被写体位置とほぼ一致（±20-30mm まで許容、これより悪ければ hand-eye/table_z 再校正）。

## 13. 目視照合
- UI で `world_xyz_mm` の座標を IK 入力 → 「IK 解いてプレビュー」。
- 3D ビュー上で橙ゴースト先端が被写体の上に来るか確認。
- 成功判定: 目視で「概ね合ってる」と判断できる。

## 14. /grasp_sequence 実行 (Phase 1 = 接触はしない、上で止まる)
```powershell
Invoke-RestMethod -Method Post -Uri http://localhost:8000/grasp_sequence `
  -ContentType 'application/json' `
  -Body '{"x":200,"y":50,"z":-10,"radius":30,"speed":20}'
```
- 成功判定: アームが pre-grasp → approach (物体上 ~10mm) → lift で停止、衝突なし。

## 15. 失敗時トラブルシュート
- preflight サマリを再確認 — `[!]` がまだ残っていないか。
- サーバログ (stderr) のエラーコード（`CALIBRATION_PLACEHOLDER_ONLY` / `VLM_API_ERROR` / `OUT_OF_WORKSPACE`）。
- `/perceive` 応答 `diagnostics` の `T_base_cam`, `ray_dir_base` を見て、カメラ位置の見立てと合っているか確認。
- カメラがブラウザに映らない → `--cam` 番号違い。`python -c "import cv2; [print(i) for i in range(5) if cv2.VideoCapture(i).isOpened()]"` で探索。

---

## チェックリスト最終形 (実機検証直前 5 行)

- [ ] preflight 全 `[OK]`（API key set / wrist calibrated / table_z >= FLOOR_Z）
- [ ] 被写体を退かして HOME→OBSERVE が衝突しないことを目視確認
- [ ] 被写体配置: base 半径 200-300mm、ツール直下 >=150mm クリアランス
- [ ] `/perceive` の world_xyz_mm が目視位置と ±30mm 以内で一致
- [ ] 緊急停止ボタンに手が届く位置で `/grasp_sequence` 実行
