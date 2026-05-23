# 接続トラブルシュート（初回セットアップで得た知見）

myCobot 320-M5 を PC から動かそうとして無応答になった時の切り分け順序。**2026-05-23 の初回接続で実際に詰まったポイント**を体系化したもの。何より「ハード側の物理確認」が最優先。

## 切り分けフロー

### 0. 症状の特定

| 観測 | 意味 |
|---|---|
| `Get-PnpDevice` で CH9102 が見える | PC ↔ CH9102 チップは生きている |
| `version: -1` / `angles: -1` | pymycobot からは応答ゼロ。CH9102 ↔ M5 MCU の経路 or ファーム/モードを疑う |
| 一度開いた COM が次回 `PermissionError(13)` | 直前プロセスが COM を握りっぱなし → 数秒待つ or プロセス Kill |
| `FileNotFoundError(2)` | COM ポート自体が消えた = USB が抜けた / 再列挙された |
| 別 COM 番号が現れる（シリアル番号も別物）| 物理的に別の USB-Serial チップに繋がった証拠 = ケーブル挿し場所が変わった |

### 1. 物理：緊急停止と電源

- **STOP ボタンを時計回り**で確実に解除（押し込まれていると Atom 通信から落ちる）
- 電源 LED（緑）が点灯していること
- 24V DC アダプタが奥まで差さっていること

### 2. M5 画面：Atom 通信が成立しているか

- M5 画面で `Connect test` → **`Atom: ok`** が出るか
- `Atom: no` の場合の対処順位（公式 FAQ）：
  1. STOP ボタン再確認
  2. **Atom（先端の小さい LED マトリクス画面）を軽く押し込んで接点を整える**
  3. 電源 OFF → 内部 Grove ケーブル抜き差し → ON
  4. myStudio で Atom 側 `atomMain` / Basic 側 `minirobot` を最新化

### 3. M5 画面：Transponder モードに入る

- メニュー → `Transponder` → `USB UART` → OK
- **入った後の画面は `Connect test / Atom: ok` 表示**（公式ドキュメント確認済）。一見ただの診断画面に見えるが、これが「USB UART Transponder 動作中の画面」そのもの。Exit を押すまで PC からの待受状態
- WLAN Server / Bluetooth に間違えて入っていないか確認

### 4. PC 側：別アプリが COM を掴んでいないか

- myStudio、TeraTerm、Arduino IDE のシリアルモニタ等を全て閉じる
- `PermissionError(13)` が出る場合の典型原因

### 5. ボーレート確認（最強の疎通テスト）

pymycobot を使わず生 serial で確認：

```bash
python scripts/sweep.py
```

- 期待する応答（115200 で送信した場合）: `fe fe 03 02 2a fa`
  - `fe fe` ヘッダ / `03` 長さ / `02` cmd エコー / `2a` data(=4.2=ver) / `fa` チェックサム
- 別ボーレートでガベージ（`78 80 78 ...` のような繰り返しバイト）が返るのはノイズ。応答とみなさない

### 6. それでもダメ：物理ケーブル経路を疑う

**今回の決定打**: 同じ USB ケーブルでも、別の物理 USB-C ポートに挿し直したら復活した。

- 観測: 旧 COM11（CH9102 SN: `5626035067`）= 完全無応答 → 別ポートに挿し直し → COM12（SN: `56E3004757`）= 正常応答
- 原因として可能性が高い順：
  1. **USB ケーブルのデータ線が一部断**（給電は通るので Windows は CH9102 を列挙する）
  2. **M5Stack Basic 内部の USB ↔ MCU UART 配線不良**（CH9102 までは生きるが先が死ぬ）
  3. USB ハブの該当ポートのデータ品質不良
- 試す順: ケーブル交換 → ハブ違うポート → PC 本体直挿し → 別 PC

### 7. pymycobot で詳細ログを取る

```python
mc = MyCobot320(port, 115200, debug=True)
```

送信フレーム（`_write: fe fe ...`）と受信フレーム（`_read: ...`）が逐次表示される。`_read:` が空ばかりなら **CH9102 ↔ M5 の片方向通信失敗**。

## 失敗パターンと正解の対応表

| やった事 | 結果 | 学び |
|---|---|---|
| CH9102 が見えてるから通信できるはず | ❌ 完全無応答 | デバイス列挙 ≠ データ通信成立 |
| `Atom: no` のまま Transponder に入った | ❌ 当然無応答 | Atom: ok が必須前提 |
| 同じ USB-C ポートで何度もリトライ | ❌ 変化なし | ハード故障/接点不良を疑え |
| 別の USB-C ポートに挿し直し | ✅ 即応答 | **物理経路を変えるのが最短** |
| 生 serial で `fe fe 02 02 fa` 送って応答確認 | ✅ 切り分け成功 | pymycobot 抜きで疎通テストが効く |

## やってはいけない切り分け

- pymycobot のソースを変更してデバッグログを増やす → 戻すのを忘れる。`debug=True` 引数で十分
- ファームを書き換える → ハード故障でない場合は元に戻せず詰む。物理確認が全部済んでから最後の手段
- スクリプトを連打する → COM が掴まれて `PermissionError` 連鎖。一回ごとに 1-2 秒空ける

## 関連

- 機種固有の事実: [../../.claude/memory/hardware.md](../../.claude/memory/hardware.md)
- プロトコルメモ: [mycobot-protocol.md](mycobot-protocol.md)
- 安全規約: [safety.md](safety.md)
