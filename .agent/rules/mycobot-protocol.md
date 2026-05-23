# myCobot シリアルプロトコル メモ

実装で詰まったときの参照用。pymycobot がフレームを抽象化してくれるので普段は意識しない。

## 基本

- ボーレート: **115200**（USB UART Transponder モード）
- フレーム: `FE FE <len> <cmd> [<data>...] <checksum>`
  - `<len>` = cmd + data + checksum のバイト数（多くは 2）
  - `<checksum>` = フレーム末尾 1 バイト（実装により単純加算反転 or 固定）
- 受信は同じ形式で `<cmd>` のエコーが返る

## 確認済みのフレーム例

| 用途 | フレーム |
|---|---|
| get_system_version 要求 | `FE FE 02 02 FA` |
| 応答（version=4.2）| `FE FE 03 02 2A FA` |
| power_on | `FE FE 02 10 EE` |
| is_power_on 要求 | `FE FE 02 12 EC` |

## 既知の事象

### 1. `power_on()` が -1 を返すが実際は ON

ACK を取りこぼすことがある。`is_power_on()` が `1` を返せば真。

### 2. COM ポート番号が変わる

USB 抜き差し / ハブのポート変更で Windows が別 COM 番号を割当てる。ポート固定書きはしない。`serial.tools.list_ports` で CH9102 を探す。

### 3. `Atom: no` のまま

- 緊急停止ボタンが押し込まれている → 解除（時計回り）
- Basic ↔ Atom 内部 Grove ケーブル緩み → 一度電源 OFF→ 接続確認 → ON
- ファーム不整合 → myStudio で `atomMain`（Atom 側）と `minirobot`（Basic 側）を最新へ

### 4. Transponder に入ってるのに無応答

- ケーブルが USB 給電のみのもの → データ通信できる USB-C ケーブルに替える
- 内部 UART 配線/CH9102 不良 → 別の USB ポートで COM 番号が変わって復活したケースあり（cogni-storage 側で実際に発生）

### 5. ボーレートが違うかも

`scripts/sweep.py` で 9600/57600/115200/230400/460800/921600/1M を順に試して応答フレームが返るボーレートを確認。

## 参考

- 公式ドキュメント: https://docs.elephantrobotics.com/docs/mycobot_320_m5_en/
- pymycobot: https://github.com/elephantrobotics/pymycobot
