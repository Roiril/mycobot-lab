# SO-101「Input voltage error」= 電源ではなくサーボ設定を疑う

2026-07-17 に実際に踏んだ事象と、その切り分けの正解。

## 症状

コックピット(:8013)・CLI teleop が接続時に落ちる:

```
Failed to write 'Torque_Limit' on id_=3 with '700' after 1 tries.
[RxPacketError] Input voltage error!
```

- 電源は 12V5A、実測 **12.2V で完全に正常**
- `Present_Voltage` を全サーボから読むと 6 個とも 12.2V を返す
- なのに id3/4/5 だけステータスのエラービット 0x01（電圧）が立ちっぱなし

## 原因

サーボ EEPROM の **Max_Voltage（addr 14）が id3/4/5 だけ 12.0V** に設定されていた。
実電圧 12.2V がその上限を **0.2V 超過** → 恒久的に電圧エラー。
id1/2/6 は STS3215 の工場出荷値 14.0V だったので無事だった。

12V 電源が無負荷で 12.2V 前後を出すのは正常。**上限 12.0V という設定のほうが狭すぎた。**

## 効かない対処（全部試して無駄だった）

- 電源の抜き差し・再投入 → **消えない**（設定側の問題なので当然）
- reboot 命令(0x08) → 無応答
- Torque_Enable のトグル → 消えない
- `clear_error_information()` 系 → 対象外

エラービットが立っていても **レジスタの読み書き自体は通る**（Torque_Limit を書いて読み戻せる）。
lerobot 側が応答のエラービットを見て例外を投げるので接続が失敗する、という構図。

## 正解

```bash
python scripts/so101_check_voltage_limits.py --port COM13              # 診断
python scripts/so101_check_voltage_limits.py --port COM13 --set-max 14.0  # 修復
```

Max/Min_Voltage と実電圧を突き合わせて判定する。EEPROM 書込は Lock(addr 55)=0 で
解除 → 書込 → Lock=1 で再ロック。lerobot 非依存なのでどの env からでも動く。

## 教訓

**「電圧エラー」を電源の異常と読み替えない。** 実電圧が正常でも、リミット設定を
外れていれば同じエラーが出る。しかも電源側をいくら触っても直らないので、
「挿し直してもダメ」＝ 設定側を見るべきサイン。個体ごとに EEPROM 設定が
揃っていない前提で、まず全 ID の設定値を並べて比較する。

関連: [[so101_bringup]] / 5V 誤給電での沈黙（こちらは本物の給電不足）とは別物。
