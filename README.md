# mycobot-lab

myCobot 320-M5 を Python から動かす実験プロジェクト。

## セットアップ

```bash
pip install -r requirements.txt
```

## 使い方

1. アームの STOP を解除して電源ON
2. M5 画面で `Transponder` → `USB UART` → OK（画面が `Connect test / Atom: ok` に切り替わる）
3. 接続確認:
   ```bash
   python scripts/check.py
   ```
4. デモ動作:
   ```bash
   python scripts/move.py
   ```

## 構成

詳細は [CLAUDE.md](CLAUDE.md) を参照。
