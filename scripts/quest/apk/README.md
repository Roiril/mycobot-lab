# ロボットハンド操作 ランチャー APK

Meta Quest の App Library（**提供元不明** カテゴリ）に「**ロボットハンド操作**」という名前で出る 2D ランチャーアプリ。タップすると Oculus Browser で `http://localhost:8001/hand` を開き、自身は即終了する。

`localhost` は各 Quest で `adb reverse tcp:8001 tcp:8001` により PC の hand server（:8001）へ転送される前提（`scripts/quest/deploy_hand.py` が設定）。

## 仕様

| 項目 | 値 |
|---|---|
| package | `jp.mycobotlab.handteleop` |
| ラベル | ロボットハンド操作 |
| minSdk / targetSdk | 29 / 32 |
| 開くURL | `http://localhost:8001/hand`（`com.oculus.browser` 指定、無ければ既定ブラウザ） |
| 挙動 | `Theme.NoDisplay` の trampoline。VIEW intent 発行 → `finish()`（UI を出さず終了） |

## ファイル構成

```
apk/
├── AndroidManifest.xml         # package/label/LAUNCHER intent-filter
├── src/jp/mycobotlab/handteleop/LauncherActivity.java
├── res/mipmap-*/ic_launcher.png  # 手のアイコン（gen_icon.py で生成）
├── gen_icon.py                 # アイコン再生成スクリプト（PIL）
├── build.ps1                   # ビルドパイプライン（PowerShell 5.1）
├── .gitignore                  # dist/ *.apk *.keystore を除外
└── dist/                       # 生成物（git 管理外）
    └── handteleop.apk          # 署名済み成果物
```

## ビルド

すべて Unity 同梱の Android SDK / OpenJDK を絶対パスで使う（別途 SDK インストール不要）。

```powershell
powershell -ExecutionPolicy Bypass -File build.ps1
```

パイプライン: `keytool`(初回のみ debug.keystore 生成) → `aapt2 compile` → `aapt2 link` → `javac`(-source/-target 8) → `d8` → `classes.dex` 挿入(python zipfile, STORED) → `zipalign -p 4` → `apksigner sign` → `apksigner verify`。

成果物: `dist\handteleop.apk`（v3 署名スキームで検証通過）。

### アイコンだけ作り直す

```
python gen_icon.py
```

## インストール

接続中の Quest（`adb devices` で確認）に `-r`（再インストール可）で入れる。

```powershell
adb -s 2G0YC1ZF7S06BW install -r dist\handteleop.apk
adb -s 2G0YC1ZF890864 install -r dist\handteleop.apk
```

確認:

```powershell
adb -s <serial> shell pm list packages jp.mycobotlab.handteleop
```

## アンインストール

```powershell
adb -s <serial> uninstall jp.mycobotlab.handteleop
```

## Quest でアプリを見つける手順

1. Quest のホームで **アプリ一覧（App Library / アプリ）** を開く。
2. 右上のカテゴリ（提供元）ドロップダウンを **「提供元不明」**（Unknown Sources）に切り替える。
   - sideload したアプリは Meta ストア外なので、この提供元不明カテゴリにのみ表示される。
3. 一覧に **「ロボットハンド操作」**（手のアイコン）が出る。タップで起動。
   - タップすると Oculus Browser が `http://localhost:8001/hand` を開く。事前に PC 側で hand server 起動 + `adb reverse tcp:8001 tcp:8001` が済んでいること（`deploy_hand.py`）。

## 注意

- `debug.keystore` はビルド時に自動生成し **commit しない**（.gitignore 済み）。別 PC でビルドすると別鍵になり、既存インストールを `-r` 更新できない場合がある（その時は一度 uninstall）。
- 署名は v3 のみ（minSdk 29 なので v1 JAR 署名は不要）。
