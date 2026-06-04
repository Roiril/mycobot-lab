# UI デザイン規約（mycobot-lab control UI）

`scripts/ui.html` の視覚言語。**VSCode dark テーマ × フラット × デザイントークン駆動**。couple-sync の規律（単一アクセント・装飾最小・トークン一元管理）を VSCode 配色で適用したもの。UI/CSS を変更する前に必ず読む。

## 原則

1. **トークン経由のみ** — 色・サイズ・余白・角丸は `:root`（ui.html 冒頭 `<style>`）の変数で指定。**`#rrggbb` の直書き禁止**（例外: 3D viz と対応する legend スウォッチ＝意味エンコード色）
2. **単一アクセント + 機能ステータス色** — ブランド/アクティブ = `--accent`（シアン `#4ec9ff`）/ プライマリ塗り = `--accent-bg`（青）。状態色は `--ok`(緑) / `--warn`(琥珀) / `--danger`(赤) **のみ**。装飾目的で緑/紫/teal 等を増やさない
3. **フラット** — `backdrop-filter` / glassmorphism / glow `box-shadow` 新規禁止。カードは単色 surface + 1px border、強調は左 3px `--accent-line` border
4. **直線主義** — 角丸は `--radius`(4px) / `--radius-sm`(3px) 既定。8px/20px を増やさない
5. **静かなデフォルト** — ボタン既定は surface 塗り、active/primary だけアクセント。CTA を巨大にしない
6. **アニメは小さく** — `--t-fast`(.1s) / `--t-std`(.16s)、`--ease`(ease-out)。glow パルス・浮遊・回転禁止

## トークン早見

| 種別 | 変数 |
|---|---|
| 面 | `--bg` `--surface` `--surface-2` `--surface-3` `--border` `--border-strong` `--divider` |
| 文字 | `--text` `--text-muted` `--text-faint` `--text-inv` |
| アクセント | `--accent`(highlight/active) `--accent-bg`/`--accent-bg-hover`(filled) `--accent-soft`/`--accent-line` |
| 状態 | `--ok`/`--ok-text` `--warn`/`--warn-text` `--danger`/`--danger-hover`/`--danger-text`/`--danger-soft` |
| データ(mono) | `--data-num`(#9cdcfe) `--data-tgt`(#ce9178) `--data-ok` |
| 型 | `--font-ui` `--font-mono` / `--fs-2xs..md` / `--fw-reg/med/bold` |
| 形・間 | `--radius` `--radius-sm` `--bw` `--pad` `--gap` |

## コンポーネント規約

- **button**: 既定=surface-3 塗り+border+muted。`.primary`/`.apply`=accent-bg 塗り。`.abort`/`.warn`=danger。トグル系(`modeBtn`/`vrModeBtn`/`topModeBtn`/`opModeBtn`/`poseBtn`/`poseLibBtn`)は active=`--accent-soft`地+`--accent`文字+`--accent-line`枠で**統一**（個別色を足さない）
- **badge**: `.ok/.warn/.err/.gray/.teal`。teal=アクセントの別名（独立色ではない）
- **panel**: `.panel` は折りたたみ対応（h2 クリック→`.collapsed`、localStorage 永続）。独自トグルを持つ h2 は accordion init で除外する
- **status bar**: `#statusBar` は sticky 常時表示。システム状態（接続/通電/オフライン）の一覧性を担う
- **入力**: bg=`--bg`、focus で border=`--accent`（glow 無し）
- **アーム/ハンドの区別は色でなくアイコン(🦾/✋)+ラベル**で。2 つ目のアクセント色を作らない

## 禁止事項

- `#rrggbb` 直書き（legend/3D 対応色を除く）／ 装飾での複数アクセント色併用
- 新規 `backdrop-filter` / glow box-shadow / グラデーション装飾
- インライン `style` で飽和色のボタン・パネル背景を作る（トークン化 or クラス化する）
- 角丸 8px+ を例外外の要素に付ける

## 変更後の検証

ui.html は都度読み直し（サーバ再起動不要）。preview で computed style を確認（`getComputedStyle`）。Quest 反映は [docs/QUEST_DEV.md] の `/quest-reload`。
