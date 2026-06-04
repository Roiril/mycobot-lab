---
name: browser-ui-testing
description: ブラウザ UI の動作確認方法。MCP の合成イベントは page に届かない→Chrome computer の実 OS キーを使う。キーボード/マウス駆動 UI のテスト全般に流用可
metadata:
  type: technique
---

ブラウザ UI（このプロジェクトの three.js 操作系含む）をエージェントが動作確認する時の鉄則。

## 罠：MCP の合成イベントは page の listener に届かない

```js
// ❌ これは効かない（Chrome 拡張の isolated world で実行されるため、
//    page 側 window.addEventListener('keydown', ...) が発火しない）
window.dispatchEvent(new KeyboardEvent('keydown', {key:'ArrowUp'}))
```

`mcp__Claude_in_Chrome__javascript_tool` のコードは page とは別の JS world で走る。
そこから `dispatchEvent` しても、**自分で同じ eval 内に足した capture listener すら発火しない**ことで確認済み。

## 正解：Chrome computer の実 OS キーイベント

```
mcp__Claude_in_Chrome__computer  action:"key"  text:"Right"  repeat:30
```

これは OS レベルのキー入力なので page handler に正しく届く。マウスも `left_click` 等が実イベント。

### 検証パターン（before/after を fetch + DOM で読む）

1. `javascript_tool` で before 値を `window.__x` に退避（state はモジュールスコープで直接読めないので、
   **スライダ値 = `state.target` の反映** や `fetch('/angles')` = 実機角度 で間接観測）
2. `computer` で `left_click` して canvas にフォーカス（スライダから focus を外す）
3. `computer` で実キー連打（jog は「キー押しっぱ」速度制御なので、`repeat:N` で N 回の短押し＝累積移動）
4. `javascript_tool` で after 値を読み、差分判定

## ハマりどころ

- **jog は held-key 駆動**：単発 down+up は rAF 1 tick 分しか動かない。`repeat` で稼ぐ
- **focus 奪い**：トグルボタンやスライダに focus が残ると矢印キーが別動作になる。先に canvas を click
- モジュールスコープ変数（`liveMode`, `goalSphere` 等）は eval から直接読めない。
  DOM（スライダ/ステータス）や HTTP（/angles, /coords）経由で観測する

## How to apply

「ブラウザで動作チェックして」と言われたら、合成イベントを試さず最初から
`computer action:key/left_click` を使う。状態は DOM か API 経由で読む。
