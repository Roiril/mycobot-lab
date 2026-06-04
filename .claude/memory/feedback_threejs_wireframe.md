---
name: feedback-threejs-wireframe
description: ワイヤーフレーム表現は Mesh+wireframe:true ではなく LineSegments+EdgesGeometry を使う
metadata:
  type: feedback
---

Three.js でワイヤーフレーム見た目を作る時、安易に `MeshBasicMaterial({wireframe:true})` を使うと
**内部三角形のエッジまですべて描画される**ため、ジオメトリが画面に占める割合が大きくなるほど
GPU overdraw が爆発し、近距離でカクつく。

**代わりに `THREE.LineSegments + THREE.EdgesGeometry`** を使うと「面の境界線（隣接面の角度が
閾値以上の辺）」だけが線として描画され、描画コストが激減する。視覚的にもクリーンになる。

```js
// ❌ overdraw 爆発
new THREE.Mesh(new THREE.SphereGeometry(r, 18, 14),
               new THREE.MeshBasicMaterial({wireframe: true}))

// ✅ 軽量・クリーン
new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.SphereGeometry(r, 10, 7)),
  new THREE.LineBasicMaterial({color: 0xffa040}))
```

**Why:**
`scripts/ui.html` の `targetObj`（把持対象ワイヤ球）でこの罠を踏み、ズームインで FPS が一桁まで落ちた。
セグメント数を減らしても根本解決にならず、EdgesGeometry 化で完全解消。

**How to apply:**
- 「ワイヤーフレーム見た目が欲しい」と思ったらまず EdgesGeometry を検討
- `wireframe:true` を使うのは「Mesh 全部のエッジを見たい / デバッグ用」だけにする
- Points 系も `PointsMaterial({size, sizeAttenuation:true, map:円形tex, alphaTest:0.5})` で円スプライト化すると見た目良くて軽い
