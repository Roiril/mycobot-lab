# Vision 統合 Phase 1 — VLM 検出と平面投影 (PoC)

実装日: 2026-05-24
担当: シュビー (Claude Code, Opus 4.7)

## 目的

myCobot 320 に視覚を与え、自然言語クエリで物体位置を world 座標 (base 系) として返す API を作る。Phase 1 は **PoC レベル**：

- 手首カメラ 1 台のみ
- VLM (Claude vision) で bbox 取得
- ピクセル → カメラ光線 → テーブル平面交点で 3D 位置推定
- placeholder キャリブレーション（合理的デフォルトで動く）
- offline で仮想物体検出可能

## 確定設計

### モジュール構成

`src/arm/vision/` package：
- `transforms.py` — SE(3) helper、`T_base_ee`, `T_base_cam_wrist`
- `localizer.py` — `pixel_to_ray`, `intersect_plane`, `localize_on_table`, `estimate_radius`
- `camera.py` — `Camera` dataclass + `CameraRegistry`
- `detector.py` — `Detection`, `Detector` ABC, `ClaudeVLMDetector`, `FixtureDetector`
- `__init__.py` — public re-export

`src/arm/vision_hub.py` — `VisionHub` / `VirtualVisionHub`、motion hub から状態を引いて perceive

### API

- `GET /cameras` — 登録カメラ一覧
- `GET /frame.jpg?cam=<id>` — 既存拡張 (cam 省略時は wrist 後方互換)
- `POST /perceive` — クエリ → world 座標物体リスト + retry_hints

### キャリブレーション

`data/calibration.json` (commit) — placeholder：
- intrinsics: fx=fy=500, cx=320, cy=240 (640x480)
- hand_eye_T_ee_cam: ツール前方 30mm、光軸 +z (tool 方向)
- table_z_mm = 0

### 依存追加

- `anthropic>=0.40.0`
- `opencv-contrib-python-headless>=4.8` ← 既存 `opencv-python-headless` 置換 (ChArUco 用に Phase 2 で活用)

### エラーコード

- `OBJECT_NOT_FOUND` / `LOW_CONFIDENCE` / `MULTIPLE_AMBIGUOUS` / `OCCLUDED` / `DEPTH_UNCERTAIN` / `VLM_API_ERROR` / `CALIBRATION_MISSING`
- 各 retry_hints に action+rationale

## 決定事項 (実装中の選択)

- **VLM model**: `claude-sonnet-4-6`（最新 vision + コスト中庸）
- **画像 encode**: 640x480 JPEG q=85 → base64
- **bbox 中心ピクセル**を localize の起点（重心ではなく単純中心、Phase 1）
- **size_class → radius**: small/medium/large = 15/25/40mm 上限。bbox 直径 + depth で幾何推定し、min を採用
- **複数カメラ統合**: Phase 1 は confidence 上位 1 件のみ採用（クラスタリング無し）
- **timeout**: VLM call = 15s
- **`recommended_speed`**: confidence ≥ 0.8 → 20, 0.5-0.8 → 10, < 0.5 → 5
- **`consensus`** flag は受け取って ignore（Phase 2 で実装）

## テスト

- `tests/test_vision_localizer.py` — 純関数 (pixel_to_ray / intersect_plane / estimate_radius)
- `tests/test_vision_perceive.py` — VirtualVisionHub + FixtureDetector の E2E
- 既存テスト全 pass

## Phase 2 への申し送り

- ChArUco による intrinsics + hand-eye 自動校正 (`scripts/calibrate_charuco.py`)
- 複数カメラの統合（cluster + IoU で同一物体判定）
- `consensus: true` の 2 回連続検出 + 一致確認
- depth refinement: 物体表面の事前形状仮定（球/円柱）で平面交点を補正
- 物体追跡 (TrackedObject)、`/perceive_refine` (細部 zoom-in 再撮影)
- overhead カメラの追加（手首 + 俯瞰の 2 系統で精度向上）

## 未解決

- ANTHROPIC_API_KEY は env var で渡す前提。CI でテストする場合 FixtureDetector のみで進める設計に
- hand-eye の手測 placeholder は実機で合わない可能性大 → Phase 2 校正必須
- table_z 推定の自動化（現在は固定 0mm）

## レビュー後 fix wave 反映 (同日)

4 視点レビュー (安全 / UX / アーキ / コード品質) で抽出された HIGH 修正を一括反映：

### 安全層
- **placeholder calibration の motion 流出阻止**: `placeholder=true` のカメラからの world 座標は標準で拒否 (`CALIBRATION_PLACEHOLDER_ONLY` 新コード, `terminal: true`)。`allow_uncalibrated: true` 明示時のみ通過、object に `uncalibrated: true` フラグ
- **workspace cube サニティ**: 新定数 `WORKSPACE_REACH_MAX_MM=380`, `WORKSPACE_Z_MAX_MM=500`、`FLOOR_Z` 下限。範囲外は除外 or `OUT_OF_WORKSPACE` 新コード。混在時は inside のみ返却 + diagnostics 件数
- **table_z 起動時 warning**: `table_z_mm < FLOOR_Z - 50` で stderr に警告
- **VLM 例外 sanitize**: response message は `type(e).__name__` のみ。詳細は `logging.exception` でサーバログに。`_extract_json` 失敗時は 200 文字 truncate

### VLM / bbox 堅牢化
- **scale_to_orig バグ修正**: `_bgr_to_jpeg_b64` が orig/sent サイズ両方を返し、detector 内部で `scale_to_orig` を算出して bbox を元解像度に戻す。`image_meta.scale_to_orig` は撤廃
- **bbox 検証/クランプ**: NaN/Inf, 負サイズ, 中心が画像外 → skip。一部はみ出しは frame 内に clamp
- **NaN/Inf 防御**: confidence 非有限を skip、`json.loads(parse_constant=lambda c: None)` で NaN/Infinity を None 化
- **Phase 2 hooks 明示拒否**: `consensus`/`refine` を warnings に通知して ignore

### アーキ整合
- **/perceive 入力検証エラー envelope 統一**: `{ok:false, error:{code:"BAD_REQUEST", message, diagnostics, retry_hints}}` に揃えた

### AI agent UX
- **OBSERVE 姿勢追加**: `poses.OBSERVE` `OBSERVE_LEFT` `OBSERVE_RIGHT` `OBSERVE_HIGH` を新規追加、全て `check_angles` を通る (tests/test_poses_observe.py)
- **retry_hints に具体 angles**: `_observe_retry_hints()` ヘルパで各エラーコードに OBSERVE 系の `suggested_move` patch を埋め込む
- **LOW_CONFIDENCE 順序入替**: `observe_from_another_angle` を 1 番目、`lower_confidence_threshold` は fallback として 2 番目に
- **CALIBRATION_*, VLM_API_ERROR (auth) に terminal フラグ**: 再試行不能を明示

### docs
- `docs/AGENT_API.md` §8 を §8.1-8.8 に拡張、完全な呼出しシーケンス・信頼度バンド・OBSERVE 姿勢表・terminal エラー表・リカバリ例
- `docs/ARCHITECTURE.md` §9.4 に placeholder guard / workspace check / bbox sanitize / VLM sanitize を追記

### テスト
- `tests/test_poses_observe.py` 4 件
- `tests/test_vision_workspace_check.py` 2 件
- `tests/test_vision_bbox_clamp.py` 7 件
- `tests/test_vision_placeholder_guard.py` 3 件
- 既存 73 + 新規 16 = 計 90 件 pass
