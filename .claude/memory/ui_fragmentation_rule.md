---
name: ui-fragmentation-rule
description: UI は 3 HTML + 2 サーバに分かれている。新 UI 追加時の統合チェックリスト（相互リンク・トークンコピー・docs 表更新）
metadata:
  type: project
---

UI 構成（2026-06-10 時点）: `ui.html`(:8000 メイン) / `hand.html`(同サーバ `/hand`) / `so101.html`(:8011 別サーバ)。SO-101 分離は bring-up 用の意図的措置で、Phase 2 で server.py へ統合予定（[[so101_bringup]]）。

**Why:** 過去にロボットを追加するたび別ページ・別サーバを増設し、相互導線・ドキュメント反映を怠った結果「機能が散らばって不便」というユーザー指摘が摩擦ログに繰り返し記録された（friction 5 件中 3 件が UI 散在系）。

**How to apply:**
- 新しいロボット UI を作る時は必ず: ① ui.html status bar に行きリンク、② サブ UI にメイン UI への戻りリンク、③ ui.html の `:root` トークンをコピー（正本は ui.html）、④ `docs/ARCHITECTURE.md` §2.1b の UI/サーバ表に追記、⑤ `.agent/rules/design.md` 冒頭のマルチ UI 注意に従う。
- 機能追加はまず「既存 ui.html の 5 タブのどこに入るか」を検討し、別ページ化は Quest 軽量化など明確な理由がある時だけ。
- Quest 開発時のポートは 8001（QUEST_DEV.md 慣習）。hand.html の案内文を 8000 に"修正"しないこと（過去に一度誤修正しかけた）。
