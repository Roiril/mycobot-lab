---
name: ui-fragmentation-rule
description: UI は 3 HTML + 2 サーバに分かれている。新 UI 追加時の統合チェックリスト（相互リンク・トークンコピー・docs 表更新）
metadata:
  type: project
---

UI 構成（2026-06-10 統合完了）: 1 サーバ（server.py）+ 統一 ui.html（6 タブ、SO-101 含む）+ hand.html（`/hand`、Quest 軽量版のみ例外）。旧 so101_server.py(:8011)/so101.html は廃止（[[so101_bringup]]）。

**Why:** 過去にロボットを追加するたび別ページ・別サーバを増設し、相互導線・ドキュメント反映を怠った結果「機能が散らばって不便」というユーザー指摘が摩擦ログに繰り返し記録された（friction 5 件中 3 件が UI 散在系）。

**How to apply:**
- **新しいロボットは server.py のサブシステム（`So101Subsystem` が雛形: lazy-init + `/robot名/*` 名前空間）+ ui.html のワークスペースタブとして足す**。別サーバ・別ページを新設しない。
- 別ページ化は Quest 軽量化など明確な理由がある時だけ。その場合: ① 戻り/行きリンク相互、② ui.html の `:root` トークンをコピー、③ `docs/ARCHITECTURE.md` §2.1b の表に追記。
- Quest 開発時のポートは 8001（QUEST_DEV.md 慣習）。hand.html の案内文を 8000 に"修正"しないこと（過去に一度誤修正しかけた）。
