---
name: harness-tune
description: ハーネス自己改善ループの実行役。摩擦シグナル（.claude/harness/friction.jsonl、または会話で気づいた繰り返しミス・非効率）を root cause 分析し、rule/hook/command/skill/memory のどれかに符号化して反映する。使うタイミング：SessionStart hook に「/harness-tune を実行」と促された時／同じミスや非効率を2回踏んだと気づいた時／ユーザーが「ハーネス見直して」「自己改善して」と言った時／時間のかかった成功の後に手順を圧縮したい時。
---

# Harness Tune — 自己改善ループの実行

摩擦（繰り返しミス・非効率・時間のかかった成功）を**恒久的な仕組み**に変換する。単発修正で終わらせない。グローバル規約 `~/.claude/CLAUDE.md` の「自己改善ループ」の実行版。

## 0. 発動経路

- **自動**: Stop hook（`harness_friction_log.py`）が摩擦を `.claude/harness/friction.jsonl` に記録 → 次の SessionStart hook が冒頭で「/harness-tune を実行」と促す。
- **手動/自己**: 同じ指摘を2回受けた・同じ調べ物を3回した・テスト/hook が2連続失敗・時間のかかった成功の直後・ユーザー明示要請。

## 1. シグナルを集める

```bash
# 未対応の摩擦ログ（reviewed.json の last_ts 以降）を要約
python .claude/hooks/harness_tune_summary.py
```

これが無い／追加の文脈が要る時は、`.claude/harness/friction.jsonl` を直接 Read し、直近セッションの会話も振り返る。**ログに出ない摩擦**（ユーザーの言い回し、設計のちぐはぐ、ツール選択ミス）も自分で拾う — ログはあくまで補助。

## 2. クラスタリング & root cause（一言で）

似たシグナルをまとめ、各クラスタの根本原因を1行で言語化する。型は3つ：

| シグナル | 典型 root cause | 直し方の方向 |
|---|---|---|
| `correction`（訂正・「前にも」） | ルール不明確 / コンテキスト不足 / 規約をロードしてない | rule か memory で明文化、強制力が要れば hook |
| `repeated_errors`（連続失敗） | ツール設定不備 / 手順が暗黙知 | command 化、または rule に手順・前提を固定 |
| `effortful`（時間のかかった成功） | 定型なのに毎回手で組み立ててる | command/skill に圧縮（今回の手順を再利用可能に） |

## 3. 符号化先を選ぶ（グローバル表に従う）

| 種類 | 置き場所 | 用途 |
|---|---|---|
| 行動ルール | `.claude/rules/*.md`（`globs:` で path-scope）or `.agent/rules/*.md` | 「BLE後50msディレイ」等の作業規約 |
| 自動実行 | `.claude/hooks/` + `.claude/settings.json` 登録 | 強制力が要るもの |
| 繰り返しタスク | `.claude/commands/*.md` | 複数ステップの定型フロー |
| 作業パターン | `.claude/skills/<name>/SKILL.md` | 判断を要する非定型ワークフロー |
| 文脈・嗜好 | `.claude/memory/*.md` + MEMORY.md 索引 | プロジェクト文脈・ユーザー嗜好・踏んだ罠 |

判断のコツ:
- **1回の罠 → memory**（次に思い出せれば十分）。**2回以上の繰り返し → rule/command/hook**（仕組みで防ぐ）。
- すでに該当 memory/rule があるなら**新規作成せず更新**（重複を作らない）。
- ドキュメントが既にあるなら、そこへ追記して導線（CLAUDE.md からの参照）を確保。

## 4. 反映（自律レベル）

- **自動適用してよい（低リスク）**: memory 追加/更新、rule 追加、command 追加、skill 追加、docs 追記、既存ファイルの導線追加。
- **適用前にユーザー確認（副作用あり）**: **hook の新規追加/変更、`.claude/settings.json` の変更**、依存追加、破壊的操作。理由（何を・なぜ・どの摩擦に対して）を1行で添えて聞く。

反映したら、グローバル規約の Verify を満たす：次セッションで確実にロードされる導線（CLAUDE.md / MEMORY.md / globs）を張る。

## 5. 対応済みマークを刻む（nudge を消す）

符号化が終わったら、レビュー済み位置を更新して SessionStart の催促を止める：

```bash
python .claude/hooks/harness_tune_summary.py --mark-reviewed
```

## 6. 報告

ユーザーに簡潔に: 何の摩擦を / どの root cause で / どこに符号化したか、を箇条書き1〜3行。hook 追加が要るなら提案だけして承認待ち。

---

関連: グローバル `~/.claude/CLAUDE.md`「自己改善ループ」、`docs/QUEST_DEV.md`（過去の摩擦→ドキュ化の実例）、hooks: `harness_friction_log.py`（検知）/ `harness_friction_review.py`（催促）。
