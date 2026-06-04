#!/usr/bin/env python3
"""Helper for the /harness-tune skill.

  python .claude/hooks/harness_tune_summary.py             # summarize unreviewed friction
  python .claude/hooks/harness_tune_summary.py --mark-reviewed   # stamp reviewed.json to now

Reads .claude/harness/friction.jsonl, filters to entries after reviewed.json's
last_ts, and prints a grouped summary for the model to reason over. --mark-reviewed
advances last_ts to the newest entry so the SessionStart nudge clears.
"""
import sys, os, json

HDIR = os.path.join(os.environ.get("CLAUDE_PROJECT_DIR", os.getcwd()), ".claude", "harness")
FRICTION = os.path.join(HDIR, "friction.jsonl")
REVIEWED = os.path.join(HDIR, "reviewed.json")


def load_reviewed_ts():
    try:
        with open(REVIEWED, encoding="utf-8") as fh:
            return float(json.load(fh).get("last_ts", 0))
    except Exception:
        return 0.0


def load_unreviewed():
    last = load_reviewed_ts()
    out = []
    if not os.path.exists(FRICTION):
        return out, last
    with open(FRICTION, encoding="utf-8") as fh:
        for ln in fh:
            ln = ln.strip()
            if not ln:
                continue
            try:
                rec = json.loads(ln)
            except Exception:
                continue
            if float(rec.get("ts", 0)) > last:
                out.append(rec)
    return out, last


def main():
    recs, last = load_unreviewed()

    if "--mark-reviewed" in sys.argv:
        newest = max((float(r.get("ts", 0)) for r in recs), default=last)
        os.makedirs(HDIR, exist_ok=True)
        with open(REVIEWED, "w", encoding="utf-8") as fh:
            json.dump({"last_ts": newest}, fh)
        print(f"marked reviewed up to ts={newest} ({len(recs)} entries cleared)")
        return

    if not recs:
        print("摩擦シグナルなし（未対応分は空）。会話から自分で拾うこと。")
        return

    by_kind = {}
    for r in recs:
        for s in r.get("signals", []):
            by_kind.setdefault(s.get("kind"), 0)
            by_kind[s.get("kind")] += 1
    print(f"未対応の摩擦: {len(recs)} ターン  内訳: " +
          ", ".join(f"{k}={v}" for k, v in sorted(by_kind.items())))
    print("---")
    for r in recs[-30:]:
        kinds = "/".join(s.get("kind") for s in r.get("signals", []))
        detail = []
        for s in r.get("signals", []):
            if s.get("kind") == "correction":
                detail.append("phrases:" + ",".join(s.get("phrases", [])))
            elif s.get("kind") == "repeated_errors":
                detail.append(f"errors:{s.get('count')}")
            elif s.get("kind") == "effortful":
                detail.append(f"tools:{s.get('tool_uses')}")
        print(f"[{kinds}] ({'; '.join(detail)})  {r.get('user_snippet','')}")


if __name__ == "__main__":
    main()
