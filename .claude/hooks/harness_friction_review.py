#!/usr/bin/env python3
"""SessionStart hook — surface accumulated friction so the harness self-improves.

Reads .claude/harness/friction.jsonl (written by the Stop hook), counts signals
NOT yet reviewed (after reviewed.json's last_ts), and if they cross a threshold
prints a short reminder to stdout. SessionStart stdout IS injected into the model
as a system reminder, so the next session starts knowing it should run
/harness-tune.

Quiet by default: only nudges when there's a real pattern (any correction, or
>=3 flagged turns). The /harness-tune skill stamps reviewed.json so the nudge
clears once acted upon. Never raises.
"""
import sys, os, json

# Force UTF-8 stdout: the nudge contains non-cp932 chars (⚠, 日本語); on Windows
# the default console encoding would make print() raise and the nudge would be
# silently lost. The harness reads hook stdout as UTF-8.
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

NUDGE_MIN_TURNS = 3   # flagged turns needed to nudge when no correction present


def main():
    try:
        data = json.load(sys.stdin)
    except Exception:
        data = {}
    # Only on real session boundaries, not mid-session compaction.
    if data.get("source") == "compact":
        return
    proj = os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or os.getcwd()
    hdir = os.path.join(proj, ".claude", "harness")
    fpath = os.path.join(hdir, "friction.jsonl")
    if not os.path.exists(fpath):
        return

    last_ts = 0.0
    try:
        with open(os.path.join(hdir, "reviewed.json"), encoding="utf-8") as fh:
            last_ts = float(json.load(fh).get("last_ts", 0))
    except Exception:
        pass

    corrections = 0
    repeated = 0
    effortful = 0
    flagged = 0
    samples = []
    try:
        with open(fpath, encoding="utf-8") as fh:
            for ln in fh:
                ln = ln.strip()
                if not ln:
                    continue
                try:
                    rec = json.loads(ln)
                except Exception:
                    continue
                if float(rec.get("ts", 0)) <= last_ts:
                    continue
                kinds = [s.get("kind") for s in rec.get("signals", [])]
                if not kinds:
                    continue
                flagged += 1
                if "correction" in kinds:
                    corrections += 1
                if "repeated_errors" in kinds:
                    repeated += 1
                if "effortful" in kinds:
                    effortful += 1
                if len(samples) < 4 and rec.get("user_snippet"):
                    samples.append(f"  - [{'/'.join(kinds)}] {rec['user_snippet']}")
    except Exception:
        return

    if flagged == 0:
        return
    if corrections == 0 and flagged < NUDGE_MIN_TURNS:
        return  # not enough to bother

    lines = [
        "⚠ ハーネス自己改善: 前回までに未対応の摩擦シグナルがあります "
        f"(訂正 {corrections} / 連続失敗 {repeated} / 高コスト成功 {effortful}、計 {flagged} ターン)。",
        "手が空いたら **/harness-tune** を実行して、root cause を rule/hook/command/skill/memory に符号化してください。",
    ]
    if samples:
        lines.append("直近の例:")
        lines.extend(samples)
    print("\n".join(lines))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        pass
    sys.exit(0)
