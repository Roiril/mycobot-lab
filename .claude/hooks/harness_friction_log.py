#!/usr/bin/env python3
"""Stop hook — friction detector (silent side-effect only).

Fires at the end of every assistant turn. Reads the session transcript, looks at
the CURRENT turn (last human message + the assistant work after it), and appends
any detected friction signals to .claude/harness/friction.jsonl.

Stop-hook output is NOT injected into the model, so this only records. The
SessionStart hook (harness_friction_review.py) surfaces the accumulated signals
at the next session start, and the /harness-tune skill acts on them.

Signals:
  - correction      : the human used a "you did this wrong / again" phrase
                      → strong sign a harness gap let a repeat mistake through
  - repeated_errors : >=3 tool errors in the turn → flaky workflow worth encoding
  - effortful       : >=20 tool calls, no errors → slow success → candidate to
                      compress into a command/skill next time

Never raises, never blocks. Designed to be cheap (tail of transcript only).
"""
import sys, os, json, time, re

# Human phrases that signal a REPEAT mistake / reprimand (not a normal bug report).
# Kept tight to avoid flagging ordinary "it doesn't work" hardware/bug reports.
CORRECTION_PATTERNS = [
    r"前にも", r"またか", r"また同じ", r"何度も", r"何回も", r"二度と",
    r"いい加減", r"じゃなくて", r"そうじゃない", r"言ったよね", r"って言った",
    r"again and again", r"i (?:already )?told you", r"stop doing", r"like i said",
]
_CORR = re.compile("|".join(CORRECTION_PATTERNS), re.IGNORECASE)

EFFORT_TOOL_THRESHOLD = 20   # tool calls in one turn → "effortful" (slow success)
ERROR_THRESHOLD = 3          # tool errors in one turn → "repeated_errors"


def _text_of(content):
    """Flatten a message 'content' (str or list of blocks) to plain text."""
    if isinstance(content, str):
        return content
    out = []
    if isinstance(content, list):
        for b in content:
            if isinstance(b, dict) and b.get("type") == "text" and isinstance(b.get("text"), str):
                out.append(b["text"])
    return "\n".join(out)


def _is_tool_result(content):
    return isinstance(content, list) and any(
        isinstance(b, dict) and b.get("type") == "tool_result" for b in content)


def main():
    try:
        data = json.load(sys.stdin)
    except Exception:
        return
    tpath = data.get("transcript_path")
    proj = os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or os.getcwd()
    if not tpath or not os.path.exists(tpath):
        return

    try:
        with open(tpath, encoding="utf-8") as fh:
            lines = fh.readlines()[-400:]  # tail only — cheap
    except Exception:
        return

    entries = []
    for ln in lines:
        ln = ln.strip()
        if not ln:
            continue
        try:
            entries.append(json.loads(ln))
        except Exception:
            continue

    # Find the last genuine human message (a user entry that is NOT a tool_result).
    last_human_idx = None
    last_human_text = ""
    for i, e in enumerate(entries):
        msg = e.get("message") or {}
        if e.get("type") == "user" or msg.get("role") == "user":
            content = msg.get("content", e.get("content"))
            if _is_tool_result(content):
                continue
            txt = _text_of(content)
            if txt.strip():
                last_human_idx = i
                last_human_text = txt
    if last_human_idx is None:
        return

    # Current turn = everything after the last human message.
    turn = entries[last_human_idx + 1:]
    tool_uses = 0
    tool_errors = 0
    for e in turn:
        msg = e.get("message") or {}
        content = msg.get("content", e.get("content"))
        if isinstance(content, list):
            for b in content:
                if not isinstance(b, dict):
                    continue
                if b.get("type") == "tool_use":
                    tool_uses += 1
                if b.get("type") == "tool_result" and (b.get("is_error") or b.get("isError")):
                    tool_errors += 1

    signals = []
    corr = _CORR.findall(last_human_text)
    if corr:
        signals.append({"kind": "correction", "phrases": sorted(set(corr))[:4]})
    if tool_errors >= ERROR_THRESHOLD:
        signals.append({"kind": "repeated_errors", "count": tool_errors})
    if tool_uses >= EFFORT_TOOL_THRESHOLD and tool_errors == 0:
        signals.append({"kind": "effortful", "tool_uses": tool_uses})

    if not signals:
        return

    rec = {
        "ts": round(time.time(), 1),
        "session": data.get("session_id"),
        "signals": signals,
        "user_snippet": last_human_text.strip().replace("\n", " ")[:160],
    }
    hdir = os.path.join(proj, ".claude", "harness")
    try:
        os.makedirs(hdir, exist_ok=True)
        with open(os.path.join(hdir, "friction.jsonl"), "a", encoding="utf-8") as fh:
            fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
    except Exception:
        return


if __name__ == "__main__":
    try:
        main()
    except Exception:
        pass  # never block Stop
    sys.exit(0)
