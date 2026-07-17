"""mycobot-lab home / launcher server (stdlib only — runs on any Python).

One page that lists every UI entry point in this project (unified server,
SO-101 cockpit, SO-101 calibration, hand wiring diagram), shows which ones
are currently up (server-side probe, so no browser CORS issues), links to
them, and gives copy-paste start commands with COM/port exclusivity notes.

Run:  python scripts/home_server.py
Open: http://localhost:8010/
"""
from __future__ import annotations
import json
import sys
import time
import pathlib
import threading
import urllib.request
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

HERE = pathlib.Path(__file__).resolve().parent
UI_HTML = HERE / "home.html"
WIRING_HTML = HERE.parent / "hand" / "wiring.html"

# Quest one-click launch helpers (adb reverse/forward + CDP). Imported lazily-safe:
# scripts/quest is a sibling of this file.
sys.path.insert(0, str(HERE / "quest"))
try:
    import hand_launch  # noqa: E402
except Exception:  # pragma: no cover - keep the launcher usable even if import fails
    hand_launch = None

# Overall wall-clock cap for the /quest/launch-hand handler (adb/CDP can hang).
QUEST_LAUNCH_DEADLINE_S = 18.0
QUEST_LAUNCH_JOIN_S = 20.0

# ports probed for liveness (key -> url)
PROBES = {
    "8000": "http://127.0.0.1:8000/",
    "8001": "http://127.0.0.1:8001/",
    "8012": "http://127.0.0.1:8012/cal/state",
    "8013": "http://127.0.0.1:8013/state",
}

LOCK = threading.Lock()
STATUS = {k: False for k in PROBES}


def probe_once(url: str, timeout: float = 0.8) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return 200 <= r.status < 500
    except Exception:
        return False


def prober():
    while True:
        for key, url in PROBES.items():
            ok = probe_once(url)
            with LOCK:
                STATUS[key] = ok
        time.sleep(2.0)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, body: bytes, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, UI_HTML.read_bytes(), "text/html; charset=utf-8")
        elif path == "/status":
            with LOCK:
                self._send(200, json.dumps(STATUS).encode())
        elif path == "/wiring":
            if WIRING_HTML.exists():
                self._send(200, WIRING_HTML.read_bytes(), "text/html; charset=utf-8")
            else:
                self._send(404, b'{"error":"hand/wiring.html not found"}')
        else:
            self._send(404, b'{"error":"not found"}')

    def do_POST(self):
        path = urlparse(self.path).path
        # Consume any request body so keep-alive stays in sync (there is none, but be safe).
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n:
                self.rfile.read(n)
        except Exception:
            pass
        if path == "/quest/launch-hand":
            self._quest_launch_hand()
        else:
            self._send(404, b'{"error":"not found"}')

    def _quest_launch_hand(self):
        """Open the ✋ hand-teleop page (/hand) on every connected Quest, one click.
        Never blocks the handler indefinitely: the adb/CDP work runs on a worker
        thread joined with a hard cap."""
        result = {"hand_server_up": probe_once(PROBES["8001"])}

        if hand_launch is None:
            result.update(ok=False,
                          error="hand_launch モジュールを読み込めませんでした（scripts/quest）。",
                          devices=[])
            result["message"] = result["error"]
            self._send(200, json.dumps(result, ensure_ascii=False).encode())
            return

        if not result["hand_server_up"]:
            # Do NOT auto-start it — just report.
            result.update(ok=False,
                          error="hand server が起動していません（:8001）。"
                                "teleop_all.ps1 か server.py --real-hand を先に起動してください。",
                          devices=[])
            result["message"] = result["error"]
            self._send(200, json.dumps(result, ensure_ascii=False).encode())
            return

        box = {}

        def work():
            try:
                deadline = time.monotonic() + QUEST_LAUNCH_DEADLINE_S
                box["r"] = hand_launch.launch_hand_all(srv_port=8001, cdp_base=9223,
                                                       deadline=deadline)
            except Exception as e:  # pragma: no cover
                box["r"] = {"ok": False, "error": f"内部エラー: {e}", "devices": []}

        th = threading.Thread(target=work, daemon=True)
        th.start()
        th.join(QUEST_LAUNCH_JOIN_S)
        if th.is_alive():
            result.update(ok=False,
                          error=f"adb/CDP がタイムアウトしました（{int(QUEST_LAUNCH_JOIN_S)}s）。"
                                "USB 接続を確認してください。",
                          devices=[])
        else:
            result.update(box.get("r", {"ok": False, "error": "結果が取得できませんでした。",
                                        "devices": []}))
        result["message"] = _quest_launch_message(result)
        self._send(200, json.dumps(result, ensure_ascii=False).encode())


_ACTION_JA = {
    "launched_native": "VR アプリ起動（被るだけで操作可）",
    "already_native": "VR アプリ起動済み（被るだけで操作可）",
    "navigated": "既存タブを /hand に遷移",
    "already": "すでに /hand 表示中",
    "launched": "アプリ起動（新規タブ）",
    "error": "失敗",
}

# Native = the app enters VR on launch (no in-headset tap). Browser = the WebXR
# /hand page still needs a "VR 開始" tap once worn.
_NATIVE_ACTIONS = ("launched_native", "already_native")


def _quest_launch_message(result: dict) -> str:
    """Short human summary for the launcher card. The lead sentence changes with
    HOW the hand teleop was opened: native VR app (no tap) vs WebXR page (tap)."""
    if not result.get("ok"):
        return result.get("error") or "起動に失敗しました。"
    devices = result.get("devices", [])
    parts = []
    for d in devices:
        tag = _ACTION_JA.get(d.get("action"), d.get("action", "?"))
        detail = f"（{d['detail']}）" if d.get("action") == "error" and d.get("detail") else ""
        parts.append(f"{d.get('serial', '?')[:8]}…: {tag}{detail}")
    actions = [d.get("action") for d in devices]
    if actions and all(a in _NATIVE_ACTIONS for a in actions):
        head = "HMD で VR アプリを起動しました。被るだけで操作できます。"
    elif any(a in _NATIVE_ACTIONS for a in actions):
        head = ("HMD で VR アプリを起動しました（被るだけで操作可）。"
                "ブラウザ版で開いた台はヘッドセット内で「VR 開始」をタップしてください。")
    else:
        head = "HMD でページを開きました。ヘッドセット内で「VR 開始」をタップしてください。"
    return head + ("  " + " / ".join(parts) if parts else "")


def main():
    threading.Thread(target=prober, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", 8010), Handler)
    print("[HOME] -> http://127.0.0.1:8010/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
