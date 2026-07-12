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
import time
import pathlib
import threading
import urllib.request
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

HERE = pathlib.Path(__file__).resolve().parent
UI_HTML = HERE / "home.html"
WIRING_HTML = HERE.parent / "hand" / "wiring.html"

# ports probed for liveness (key -> url)
PROBES = {
    "8000": "http://127.0.0.1:8000/",
    "8001": "http://127.0.0.1:8001/",
    "8012": "http://127.0.0.1:8012/cal/state",
    "8013": "http://127.0.0.1:8013/state",
}

LOCK = threading.Lock()
STATUS = {k: False for k in PROBES}


def prober():
    while True:
        for key, url in PROBES.items():
            ok = False
            try:
                with urllib.request.urlopen(url, timeout=0.8) as r:
                    ok = 200 <= r.status < 500
            except Exception:
                ok = False
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
