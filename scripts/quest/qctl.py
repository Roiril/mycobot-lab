"""Quest WebXR dev control over CDP (Chrome DevTools Protocol).

Canonical, repo-resident replacement for the old throwaway helpers that lived
in %TEMP%\\claude\\xr\\ (volatile — got wiped; also shadowed by a stray
Temp\\inspect.py). Keep all Quest dev tooling HERE.

Why CDP and not `adb am start VIEW`: the Oculus Browser opens a NEW TAB on every
VIEW intent (tabs pile up → GPU pressure → jank). CDP `Page.reload` updates the
existing tab in place. See .claude/memory/feedback_quest_reload_protocol.md.

Prereqs (see docs/QUEST_DEV.md for the full loop):
  - Quest connected via USB, developer mode on.
  - CDP forwarded:   adb -s <quest> forward tcp:9223 localabstract:chrome_devtools_remote
  - Server reverse:  adb -s <quest> reverse tcp:8001 tcp:8001
  - pip install websocket-client

Usage:
  python scripts/quest/qctl.py reload      # end VR -> Page.reload -> wait -> reinject recorder  (DO THIS after editing ui.html)
  python scripts/quest/qctl.py check        # dump page state (VR mode, badges, in-VR, hook, hand us)
  python scripts/quest/qctl.py nav <url>    # navigate the same tab (no new tab)
  python scripts/quest/qctl.py end          # end the immersive-vr session
  python scripts/quest/qctl.py install      # (re)inject the recorder/fetch hook only
  python scripts/quest/qctl.py tabs         # list / collapse to a single localhost tab

Options: --port N (server port, default 8001) --cdp N (CDP port, default 9223)
"""
from __future__ import annotations
import json, sys, time, urllib.request

try:
    import websocket  # websocket-client
except ImportError:
    sys.exit("pip install websocket-client  (required for CDP)")

CDP_PORT = 9223
SRV_PORT = 8001


def _cdp(path=""):
    return json.load(urllib.request.urlopen(f"http://localhost:{CDP_PORT}/json{path}"))


def pages():
    return [p for p in _cdp("/list") if p.get("type") == "page"]


def target_page():
    host = f"localhost:{SRV_PORT}"
    p = next((p for p in pages() if host in p.get("url", "")), None)
    if p is None:
        p = next((p for p in pages() if "localhost" in p.get("url", "")), None)
    return p


def connect(p):
    return websocket.create_connection(p["webSocketDebuggerUrl"], timeout=8, suppress_origin=True)


def ev(ws, expr, await_promise=False):
    ws.send(json.dumps({"id": 1, "method": "Runtime.evaluate",
                        "params": {"expression": expr, "returnByValue": True,
                                   "awaitPromise": await_promise}}))
    return json.loads(ws.recv()).get("result", {}).get("result", {}).get("value")


# Recorder + /jog fetch hook. Reinstalled after every reload (the page reload
# wipes window.__log / __jogLog / __origFetch).
RECORDER = r"""
(function(){
  if (window.__recInterval) clearInterval(window.__recInterval);
  if (window.__origFetch) window.fetch = window.__origFetch;
  window.__log = []; window.__jogLog = [];
  const t0 = performance.now(); window.__t0 = t0;
  window.__recInterval = setInterval(() => {
    const t = Math.round(performance.now() - t0);
    const s = window.__state || {}, x = window.__xr || {};
    window.__log.push({ t,
      pin: x.pinching ? (x.pinching.right?'R':(x.pinching.left?'L':'-')) : '?',
      target:  s.target  ? s.target.map(v=>+v.toFixed(1)) : null,
      current: s.current ? s.current.map(v=>+v.toFixed(1)) : null });
  }, 50);
  window.__origFetch = window.fetch;
  window.fetch = function(url, opts){
    if (typeof url === 'string' && url.endsWith('/jog')) {
      const tSend = Math.round(performance.now() - t0);
      let sentAngles = null;
      try { sentAngles = JSON.parse(opts.body).angles.map(v=>+v.toFixed(1)); } catch(e){}
      const p = window.__origFetch.apply(this, arguments);
      p.then(r => r.clone().json().then(j => {
        const tAck = Math.round(performance.now() - t0);
        window.__jogLog.push({ tSend, tAck, latency: tAck-tSend, sent: sentAngles,
          ok: !!j.ok, code: j.code || (j.ok ? 'OK' : 'ERR'), mode: j.mode });
      })).catch(()=> window.__jogLog.push({tSend, tAck:-1, sent:sentAngles, ok:false, code:'NET_ERR'}));
      return p;
    }
    return window.__origFetch.apply(this, arguments);
  };
  return 'recorder + /jog hook installed';
})()
"""

CHECK_EXPR = r"""({
  url: location.href,
  vrMode: (document.querySelector('.vrModeBtn.active')||{}).dataset ? document.querySelector('.vrModeBtn.active').dataset.vrmode : null,
  vrBadge: (document.getElementById('vrBadge')||{}).textContent,
  handBadge: (document.getElementById('handBadge')||{}).textContent,
  handStatus: (document.getElementById('handStatus')||{}).textContent,
  inVR: !!(window.__xr && window.__xr.session),
  hookInstalled: typeof window.__origFetch === 'function',
})"""


def cmd_end(ws):
    try:
        return ev(ws, '(async()=>{ if(window.__xr&&window.__xr.session){await window.__xr.session.end(); return "ended";} return "not in vr"; })()', True)
    except Exception:
        return "end-failed"


def main():
    global CDP_PORT, SRV_PORT
    args = sys.argv[1:]
    # option parse
    out = []
    i = 0
    while i < len(args):
        if args[i] == "--port": SRV_PORT = int(args[i+1]); i += 2
        elif args[i] == "--cdp": CDP_PORT = int(args[i+1]); i += 2
        else: out.append(args[i]); i += 1
    args = out
    sub = args[0] if args else "check"

    if sub == "tabs":
        ps = pages()
        host = f"localhost:{SRV_PORT}"
        keep = next((p for p in ps if host in p.get("url", "")), ps[0] if ps else None)
        for p in ps:
            if p is not keep:
                try: urllib.request.urlopen(f"http://localhost:{CDP_PORT}/json/close/{p['id']}").read()
                except Exception: pass
        print(f"kept 1 tab: {keep['url'] if keep else None} (closed {len(ps)-1})")
        return

    p = target_page()
    if not p:
        sys.exit(f"no localhost:{SRV_PORT} page found over CDP (is the tab open? adb forward 9223 set?)")

    if sub == "reload":
        ws = connect(p)
        print("end VR:", cmd_end(ws))
        ws.send(json.dumps({"id": 2, "method": "Page.reload", "params": {"ignoreCache": True}}))
        try: ws.recv()
        except Exception: pass
        ws.close()
        print("reloaded in-place; waiting for boot...")
        time.sleep(3.5)
        p2 = target_page(); ws = connect(p2)
        print("reinject:", ev(ws, RECORDER)); ws.close()
    elif sub == "install":
        ws = connect(p); print(ev(ws, RECORDER)); ws.close()
    elif sub == "end":
        ws = connect(p); print(cmd_end(ws)); ws.close()
    elif sub == "nav":
        url = args[1] if len(args) > 1 else f"http://localhost:{SRV_PORT}/"
        ws = connect(p)
        ws.send(json.dumps({"id": 3, "method": "Page.navigate", "params": {"url": url}}))
        try: ws.recv()
        except Exception: pass
        ws.close(); print("navigated:", url)
    elif sub == "check":
        ws = connect(p)
        st = ev(ws, CHECK_EXPR); ws.close()
        print(json.dumps(st, ensure_ascii=False, indent=2))
    else:
        sys.exit(f"unknown subcommand: {sub}\n{__doc__}")


if __name__ == "__main__":
    main()
