#!/usr/bin/env python3
import os
import time
import threading
from typing import Dict, Any

from ntcore import NetworkTableInstance
from flask import Flask, jsonify, render_template_string, request

TEAM = int(os.environ.get("FRC_TEAM", "0"))  # set to your team number, or 0 to use explicit host
HOST = os.environ.get("NT_HOST", "")         # e.g. "10.XX.YY.2"
POLL_HZ = float(os.environ.get("POLL_HZ", "10"))

app = Flask(__name__)

_state_lock = threading.Lock()
_tests: Dict[str, Dict[str, Any]] = {}  # testId -> fields


def nt_connect():
    inst = NetworkTableInstance.getDefault()
    inst.startClient4("testmode-web")

    if TEAM > 0:
        inst.setServerTeam(TEAM)
        inst.startDSClient()  # recommended when running on DS machine
    elif HOST:
        inst.setServer(HOST, ntcore.NetworkTableInstance.kDefaultPort4)
    else:
        raise RuntimeError("Set FRC_TEAM or NT_HOST")

    return inst


def poll_loop():
    inst = nt_connect()
    root = inst.getTable("testmode").getSubTable("tests")

    while True:
        time.sleep(1.0 / max(POLL_HZ, 1.0))

        # list subtable names under /testmode/tests
        # ntcore Python exposes getSubTables() on NetworkTable
        test_ids = root.getSubTables()

        snapshot: Dict[str, Dict[str, Any]] = {}

        for tid in test_ids:
            t = root.getSubTable(tid)
            snapshot[tid] = {
                "id": tid,
                "name": t.getStringTopic("name").subscribe("").get(),
                "durationSec": t.getDoubleTopic("durationSec").subscribe(0.0).get(),
                "progress": t.getDoubleTopic("progress").subscribe(0.0).get(),
                "status": t.getStringTopic("status").subscribe("PENDING").get(),
                "done": t.getBooleanTopic("done").subscribe(False).get(),
                "pass": t.getBooleanTopic("pass").subscribe(False).get(),
                "message": t.getStringTopic("message").subscribe("").get(),
            }

        with _state_lock:
            _tests.clear()
            _tests.update(snapshot)


@app.get("/api/tests")
def api_tests():
    with _state_lock:
        # stable ordering for UI
        tests = sorted(_tests.values(), key=lambda x: x["id"])
    return jsonify({"tests": tests})


PAGE = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Robot Test Mode</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 24px; }
    .test { border: 1px solid #ddd; border-radius: 10px; padding: 12px 14px; margin-bottom: 12px; }
    .row { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    .name { font-size: 16px; font-weight: 600; }
    .status { font-family: monospace; padding: 2px 8px; border-radius: 8px; }
    .PENDING { background: #eef; }
    .RUNNING { background: #ffe; }
    .PASS { background: #e8ffe8; }
    .FAIL { background: #ffe8e8; }
    .bar { height: 10px; background: #f1f1f1; border-radius: 99px; overflow: hidden; margin-top: 8px; }
    .fill { height: 100%; width: 0%; background: #4a90e2; transition: width 150ms linear; }
    .msg { margin-top: 8px; color: #444; white-space: pre-wrap; }
    .meta { color:#666; font-size: 12px; }
  </style>
</head>
<body>
  <h2>Robot Test Mode</h2>
  <div class="meta">
    Polling: <span id="poll"></span> ms
  </div>
  <div id="tests"></div>

<script>
const pollMs = 250;
document.getElementById("poll").innerText = pollMs;

function esc(s){ return (s ?? "").replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;"); }

function render(tests) {
  const root = document.getElementById("tests");
  if (!tests.length) {
    root.innerHTML = "<div class='meta'>No tests published yet (waiting for /testmode/tests/...)</div>";
    return;
  }

  root.innerHTML = tests.map(t => {
    const pct = Math.max(0, Math.min(1, t.progress || 0)) * 100;
    const st = t.status || "PENDING";
    const msg = t.message || "";
    const subtitle = `id=${t.id} duration=${(t.durationSec||0).toFixed(2)}s done=${t.done} pass=${t.pass}`;
    return `
      <div class="test">
        <div class="row">
          <div class="name">${esc(t.name || t.id)}</div>
          <div class="status ${esc(st)}">${esc(st)}</div>
        </div>
        <div class="bar"><div class="fill" style="width:${pct}%"></div></div>
        <div class="meta">${esc(subtitle)}</div>
        ${msg ? `<div class="msg">${esc(msg)}</div>` : ""}
      </div>
    `;
  }).join("");
}

async function tick() {
  try {
    const r = await fetch("/api/tests");
    const j = await r.json();
    render(j.tests || []);
  } catch (e) {
    document.getElementById("tests").innerHTML =
      "<div class='meta'>Disconnected from server/API.</div>";
  }
}

setInterval(tick, pollMs);
tick();
</script>
</body>
</html>
"""

@app.get("/")
def index():
    return render_template_string(PAGE)


def start_background_poll():
    th = threading.Thread(target=poll_loop, daemon=True)
    th.start()


if __name__ == "__main__":
    start_background_poll()
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", "5805")), debug=False)
