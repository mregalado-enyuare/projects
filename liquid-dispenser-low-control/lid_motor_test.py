#!/usr/bin/env python3
"""
Lid stepper motor test webapp for liquid-dispenser-low-control.

Usage:
    pip install flask          # one-time
    python3 lid_motor_test.py
    open http://localhost:5051
"""

import argparse
import json
import socket
import threading
import time
import uuid
from dataclasses import dataclass, field
from queue import Empty, Queue
from typing import Optional

from flask import Flask, Response, jsonify, request

# ── Config ────────────────────────────────────────────────────────────────────
STM32_UDP_PORT = 9000
LISTEN_PORT    = 9000
ACK_TIMEOUT_S       = 2.0
DONE_TIMEOUT_S      = 30.0   # moves
DONE_TIMEOUT_CAL_S  = 120.0  # calibration (limit seek can take time)
HTTP_PORT      = 5051

# ── Shared state ──────────────────────────────────────────────────────────────
settings: dict = {
    "stm32_ip": "10.10.1.255",
    "rack_num": 4,
}


@dataclass
class PendingCmd:
    cmd_id:     str
    sent_at:    float
    ack_at:     Optional[float] = None
    result:     Optional[str]   = None
    done_event: threading.Event = field(default_factory=threading.Event)


pending:      dict[str, PendingCmd] = {}
pending_lock: threading.Lock        = threading.Lock()
log_queue:    Queue                 = Queue()
send_sock:    Optional[socket.socket] = None


# ── Logging helper ────────────────────────────────────────────────────────────
def _log(msg: str, kind: str = "info") -> None:
    ts = time.strftime("%H:%M:%S")
    log_queue.put({"ts": ts, "msg": msg, "kind": kind})


# ── UDP listener thread ───────────────────────────────────────────────────────
def udp_listener() -> None:
    global send_sock

    recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass
    recv.bind(("0.0.0.0", LISTEN_PORT))
    recv.settimeout(1.0)

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    _log(f"UDP listener ready on 0.0.0.0:{LISTEN_PORT}", "info")

    while True:
        try:
            data, addr = recv.recvfrom(8192)
        except socket.timeout:
            continue
        except Exception as exc:
            _log(f"UDP recv error: {exc}", "error")
            continue

        try:
            pkt = json.loads(data.decode())
        except Exception as e:
            _log(f"UDP parse error from {addr[0]}: {e} | raw={data.decode()[:80]}", "error")
            continue

        cmd    = pkt.get("cmd", "")
        cmd_id = pkt.get("cmd_id", "")

        if cmd == "ACK":
            with pending_lock:
                p = pending.get(cmd_id)
            if p:
                p.ack_at = time.time()
                _log(f"ACK  {cmd_id[:8]}…  latency={int((p.ack_at - p.sent_at)*1000)}ms  from={addr[0]}", "ack")
            else:
                _log(f"ACK  {cmd_id[:8]}…  (unsolicited)  from={addr[0]}", "ack")

        elif cmd == "DONE":
            msg = pkt.get("msg", "")
            with pending_lock:
                p = pending.get(cmd_id)
            if p:
                p.result = msg
                p.done_event.set()
                kind = "done_ok" if "OK" in msg else "done_err"
                move_ms = f"  move={int((time.time() - p.ack_at)*1000)}ms" if p.ack_at else ""
                _log(f"DONE {cmd_id[:8]}…  {msg}{move_ms}  from={addr[0]}", kind)
            else:
                kind = "done_ok" if "OK" in msg else "done_err"
                _log(f"DONE {cmd_id[:8]}…  {msg}  (unsolicited)  from={addr[0]}", kind)

            try:
                ack_pkt = json.dumps({
                    "cmd":      "DONE_ACK",
                    "cmd_id":   cmd_id,
                    "rack_num": settings["rack_num"],
                }).encode()
                send_sock.sendto(ack_pkt, (settings["stm32_ip"], STM32_UDP_PORT))
            except Exception:
                pass

        else:
            _log(f"UDP  {data.decode()[:120]}  from={addr[0]}", "info")


# ── Send helper ───────────────────────────────────────────────────────────────
def send_and_wait(payload: dict, done_timeout: float = DONE_TIMEOUT_S) -> dict:
    if send_sock is None:
        return {"ok": False, "error": "UDP socket not ready"}

    cmd_id = str(uuid.uuid4())
    payload = {**payload, "cmd_id": cmd_id, "rack_num": settings["rack_num"]}

    entry = PendingCmd(cmd_id=cmd_id, sent_at=time.time())
    with pending_lock:
        pending[cmd_id] = entry

    _log(f"SEND {cmd_id[:8]}…  {json.dumps(payload)}", "send")

    try:
        send_sock.sendto(json.dumps(payload).encode(),
                         (settings["stm32_ip"], STM32_UDP_PORT))
    except Exception as exc:
        with pending_lock:
            pending.pop(cmd_id, None)
        _log(f"Send error: {exc}", "error")
        return {"ok": False, "error": str(exc)}

    # Wait for ACK
    deadline = time.time() + ACK_TIMEOUT_S
    while entry.ack_at is None and time.time() < deadline:
        time.sleep(0.02)

    if entry.ack_at is None:
        with pending_lock:
            pending.pop(cmd_id, None)
        _log(f"TIMEOUT waiting for ACK on {cmd_id[:8]}… — check IP and rack_num", "error")
        return {"ok": False, "error": "No ACK from board (check IP and rack_num)"}

    ack_latency_ms = int((entry.ack_at - entry.sent_at) * 1000)

    # Wait for DONE
    got_done = entry.done_event.wait(timeout=done_timeout)
    done_at  = time.time()

    with pending_lock:
        pending.pop(cmd_id, None)

    if not got_done:
        return {"ok": False, "error": f"Timed out waiting for DONE ({done_timeout:.0f}s)",
                "ack_latency_ms": ack_latency_ms}

    move_ms = int((done_at - entry.ack_at) * 1000)
    ok      = entry.result is not None and "OK" in entry.result
    return {
        "ok":             ok,
        "result":         entry.result,
        "ack_latency_ms": ack_latency_ms,
        "move_ms":        move_ms,
    }


# ── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.config["PROPAGATE_EXCEPTIONS"] = True


@app.get("/")
def index():
    return HTML, 200, {"Content-Type": "text/html; charset=utf-8"}


@app.post("/api/settings")
def api_settings():
    body = request.get_json(force=True)
    settings["stm32_ip"] = str(body.get("stm32_ip", settings["stm32_ip"]))
    settings["rack_num"] = int(body.get("rack_num", settings["rack_num"]))
    _log(f"Settings → ip={settings['stm32_ip']}  rack={settings['rack_num']}", "info")
    return jsonify(ok=True)


_CAL_CMDS = {"LID_TRANSLATE_CALIBRATE", "LID_SEAL_CALIBRATE"}

@app.post("/api/cmd")
def api_cmd():
    payload  = request.get_json(force=True)
    timeout  = DONE_TIMEOUT_CAL_S if payload.get("cmd") in _CAL_CMDS else DONE_TIMEOUT_S
    result   = send_and_wait(payload, done_timeout=timeout)
    return jsonify(result)


@app.get("/api/stream")
def api_stream():
    def generate():
        while True:
            try:
                item = log_queue.get(timeout=15)
                yield f"data: {json.dumps(item)}\n\n"
            except Empty:
                yield 'data: {"kind":"ping"}\n\n'
    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


# ── Embedded HTML/CSS/JS ──────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Lid Motor Test</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: system-ui, -apple-system, sans-serif;
  background: #0f1117; color: #e2e8f0; font-size: 14px;
}
h1  { font-size: 1.2rem; font-weight: 600; }
h2  { font-size: .9rem;  font-weight: 600; color: #a0aec0; text-transform: uppercase; letter-spacing: .5px; }
.wrap { max-width: 960px; margin: 0 auto; padding: 20px 16px 200px; }
.hdr  { display: flex; align-items: center; gap: 12px; margin-bottom: 18px; }
.badge {
  background: #1e3a5f; color: #63b3ed; font-size: 11px; font-weight: 700;
  padding: 3px 9px; border-radius: 12px; letter-spacing: .4px;
}
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 12px; }
@media (max-width: 640px) { .grid { grid-template-columns: 1fr; } }
.card {
  background: #151a27; border: 1px solid #2a3249; border-radius: 10px;
  padding: 16px 18px;
}
.card-title { display: flex; align-items: center; gap: 8px; margin-bottom: 12px; }
.dot {
  width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0;
  background: #4299e1;
}
.dot.green  { background: #48bb78; }
.dot.orange { background: #ed8936; }
.dot.purple { background: #9f7aea; }
.row { display: flex; flex-wrap: wrap; gap: 8px; align-items: flex-end; margin-bottom: 8px; }
.fg  { display: flex; flex-direction: column; }
label { color: #718096; font-size: 11px; margin-bottom: 3px; text-transform: uppercase; letter-spacing: .4px; }
input[type=number] {
  background: #1e2535; border: 1px solid #3a4563; border-radius: 6px;
  color: #e2e8f0; padding: 6px 8px; font-size: 13px; width: 90px;
}
input:focus { outline: 2px solid #4299e1; border-color: transparent; }
.btn-row { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 4px; }
button {
  background: #2b6cb0; color: #fff; border: none; border-radius: 6px;
  padding: 6px 14px; font-size: 13px; font-weight: 500; cursor: pointer;
  white-space: nowrap;
}
button:hover:not(:disabled) { background: #3182ce; }
button:disabled { opacity: .4; cursor: not-allowed; }
button.cal  { background: #744210; }
button.cal:hover:not(:disabled) { background: #975a16; }
button.suction { background: #276749; }
button.suction:hover:not(:disabled) { background: #2f855a; }
button.suction.off { background: #742a2a; }
button.suction.off:hover:not(:disabled) { background: #9b2c2c; }
button.save { background: #276749; }
button.save:hover:not(:disabled) { background: #2f855a; }
.result {
  margin-top: 8px; padding: 8px 12px; border-radius: 6px;
  font-size: 12.5px; line-height: 1.6; display: none;
}
.result.ok  { background: #162a1e; border: 1px solid #276749; color: #68d391; }
.result.err { background: #2d1515; border: 1px solid #742a2a; color: #fc8181; }
.conn-card { margin-bottom: 12px; }
.conn-card .row { margin-bottom: 0; }
/* Log panel */
.log-wrap {
  position: fixed; bottom: 0; left: 0; right: 0; height: 185px;
  background: #0a0d13; border-top: 1px solid #2a3249; display: flex; flex-direction: column;
}
.log-title {
  padding: 5px 14px; font-size: 10.5px; font-weight: 600; color: #4a5568;
  letter-spacing: .5px; flex-shrink: 0; border-bottom: 1px solid #1a1f2e;
  display: flex; gap: 16px;
}
#log {
  flex: 1; overflow-y: auto; padding: 4px 14px 8px;
  font-family: 'Menlo', 'Consolas', monospace; font-size: 11.5px; line-height: 1.55;
}
.l-send { color: #4299e1; }
.l-ack  { color: #4a5568; }
.l-ok   { color: #48bb78; }
.l-err  { color: #fc8181; }
.l-info { color: #718096; }
</style>
</head>
<body>
<div class="wrap">

  <div class="hdr">
    <h1>Lid Motor Test</h1>
    <span class="badge">liquid-dispenser-low-control</span>
    <span id="inflight" style="display:none;margin-left:auto;background:#744210;color:#f6ad55;
      font-size:11px;font-weight:700;padding:3px 10px;border-radius:12px;letter-spacing:.4px">
      ⏳ <span id="inflight_cmd"></span>
    </span>
  </div>

  <!-- Connection -->
  <div class="card conn-card">
    <div class="row">
      <div class="fg"><label>STM32 IP</label>
        <input type="text" id="stm32_ip" value="10.10.1.255" style="width:140px"></div>
      <div class="fg"><label>Rack #</label>
        <input type="number" id="rack_num" value="4" min="1" max="32" style="width:60px"></div>
      <button class="save" onclick="saveSettings()">Save</button>
    </div>
  </div>

  <div class="grid">

    <!-- LID TRANSLATE -->
    <div class="card">
      <div class="card-title"><div class="dot"></div><h2>Lid Translate</h2></div>
      <div class="row">
        <div class="fg"><label>Motor ID</label>
          <input type="number" id="tr_motor_id" value="1" min="1" max="2" style="width:60px"></div>
        <div class="fg"><label>Steps</label>
          <input type="number" id="tr_steps" value="500"></div>
        <div class="fg"><label>Speed (Hz)</label>
          <input type="number" id="tr_speed" value="2000"></div>
        <div class="fg"><label>Accel</label>
          <input type="number" id="tr_accel" value="500"></div>
      </div>
      <div class="row">
        <div class="fg"><label>Cal Speed (Hz)</label>
          <input type="number" id="tr_cal_speed" value="2000"></div>
      </div>
      <div class="btn-row">
        <button onclick="lidTranslateMove()">&#9654; Move</button>
        <button class="cal" onclick="lidTranslateCal()">&#8962; Calibrate</button>
      </div>
      <div id="tr_result" class="result"></div>
    </div>

    <!-- LID ROTATE -->
    <div class="card">
      <div class="card-title"><div class="dot green"></div><h2>Lid Rotate</h2></div>
      <div class="row">
        <div class="fg"><label>Motor ID</label>
          <input type="number" id="ro_motor_id" value="1" min="1" max="2" style="width:60px"></div>
        <div class="fg"><label>Steps</label>
          <input type="number" id="ro_steps" value="200"></div>
        <div class="fg"><label>Speed (Hz)</label>
          <input type="number" id="ro_speed" value="1500"></div>
        <div class="fg"><label>Accel</label>
          <input type="number" id="ro_accel" value="400"></div>
      </div>
      <div class="btn-row">
        <button onclick="lidRotateMove()">&#9654; Move</button>
      </div>
      <div id="ro_result" class="result"></div>
    </div>

    <!-- LID SEAL -->
    <div class="card">
      <div class="card-title"><div class="dot orange"></div><h2>Lid Seal</h2></div>
      <div class="row">
        <div class="fg"><label>Steps</label>
          <input type="number" id="sl_steps" value="300"></div>
        <div class="fg"><label>Speed (Hz)</label>
          <input type="number" id="sl_speed" value="2000"></div>
        <div class="fg"><label>Accel</label>
          <input type="number" id="sl_accel" value="500"></div>
      </div>
      <div class="row">
        <div class="fg"><label>Cal Speed (Hz)</label>
          <input type="number" id="sl_cal_speed" value="2000"></div>
      </div>
      <div class="btn-row">
        <button onclick="lidSealMove()">&#9654; Move</button>
        <button class="cal" onclick="lidSealCal()">&#8962; Calibrate</button>
      </div>
      <div id="sl_result" class="result"></div>
    </div>

    <!-- LID SUCTION -->
    <div class="card">
      <div class="card-title"><div class="dot purple"></div><h2>Lid Suction</h2></div>
      <p style="color:#718096;font-size:12.5px;margin-bottom:12px">
        Controls LID_SUCTION_FET (PD0). Toggle vacuum on/off.
      </p>
      <div class="btn-row">
        <button class="suction" onclick="suctionOn()">&#9650; Suction ON</button>
        <button class="suction off" onclick="suctionOff()">&#9660; Suction OFF</button>
      </div>
      <div id="su_result" class="result"></div>
    </div>

  </div><!-- /grid -->
</div><!-- /wrap -->

<!-- Live log -->
<div class="log-wrap">
  <div class="log-title">
    LIVE UDP LOG
    <span class="l-send">blue=sent</span>
    <span class="l-ack">gray=ACK</span>
    <span class="l-ok">green=DONE OK</span>
    <span class="l-err">red=error/stall</span>
  </div>
  <div id="log"></div>
</div>

<script>
// ── SSE log stream ──────────────────────────────────────────────────────────
function appendLog(item) {
  if (item.kind === 'ping') return;
  const el  = document.getElementById('log');
  const div = document.createElement('div');
  const cls = { send:'l-send', ack:'l-ack', done_ok:'l-ok',
                done_err:'l-err', error:'l-err', info:'l-info' }[item.kind] || 'l-info';
  div.className = cls;
  div.textContent = (item.ts ? `[${item.ts}] ` : '') + item.msg;
  el.appendChild(div);
  el.scrollTop = el.scrollHeight;
}

const es = new EventSource('/api/stream');
es.onmessage = e => { try { appendLog(JSON.parse(e.data)); } catch(_){} };
es.onerror   = () => appendLog({msg:'SSE disconnected — refresh page', kind:'error'});

// ── Helpers ─────────────────────────────────────────────────────────────────
function numVal(id) { return parseInt(document.getElementById(id).value) || 0; }
function strVal(id) { return document.getElementById(id).value; }

function showResult(id, ok, msg) {
  const d = document.getElementById(id);
  d.className = 'result ' + (ok ? 'ok' : 'err');
  d.textContent = ok ? `✓ ${msg}` : `✗ ${msg}`;
  d.style.display = 'block';
}

async function send(payload, resultId) {
  document.querySelectorAll('button').forEach(b => b.disabled = true);
  const isCal = payload.cmd && payload.cmd.includes('CALIBRATE');
  const banner = document.getElementById('inflight');
  document.getElementById('inflight_cmd').textContent =
    isCal ? `${payload.cmd} running…` : payload.cmd;
  banner.style.display = '';

  let d;
  try {
    const r = await fetch('/api/cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload),
    });
    d = await r.json();
  } catch(e) {
    d = {ok: false, error: String(e)};
  }

  banner.style.display = 'none';
  document.querySelectorAll('button').forEach(b => b.disabled = false);

  const msg = d.ok
    ? `OK  move=${d.move_ms}ms  ack=${d.ack_latency_ms}ms`
    : (d.error || d.result || 'unknown error');
  showResult(resultId, d.ok, msg);
}

// ── Settings ────────────────────────────────────────────────────────────────
async function saveSettings() {
  await fetch('/api/settings', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ stm32_ip: strVal('stm32_ip'), rack_num: numVal('rack_num') }),
  });
}

// ── Lid Translate ────────────────────────────────────────────────────────────
function lidTranslateMove() {
  send({
    cmd:      'LID_TRANSLATE_MOVE',
    motor_id: numVal('tr_motor_id'),
    steps:    numVal('tr_steps'),
    speed:    numVal('tr_speed'),
    accel:    numVal('tr_accel'),
  }, 'tr_result');
}
function lidTranslateCal() {
  send({
    cmd:      'LID_TRANSLATE_CALIBRATE',
    motor_id: numVal('tr_motor_id'),
    speed:    numVal('tr_cal_speed'),
  }, 'tr_result');
}

// ── Lid Rotate ───────────────────────────────────────────────────────────────
function lidRotateMove() {
  send({
    cmd:      'LID_ROTATE_MOVE',
    motor_id: numVal('ro_motor_id'),
    steps:    numVal('ro_steps'),
    speed:    numVal('ro_speed'),
    accel:    numVal('ro_accel'),
  }, 'ro_result');
}

// ── Lid Seal ─────────────────────────────────────────────────────────────────
function lidSealMove() {
  send({
    cmd:   'LID_SEAL_MOVE',
    steps: numVal('sl_steps'),
    speed: numVal('sl_speed'),
    accel: numVal('sl_accel'),
  }, 'sl_result');
}
function lidSealCal() {
  send({ cmd: 'LID_SEAL_CALIBRATE', speed: numVal('sl_cal_speed') }, 'sl_result');
}

// ── Lid Suction ───────────────────────────────────────────────────────────────
function suctionOn()  { send({ cmd: 'LID_SUCTION_ON' },  'su_result'); }
function suctionOff() { send({ cmd: 'LID_SUCTION_OFF' }, 'su_result'); }
</script>
</body>
</html>"""


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lid motor test webapp")
    parser.add_argument("--port", type=int, default=HTTP_PORT,
                        help=f"HTTP port (default {HTTP_PORT})")
    args = parser.parse_args()

    t = threading.Thread(target=udp_listener, daemon=True)
    t.start()
    time.sleep(0.3)

    print(f"\nLid Motor Test")
    print(f"Open:  http://localhost:{args.port}")
    print(f"Stop:  Ctrl-C\n")

    app.run(host="0.0.0.0", port=args.port, threaded=True, use_reloader=False)
