"""
AQUABOT ROV — Dashboard Server  (MQTT edition)
Install : pip install flask paho-mqtt
Run     : python app.py
Access  : http://localhost:5000

Architecture
  Browser  ←→  Flask /api/command  ←→  paho-mqtt  →  HiveMQ broker  →  ESP32
  Browser  ←→  mqtt.js WebSocket   ←   HiveMQ broker  ←  ESP32 telemetry
"""

from flask import Flask, render_template, jsonify, request
from datetime import datetime
import paho.mqtt.client as mqtt
import threading
import json
import time
import math

app = Flask(__name__)

# ── MQTT CONFIG ───────────────────────────────────────────────────────────
BROKER      = "broker.emqx.io"
BROKER_PORT = 8883

# All three command topics — server publishes to all; ESP subscribes to all
CMD_TOPICS   = ["chennai2026", "96260706", "900398"]
STATUS_TOPIC = "chennai2026/status"

# Camera feed (HTTP stream — separate from MQTT)
ESP32_CAM_IP   = "192.168.137.107"
ESP32_CAM_PORT = 81
STREAM_URL     = f"http://{ESP32_CAM_IP}:{ESP32_CAM_PORT}/stream"

# ── SHARED STATE ──────────────────────────────────────────────────────────
_state = {
    "temp": None, "turb": None, "volt": None,
    "rssi": None, "ip": None,
    "m1": 0, "m2": 0, "servo": 90,
    "uptime": 0,
    "broker_ok": False,
    "bot_last_seen": None,
}
_lock = threading.Lock()

# ── PAHO MQTT CLIENT ─────────────────────────────────────────────────────
import random
_mqtt = mqtt.Client(client_id=f"aquabot_{random.randint(10000,99999)}", protocol=mqtt.MQTTv311)
_mqtt.tls_set()

def _on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[MQTT] Connected to {BROKER}")
        client.subscribe(STATUS_TOPIC)
        with _lock:
            _state["broker_ok"] = True
    else:
        print(f"[MQTT] Connect failed rc={rc}")


def _on_disconnect(client, userdata, rc):
    print(f"[MQTT] Disconnected rc={rc}")
    with _lock:
        _state["broker_ok"] = False


def _on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        with _lock:
            _state.update(data)
            _state["bot_last_seen"] = datetime.now().isoformat()
    except Exception as e:
        print(f"[MQTT] Parse error: {e} — payload: {msg.payload}")


_mqtt.on_connect    = _on_connect
_mqtt.on_disconnect = _on_disconnect
_mqtt.on_message    = _on_message


def _mqtt_runner():
    while True:
        try:
            print(f"[MQTT] Connecting to {BROKER}:{BROKER_PORT} …")
            _mqtt.connect(BROKER, BROKER_PORT, keepalive=60)
            _mqtt.loop_forever()
        except Exception as e:
            print(f"[MQTT] Error: {e} — retrying in 5 s")
            with _lock:
                _state["broker_ok"] = False
            time.sleep(5)


threading.Thread(target=_mqtt_runner, daemon=True, name="mqtt").start()


# ── HELPERS ───────────────────────────────────────────────────────────────
def _publish(cmd: str):
    """Publish to every command topic so any backup receives it."""
    for topic in CMD_TOPICS:
        result = _mqtt.publish(topic, cmd)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"[MQTT] Publish to {topic} failed: {result.rc}")
    print(f"[CMD] Published → {cmd!r} on all topics")


def _rssi_to_distance(rssi: int) -> float:
    """
    Free-space path loss model.
    A = -59 dBm at 1 m (typical ESP32 WiFi)
    n = 2.5  (mixed indoor/outdoor environment)
    """
    A, n = -59, 2.5
    return round(10 ** ((A - rssi) / (10 * n)), 1)


# ── ROUTES ────────────────────────────────────────────────────────────────
@app.route("/")
def dashboard():
    return render_template("index.html", stream_url=STREAM_URL)


@app.route("/api/status")
def api_status():
    """Latest telemetry snapshot — also used for initial page load."""
    with _lock:
        snap = dict(_state)
    if snap.get("rssi") is not None:
        snap["distance_m"] = _rssi_to_distance(snap["rssi"])
    snap["server_time"] = datetime.now().strftime("%H:%M:%S")
    return jsonify(snap)


@app.route("/api/command", methods=["POST"])
def api_command():
    """
    Expected body: { "type": "M1"|"M2"|"SERVO"|"STOP_M1"|"STOP_M2"|"EMERGENCY",
                     "value": <int or str> }

    Filtration pump inversion:
      Pump controller treats 255 as 0 speed and 0 as full speed.
      → backend stores desired (0-255) but publishes (255 - desired).
    """
    body = request.get_json(force=True, silent=True) or {}
    cmd_type = body.get("type", "")
    raw      = body.get("value", 0)

    if cmd_type == "M1":
        val = max(0, min(255, int(raw)))
        _publish(f"M1:{val}")
        return jsonify(ok=True, cmd=f"M1:{val}")

    elif cmd_type == "M2":
        desired  = max(0, min(255, int(raw)))
        # !! INVERT — pump controller: 255 = stop, 0 = full speed
        inverted = 255 - desired
        _publish(f"M2:{inverted}")
        return jsonify(ok=True, cmd=f"M2:{inverted}", desired=desired, inverted=inverted)

    elif cmd_type == "SERVO":
        # Receive absolute angle from dashboard (avoids triple-fire drift)
        angle = max(30, min(150, int(raw)))
        _publish(f"SA:{angle}")
        return jsonify(ok=True, cmd=f"SA:{angle}")

    elif cmd_type == "STOP_M1":
        _publish("M1:0")
        return jsonify(ok=True, cmd="M1:0 (thruster off)")

    elif cmd_type == "STOP_M2":
        # Pump: invert 0 → 255 to actually stop
        _publish("M2:255")
        return jsonify(ok=True, cmd="M2:255 (pump off — inverted)")

    elif cmd_type == "EMERGENCY":
        _publish("EMERGENCY")          # ESP handles kill + centre
        return jsonify(ok=True, cmd="EMERGENCY")

    return jsonify(ok=False, error=f"Unknown command type: {cmd_type!r}"), 400


@app.route("/api/time")
def api_time():
    now = datetime.now()
    return jsonify(date=now.strftime("%d %b %Y"), time=now.strftime("%H:%M:%S"))


# ── ENTRY POINT ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 55)
    print("  AQUABOT ROV — Dashboard Server")
    print("=" * 55)
    print(f"  Dashboard : http://localhost:5000")
    print(f"  Broker    : {BROKER}:{BROKER_PORT}")
    print(f"  Topics    : {' | '.join(CMD_TOPICS)}")
    print(f"  Cam Feed  : {STREAM_URL}")
    print("=" * 55)
    # use_reloader=False  → prevents double-starting the MQTT thread
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
