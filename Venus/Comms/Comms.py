#!/usr/bin/env python3
"""Venus Group 36 – Robot Communication Monitor."""

import json
import math
import queue
import time
import tkinter as tk
from dataclasses import dataclass, field
from typing import Optional

import paho.mqtt.client as mqtt

# ── MQTT credentials ──────────────────────────────────────────────────────────
BROKER   = "mqtt.ics.ele.tue.nl"
PORT     = 1883
MODULE   = "robot_24_1"
USERNAME = "robot_24_1"
PASSWORD = "yfbB4j7l"

TOPIC_RX = f"/pynqbridge/{MODULE}/send"   # robot publishes here  (we subscribe)
TOPIC_TX = f"/pynqbridge/{MODULE}/recv"   # robot subscribes here (we publish)
# ─────────────────────────────────────────────────────────────────────────────

# Canvas / map parameters
MAP_M   = 3.0      # map spans ±1.5 m in each direction
STALE_S = 8.0      # seconds before robot is considered offline

# Rock colour → tkinter fill colour
ROCK_COLORS: dict[str, str] = {
    "red":     "#cc3333",
    "green":   "#33aa33",
    "blue":    "#3366cc",
    "white":   "#eeeeee",
    "black":   "#222222",
    "unknown": "#999999",
}

# Terrain event → marker colour
EVENT_COLORS: dict[str, str] = {
    "cliff":    "#cc3333",
    "boundary": "#ff8800",
    "obstacle": "#884400",
}


@dataclass
class RobotState:
    status:    str             = "waiting"
    ready:     bool            = False
    x:         float           = 0.0
    y:         float           = 0.0
    theta:     float           = 0.0
    air_temp:  float           = 0.0
    moving:    bool            = False
    done:      bool            = False
    last_seen: Optional[float] = None
    path:      list            = field(default_factory=list)
    rocks:     list            = field(default_factory=list)   # rock_sample dicts
    events:    list            = field(default_factory=list)   # event dicts


class App:
    def __init__(self) -> None:
        self.state = RobotState()
        self._q: queue.Queue = queue.Queue()

        self.root = tk.Tk()
        self.root.title(f"Venus Robot Monitor – {MODULE}")
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self._build_ui()
        self._mqtt = self._make_mqtt()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        # ── top info bar ──
        info = tk.Frame(self.root, padx=10, pady=8)
        info.pack(fill=tk.X)

        self.lbl_conn   = tk.Label(info, text="MQTT: connecting…", anchor="w", fg="gray")
        self.lbl_robot  = tk.Label(info, text=f"Robot {MODULE}: waiting", anchor="w")
        self.lbl_coords = tk.Label(info, text="Position: x=0.000 m  y=0.000 m  θ=0.000 rad", anchor="w")
        self.lbl_temp   = tk.Label(info, text="Temperature: — °C", anchor="w")
        self.lbl_motion = tk.Label(info, text="Motion: —", anchor="w")
        self.lbl_rocks  = tk.Label(info, text="Rocks detected: 0", anchor="w")
        self.lbl_age    = tk.Label(info, text="Last message: never", anchor="w", fg="gray")

        self.btn_start = tk.Button(
            info, text="Send START",
            command=self._send_start,
            width=14, state=tk.DISABLED,
            bg="#cc3333", fg="white", activebackground="#991111",
        )

        labels = [self.lbl_conn, self.lbl_robot, self.lbl_coords,
                  self.lbl_temp, self.lbl_motion, self.lbl_rocks, self.lbl_age]
        for row, w in enumerate(labels):
            w.grid(row=row, column=0, sticky="w")
        self.btn_start.grid(row=0, column=1, rowspan=4, padx=(20, 0), sticky="e")
        info.columnconfigure(0, weight=1)

        # ── map canvas ──
        self.cw, self.ch = 720, 440
        self._margin = 40
        self._scale  = (min(self.cw, self.ch) - 2 * self._margin) / MAP_M

        self.canvas = tk.Canvas(
            self.root,
            width=self.cw, height=self.ch,
            bg="#f8f8f8", highlightthickness=1, highlightbackground="#bbbbbb",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 4))

        # ── detection log ──
        log_frame = tk.Frame(self.root, padx=10, pady=4)
        log_frame.pack(fill=tk.X)
        tk.Label(log_frame, text="Detection log:", anchor="w").pack(anchor="w")

        log_inner = tk.Frame(log_frame)
        log_inner.pack(fill=tk.X)
        self._log = tk.Text(
            log_inner, height=5, state=tk.DISABLED,
            font=("TkFixedFont", 9), bg="#f0f0f0",
        )
        scrollbar = tk.Scrollbar(log_inner, command=self._log.yview)
        self._log.config(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._log.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self._redraw()

    # ── MQTT ─────────────────────────────────────────────────────────────────

    def _make_mqtt(self) -> mqtt.Client:
        import os
        uid = f"venus-monitor-{MODULE}-{os.getpid()}"
        if hasattr(mqtt, "CallbackAPIVersion"):
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=uid)
        else:
            client = mqtt.Client(client_id=uid)
        client.username_pw_set(USERNAME, PASSWORD)
        client.on_connect    = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message    = self._on_message
        client.reconnect_delay_set(min_delay=2, max_delay=15)
        return client

    def _on_connect(self, client, userdata, flags, rc) -> None:
        del userdata, flags
        if rc == 0:
            client.subscribe(TOPIC_RX, qos=0)
            self._q.put(("conn", True, f"connected – listening on {TOPIC_RX}"))
        else:
            self._q.put(("conn", False, f"connect failed (rc={rc}), retrying…"))

    def _on_disconnect(self, client, userdata, rc) -> None:
        del client, userdata
        self._q.put(("conn", False, f"disconnected (rc={rc}), reconnecting…"))

    def _on_message(self, client, userdata, msg) -> None:
        del client, userdata
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
        if isinstance(data, dict):
            self._q.put(("data", data))

    def _send_start(self) -> None:
        info = self._mqtt.publish(
            TOPIC_TX, json.dumps({"cmd": "start"}), qos=1
        )
        if info.rc == mqtt.MQTT_ERR_SUCCESS:
            self.lbl_robot.config(text=f"Robot {MODULE}: start command sent")
        else:
            self.lbl_robot.config(text=f"Robot {MODULE}: publish error rc={info.rc}")

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self) -> None:
        try:
            self._mqtt.connect_async(BROKER, PORT, keepalive=30)
            self._mqtt.loop_start()
        except OSError as exc:
            self.lbl_conn.config(text=f"MQTT: {exc}", fg="red")

        self.root.after(150,  self._poll_queue)
        self.root.after(1000, self._tick)
        self.root.mainloop()

    def _close(self) -> None:
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        self.root.destroy()

    def _poll_queue(self) -> None:
        try:
            while True:
                item = self._q.get_nowait()
                if item[0] == "conn":
                    _, ok, text = item
                    self.lbl_conn.config(text=f"MQTT: {text}",
                                         fg="green" if ok else "orange")
                elif item[0] == "data":
                    self._handle_data(item[1])
        except queue.Empty:
            pass
        self.root.after(150, self._poll_queue)

    def _tick(self) -> None:
        s = self.state
        if s.last_seen is not None:
            age = time.time() - s.last_seen
            stale = age > STALE_S
            self.lbl_age.config(
                text=f"Last message: {age:.1f} s ago",
                fg="red" if stale else "gray",
            )
            self.btn_start.config(
                state=tk.DISABLED if (stale or not s.ready) else tk.NORMAL
            )
        self.root.after(1000, self._tick)

    # ── Data handling ─────────────────────────────────────────────────────────

    def _handle_data(self, data: dict) -> None:
        s = self.state
        s.last_seen = time.time()

        msg_type = data.get("type", "")

        # ── status ──
        if msg_type in ("status", "ready"):
            status = str(data.get("status", msg_type))
            s.status = status
            if status in ("ready", "waiting_start"):
                s.ready = True

        # ── telemetry (periodic pose + temperature) ──
        elif msg_type in ("telemetry", "telemetry_test"):
            s.status = "running"

        # ── rock sample detected ──
        elif msg_type == "rock_sample":
            rock = {
                "x":       float(data.get("coordX", s.x)),
                "y":       float(data.get("coordY", s.y)),
                "color":   str(data.get("color", "unknown")).lower(),
                "size_mm": int(data.get("size_mm", 0)),
                "temp_c":  float(data.get("temp_c", 0.0)),
            }
            s.rocks.append(rock)
            size_label = "small (3 cm)" if rock["size_mm"] <= 30 else "large (6 cm)"
            self._append_log(
                f"[ROCK]  {rock['color']:6s}  {size_label}  "
                f"{rock['temp_c']:5.1f} °C  "
                f"@ ({rock['x']:.2f}, {rock['y']:.2f}) m"
            )
            self.lbl_rocks.config(text=f"Rocks detected: {len(s.rocks)}")

        # ── terrain event (cliff / boundary / obstacle) ──
        elif msg_type == "event":
            event = {
                "x":     float(data.get("coordX", s.x)),
                "y":     float(data.get("coordY", s.y)),
                "event": str(data.get("event", "unknown")),
            }
            s.events.append(event)
            self._append_log(
                f"[EVENT] {event['event']:10s}  "
                f"@ ({event['x']:.2f}, {event['y']:.2f}) m"
            )

        # ── shared pose fields (present in telemetry, status+ready, rock_sample) ──
        if "coordX"  in data: s.x        = float(data["coordX"])
        if "coordY"  in data: s.y        = float(data["coordY"])
        if "theta"   in data: s.theta    = float(data["theta"])
        if "airTemp" in data: s.air_temp = float(data["airTemp"])

        if isinstance(data.get("moving"), bool): s.moving = data["moving"]
        if isinstance(data.get("done"),   bool): s.done   = data["done"]

        pt = (s.x, s.y)
        if not s.path or s.path[-1] != pt:
            s.path.append(pt)

        self._refresh_labels()
        self._redraw()

        if s.ready:
            self.btn_start.config(state=tk.NORMAL)

    def _refresh_labels(self) -> None:
        s = self.state
        self.lbl_robot.config(text=f"Robot {MODULE}: {s.status}")
        self.lbl_coords.config(
            text=f"Position:  x={s.x:.3f} m   y={s.y:.3f} m   θ={s.theta:.3f} rad"
        )
        if s.air_temp != 0.0:
            self.lbl_temp.config(text=f"Temperature: {s.air_temp:.1f} °C")
        motion_txt = "moving"  if s.moving else "stopped"
        done_txt   = "done"    if s.done   else "in progress"
        self.lbl_motion.config(text=f"Motion: {motion_txt}, {done_txt}")

    def _append_log(self, text: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self._log.config(state=tk.NORMAL)
        self._log.insert(tk.END, f"{ts}  {text}\n")
        self._log.see(tk.END)
        self._log.config(state=tk.DISABLED)

    # ── Canvas ────────────────────────────────────────────────────────────────

    def _w2px(self, x: float, y: float) -> tuple:
        """World coordinates (metres) → canvas pixel coordinates."""
        cx = self.cw / 2 + x * self._scale
        cy = self.ch / 2 - y * self._scale
        return cx, cy

    def _redraw(self) -> None:
        self.canvas.delete("all")
        s = self.state

        # ── background grid ──
        half = MAP_M / 2
        step = 0.5
        d = -half
        while d <= half + 1e-9:
            x0, y0 = self._w2px(d, -half)
            x1, y1 = self._w2px(d,  half)
            self.canvas.create_line(x0, y0, x1, y1, fill="#e0e0e0")
            x0, y0 = self._w2px(-half, d)
            x1, y1 = self._w2px( half, d)
            self.canvas.create_line(x0, y0, x1, y1, fill="#e0e0e0")
            d += step

        # ── origin axes ──
        ox, oy = self._w2px(0, 0)
        self.canvas.create_line(ox - 6, oy, ox + 6, oy, fill="#aaaaaa", width=1)
        self.canvas.create_line(ox, oy - 6, ox, oy + 6, fill="#aaaaaa", width=1)
        self.canvas.create_text(ox + 8, oy - 2, text="0", anchor="w",
                                fill="#aaaaaa", font=("TkFixedFont", 8))

        # ── terrain events ──
        for ev in s.events:
            ex, ey = self._w2px(ev["x"], ev["y"])
            color  = EVENT_COLORS.get(ev["event"], "#888888")
            r = 7
            # Draw an X marker
            self.canvas.create_line(ex - r, ey - r, ex + r, ey + r,
                                    fill=color, width=2)
            self.canvas.create_line(ex + r, ey - r, ex - r, ey + r,
                                    fill=color, width=2)
            self.canvas.create_text(ex, ey - r - 8, text=ev["event"][:3].upper(),
                                    fill=color, font=("TkFixedFont", 7))

        # ── rock samples ──
        for rock in s.rocks:
            rx_px, ry_px = self._w2px(rock["x"], rock["y"])
            fill   = ROCK_COLORS.get(rock["color"], "#999999")
            outline = "#ffffff" if rock["color"] == "black" else "#222222"
            # Square marker sized by rock (large rocks slightly bigger)
            half_s = 10 if rock["size_mm"] > 30 else 7
            self.canvas.create_rectangle(
                rx_px - half_s, ry_px - half_s,
                rx_px + half_s, ry_px + half_s,
                fill=fill, outline=outline, width=2,
            )
            # Label: colour initial + temperature
            label = f"{rock['color'][0].upper()} {rock['temp_c']:.0f}°"
            self.canvas.create_text(rx_px, ry_px + half_s + 9,
                                    text=label, fill="#333333",
                                    font=("TkFixedFont", 7))

        # ── robot path trail ──
        if len(s.path) >= 2:
            pts: list = []
            for px, py in s.path:
                cx, cy = self._w2px(px, py)
                pts += [cx, cy]
            self.canvas.create_line(*pts, fill="#999999", width=2, smooth=False)

        # ── robot body ──
        rbx, rby = self._w2px(s.x, s.y)
        r    = 10
        fill = "#cc3333" if s.moving else "#336699"
        self.canvas.create_oval(rbx - r, rby - r, rbx + r, rby + r,
                                 fill=fill, outline="#222222", width=2)

        # ── heading arrow ──
        hx = rbx + 22 * math.cos(s.theta)
        hy = rby - 22 * math.sin(s.theta)
        self.canvas.create_line(rbx, rby, hx, hy,
                                 fill="#222222", width=2, arrow=tk.LAST)

        # ── online indicator dot (top-right corner) ──
        alive = s.last_seen is not None and (time.time() - s.last_seen) < STALE_S
        dot_color = "#33aa33" if alive else "#cc3333"
        self.canvas.create_oval(self.cw - 22, 8, self.cw - 10, 20,
                                 fill=dot_color, outline="")
        self.canvas.create_text(self.cw - 25, 14,
                                 text="●" if alive else "○",
                                 anchor="e", fill=dot_color,
                                 font=("TkDefaultFont", 9))

        # ── legend (bottom-left) ──
        self._draw_legend()

    def _draw_legend(self) -> None:
        lx, ly = 8, self.ch - 10
        items = [
            ("■ rock sample", "#555555"),
            ("× cliff",       EVENT_COLORS["cliff"]),
            ("× boundary",    EVENT_COLORS["boundary"]),
            ("× obstacle",    EVENT_COLORS["obstacle"]),
        ]
        for text, color in reversed(items):
            self.canvas.create_text(lx, ly, text=text, anchor="sw",
                                    fill=color, font=("TkFixedFont", 8))
            ly -= 13


if __name__ == "__main__":
    app = App()
    app.run()
