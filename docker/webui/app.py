#!/usr/bin/env python3
import asyncio
import json
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse, JSONResponse
import uvicorn

from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__("telemetry_web_bridge")
        self._lock = threading.Lock()
        self.state = {
            "rpy_deg": {"r": 0.0, "p": 0.0, "y": 0.0},
            "acc_ms2": {"x": 0.0, "y": 0.0, "z": 0.0},
            "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            "nav_state": "",
        }
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_pub = self.create_publisher(String, "/nav_control", 10)

        self.create_subscription(Vector3Stamped, "/imu/rpy_deg", self.on_rpy, 20)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 20)
        self.create_subscription(String, "/nav_state", self.on_nav_state, 10)

    def on_rpy(self, msg: Vector3Stamped):
        with self._lock:
            self.state["rpy_deg"] = {
                "r": float(msg.vector.x),
                "p": float(msg.vector.y),
                "y": float(msg.vector.z),
            }

    def on_imu(self, msg: Imu):
        with self._lock:
            self.state["acc_ms2"] = {
                "x": float(msg.linear_acceleration.x),
                "y": float(msg.linear_acceleration.y),
                "z": float(msg.linear_acceleration.z),
            }
            self.state["gyro_dps"] = {
                "x": float(msg.angular_velocity.x * 57.2957795),
                "y": float(msg.angular_velocity.y * 57.2957795),
                "z": float(msg.angular_velocity.z * 57.2957795),
            }

    def on_nav_state(self, msg: String):
        with self._lock:
            self.state["nav_state"] = msg.data

    def snapshot(self):
        with self._lock:
            return json.loads(json.dumps(self.state))

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

    def send_nav_cmd(self, cmd: str):
        s = String()
        s.data = cmd
        self.nav_pub.publish(s)


app = FastAPI(title="PT Rover Web UI")
bridge = None

@app.get("/", response_class=HTMLResponse)
async def index():
    html = Path("/workspace/docker/webui/index.html").read_text(encoding="utf-8")
    return HTMLResponse(content=html)


@app.get("/api/state")
async def api_state():
    return JSONResponse(content=bridge.snapshot())


@app.post("/api/cmd_vel")
async def api_cmd_vel(payload: dict):
    linear = float(payload.get("linear_x", 0.0))
    angular = float(payload.get("angular_z", 0.0))
    bridge.send_cmd_vel(linear, angular)
    return {"ok": True}


@app.post("/api/nav_control")
async def api_nav(payload: dict):
    cmd = str(payload.get("cmd", "")).strip()
    if cmd:
        bridge.send_nav_cmd(cmd)
    return {"ok": True, "cmd": cmd}


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()

    async def send_snapshots():
        while True:
            await ws.send_text(json.dumps(bridge.snapshot()))
            await asyncio.sleep(0.05)

    async def recv_cmds():
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue
            t = data.get("type")
            if t == "cmd_vel":
                bridge.send_cmd_vel(
                    float(data.get("linear_x", 0.0)),
                    float(data.get("angular_z", 0.0)),
                )
            elif t == "nav_control":
                cmd = str(data.get("cmd", "")).strip()
                if cmd:
                    bridge.send_nav_cmd(cmd)

    try:
        await asyncio.gather(send_snapshots(), recv_cmds())
    except Exception:
        return


def _spin_ros():
    global bridge
    rclpy.init()
    bridge = TelemetryBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    t = threading.Thread(target=_spin_ros, daemon=True)
    t.start()
    # attendre init bridge
    while bridge is None:
        time.sleep(0.01)
    uvicorn.run(app, host="0.0.0.0", port=8080)

