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
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__("telemetry_web_bridge")
        self._lock = threading.Lock()
        self.state = {
            "rpy_deg": {"r": 0.0, "p": 0.0, "y": 0.0},
            "acc_ms2": {"x": 0.0, "y": 0.0, "z": 0.0},
            "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            "scan": [],
            "nav_state": "",
            "slam_map_points": [],
            "slam_graph_points": [],
            "slam_graph_edges": [],
        }
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_pub = self.create_publisher(String, "/nav_control", 10)

        self.create_subscription(Vector3Stamped, "/imu/rpy_deg", self.on_rpy, 20)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 20)
        self.create_subscription(LaserScan, "/lidar_scan", self.on_scan, 5)
        self.create_subscription(String, "/nav_state", self.on_nav_state, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 5)
        self.create_subscription(MarkerArray, "/slam_toolbox/graph_visualization", self.on_graph, 5)

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

    def on_scan(self, msg: LaserScan):
        pts = []
        for i, r in enumerate(msg.ranges):
            if r > msg.range_min and r < msg.range_max:
                ang_deg = int(i)
                dist_mm = int(r * 1000.0)
                pts.append([ang_deg, dist_mm])
        with self._lock:
            self.state["scan"] = pts[:1200]  # limite payload

    def on_nav_state(self, msg: String):
        with self._lock:
            self.state["nav_state"] = msg.data

    def on_map(self, msg: OccupancyGrid):
        # Convertit /map en nuage de points 2D monde (léger pour UI web).
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        data = msg.data
        pts = []

        # Décimation simple pour limiter la charge navigateur.
        stride = 3 if (w * h) > 250000 else 2
        for y in range(0, h, stride):
            row = y * w
            for x in range(0, w, stride):
                v = data[row + x]
                if v > 50:
                    wx = ox + (x + 0.5) * res
                    wy = oy + (y + 0.5) * res
                    pts.append([wx, wy])
                    if len(pts) >= 5000:
                        break
            if len(pts) >= 5000:
                break

        with self._lock:
            self.state["slam_map_points"] = pts

    def on_graph(self, msg: MarkerArray):
        gpts = []
        gedges = []
        for marker in msg.markers:
            # points de graphe
            for p in marker.points:
                gpts.append([float(p.x), float(p.y)])
                if len(gpts) >= 2000:
                    break
            # edges si marker en LINE_LIST (pairs de points)
            if marker.type == marker.LINE_LIST:
                pts = marker.points
                n = len(pts) - (len(pts) % 2)
                for i in range(0, n, 2):
                    p0 = pts[i]
                    p1 = pts[i + 1]
                    gedges.append([float(p0.x), float(p0.y), float(p1.x), float(p1.y)])
                    if len(gedges) >= 2000:
                        break
            if len(gpts) >= 2000 and len(gedges) >= 2000:
                break

        with self._lock:
            self.state["slam_graph_points"] = gpts
            self.state["slam_graph_edges"] = gedges

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
            await asyncio.sleep(0.2)

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

