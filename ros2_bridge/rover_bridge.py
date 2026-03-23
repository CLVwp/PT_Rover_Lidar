#!/usr/bin/env python3
"""
Bridge Rover -> ROS2.
LiDAR + IMU via WebSocket unique (trame synchronisée).
Publie: /lidar_scan, /imu/data, /imu/rpy_deg.
S'abonne à /cmd_vel (Twist) pour téléop : envoie les vitesses L/R au rover via WS.
"""
import asyncio
import json
import math
import os
import socket
import threading
import urllib.parse
import urllib.request
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Vector3Stamped, TransformStamped, Twist
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import StaticTransformBroadcaster
import websockets

try:
    from rover_cmd import stop as rover_stop
except ImportError:
    import sys as _sys
    _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from rover_cmd import stop as rover_stop

# CONFIG — IP du rover (env ROS_ROVER_IP ou défaut)
ROVER_IP = os.environ.get("ROS_ROVER_IP", "10.160.237.225")


def _resolve_ipv4(host: str) -> str:
    """Force la résolution en IPv4 pour éviter « Network is unreachable » (tentative IPv6)."""
    try:
        for res in socket.getaddrinfo(host, None, socket.AF_INET):
            return res[4][0]
    except Exception:
        pass
    return host


ROVER_IP_V4 = _resolve_ipv4(ROVER_IP)
LIDAR_WS_URL = f"ws://{ROVER_IP_V4}:81"
ROVER_HTTP_PORT = int(os.environ.get("ROVER_HTTP_PORT", "80"))
DEG_TO_RAD = math.pi / 180.0
# Même T que json_cmd.h / rover_cmd (vitesses L/R dans [-2, 2])
CMD_SPEED_T = 1


def rpy_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Roll, pitch, yaw (degrés) -> quaternion (x, y, z, w)."""
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class RoverBridge(Node):
    def __init__(self):
        super().__init__("rover_bridge")
        self.declare_parameter("lidar_offset_x_m", -0.02)
        self.declare_parameter("lidar_offset_y_m", 0.0)
        self.declare_parameter("lidar_offset_z_m", 0.0)

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.lidar_pub = self.create_publisher(LaserScan, "/lidar_scan", qos_lidar)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, "/imu/rpy_deg", 10)
        # TF statique base_link -> lidar (pour que le 3D Foxglove puisse afficher /lidar_scan)
        self.tf_static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "lidar"
        t.transform.translation.x = float(self.get_parameter("lidar_offset_x_m").value)
        t.transform.translation.y = float(self.get_parameter("lidar_offset_y_m").value)
        t.transform.translation.z = float(self.get_parameter("lidar_offset_z_m").value)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static.sendTransform(t)

        # Téléop depuis Foxglove (ou autre) : /cmd_vel (Twist) -> vitesses L/R rover
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_cb, 10
        )
        self._cmd_vel_turn_gain = 0.5  # angular.z (rad/s) -> écart L/R
        # Timeout : si plus de message pendant ce temps, on envoie stop.
        # Défaut 0,65 s : au-dessous, le jitter HTTP/DDS peut couper le flux en plein maintien bouton.
        self._cmd_vel_timeout_s = float(os.environ.get("CMD_VEL_TIMEOUT_S", "0.65"))
        self._last_cmd_vel_time_ns = None
        self._cmd_vel_stopped_sent = True  # au démarrage on n'envoie pas stop
        self._cmd_vel_timer = self.create_timer(0.05, self._cmd_vel_timeout_cb)
        self._motor_http_lock = threading.Lock()
        self._motor_http_latest = None
        self._motor_http_last_sent_payload = None
        self._motor_http_last_sent_ns = 0
        self._motor_http_refresh_s = 0.25
        self._motor_http_event = threading.Event()
        self._motor_http_thread = threading.Thread(target=self._motor_http_worker, daemon=True)
        self._motor_http_thread.start()

        self.get_logger().info(
            "RoverBridge initialisé (WS LiDAR+IMU, commandes moteur HTTP /js)"
        )

    def _queue_motor_lr(self, L: float, R: float, force: bool = False) -> None:
        L = max(-2.0, min(2.0, float(L)))
        R = max(-2.0, min(2.0, float(R)))
        payload = {"T": CMD_SPEED_T, "L": L, "R": R}
        now_ns = self.get_clock().now().nanoseconds
        with self._motor_http_lock:
            same_payload = self._motor_http_last_sent_payload == payload
            refresh_due = (now_ns - self._motor_http_last_sent_ns) >= int(self._motor_http_refresh_s * 1e9)
            if (not force) and same_payload and (not refresh_due):
                return
            self._motor_http_latest = payload
        self._motor_http_event.set()

    def _motor_http_url(self, payload: dict) -> str:
        raw = json.dumps(payload, separators=(",", ":"))
        query = urllib.parse.quote(raw, safe="")
        if ROVER_HTTP_PORT == 80:
            return f"http://{ROVER_IP_V4}/js?json={query}"
        return f"http://{ROVER_IP_V4}:{ROVER_HTTP_PORT}/js?json={query}"

    def _send_motor_http(self, payload: dict) -> None:
        req = urllib.request.Request(self._motor_http_url(payload), method="GET")
        with urllib.request.urlopen(req, timeout=0.2) as resp:
            resp.read()

    def _motor_http_worker(self) -> None:
        while True:
            self._motor_http_event.wait()
            self._motor_http_event.clear()
            with self._motor_http_lock:
                payload = self._motor_http_latest
            if payload is None:
                continue
            try:
                self._send_motor_http(payload)
                with self._motor_http_lock:
                    self._motor_http_last_sent_payload = payload
                    self._motor_http_last_sent_ns = self.get_clock().now().nanoseconds
            except Exception as e:
                self.get_logger().warn(f"Envoi commande moteur HTTP: {type(e).__name__}: {e!r}")

    async def lidar_task(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connexion WebSocket LiDAR à {LIDAR_WS_URL}")
                async with websockets.connect(
                    LIDAR_WS_URL,
                    open_timeout=20,
                    ping_interval=None,
                    ping_timeout=None,
                    close_timeout=1,
                ) as ws:
                    self.get_logger().info("WebSocket LiDAR connecté")
                    async for msg in ws:
                        try:
                            data = json.loads(msg)
                        except Exception:
                            continue
                        raw = data.get("points", [])
                        if raw:
                            scan = self.build_laserscan(raw)
                            self.lidar_pub.publish(scan)
                        self.publish_imu_from_dict(data)
                        await asyncio.sleep(0)
            except Exception as e:
                self.get_logger().warn(f"Erreur WS LiDAR: {type(e).__name__}: {e!r}")
                await asyncio.sleep(3.0)

    def build_laserscan(self, raw_points):
        scan = LaserScan()
        now = self.get_clock().now().to_msg()
        scan.header = Header(stamp=now, frame_id="lidar")

        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.01
        scan.range_min = 0.02
        scan.range_max = 12.0

        ranges = [float("inf")] * 360
        for i in range(0, len(raw_points), 2):
            try:
                angle_deg = int(raw_points[i])
                dist_mm = int(raw_points[i + 1])
                if 0 <= angle_deg < 360 and dist_mm > 0:
                    dist_m = dist_mm / 1000.0
                    target_angle = (360 - angle_deg) % 360
                    ranges[target_angle] = dist_m
            except Exception:
                continue

        scan.ranges = ranges
        scan.intensities = [1.0] * len(ranges)
        return scan

    def publish_imu_from_dict(self, data: dict):
        # Firmware WS envoie r/p/y ou yaw + accel/gyro.
        if "r" not in data and "p" not in data and "y" not in data and "yaw" not in data:
            return
        r = float(data.get("r", 0.0))
        p = float(data.get("p", 0.0))
        y = float(data.get("y", data.get("yaw", 0.0)))
        ax_ms2 = float(data.get("ax_ms2", 0.0))
        ay_ms2 = float(data.get("ay_ms2", 0.0))
        az_ms2 = float(data.get("az_ms2", 0.0))
        gx_deg = float(data.get("gx", 0.0))
        gy_deg = float(data.get("gy", 0.0))
        gz_deg = float(data.get("gz", 0.0))

        now = self.get_clock().now().to_msg()
        rpy_msg = Vector3Stamped()
        rpy_msg.header = Header(stamp=now, frame_id="imu_link")
        rpy_msg.vector.x, rpy_msg.vector.y, rpy_msg.vector.z = r, p, y
        self.rpy_pub.publish(rpy_msg)

        imu_msg = Imu()
        imu_msg.header = Header(stamp=now, frame_id="imu_link")
        qx, qy, qz, qw = rpy_deg_to_quaternion(r, p, y)
        imu_msg.orientation.x, imu_msg.orientation.y = qx, qy
        imu_msg.orientation.z, imu_msg.orientation.w = qz, qw
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity.x = gx_deg * DEG_TO_RAD
        imu_msg.angular_velocity.y = gy_deg * DEG_TO_RAD
        imu_msg.angular_velocity.z = gz_deg * DEG_TO_RAD
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration.x = ax_ms2
        imu_msg.linear_acceleration.y = ay_ms2
        imu_msg.linear_acceleration.z = az_ms2
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

    def cmd_vel_cb(self, msg: Twist):
        """Convertit Twist (linear.x, angular.z) en vitesses différentielles L/R et envoie au rover."""
        self._last_cmd_vel_time_ns = self.get_clock().now().nanoseconds
        self._cmd_vel_stopped_sent = False
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        g = self._cmd_vel_turn_gain
        L = v - g * w
        R = v + g * w
        L = max(-1.0, min(1.0, L))
        R = max(-1.0, min(1.0, R))
        self._queue_motor_lr(L, R)

    def _cmd_vel_timeout_cb(self):
        """Si on n'a pas reçu de /cmd_vel depuis timeout, on envoie stop (relâche = arrêt immédiat)."""
        if self._last_cmd_vel_time_ns is None or self._cmd_vel_stopped_sent:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_cmd_vel_time_ns) > self._cmd_vel_timeout_s * 1e9:
            self._queue_motor_lr(0.0, 0.0, force=True)
            self._cmd_vel_stopped_sent = True


async def spin_rclpy(node: RoverBridge):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.02)
        await asyncio.sleep(0.001)


async def main_async():
    rclpy.init()
    node = RoverBridge()
    try:
        await asyncio.gather(spin_rclpy(node), node.lidar_task())
    finally:
        rover_stop()
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()

