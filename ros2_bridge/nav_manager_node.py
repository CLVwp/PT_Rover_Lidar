#!/usr/bin/env python3
"""
Nav manager simple pour Foxglove:
- Recoit des points cliques (/clicked_point, /move_base_simple/goal)
- Recoit des commandes texte (/nav_control, std_msgs/String)
- Publie /cmd_vel
- Publie l'etat sur /nav_state (std_msgs/String)

Commandes /nav_control:
- record_on / record_off
- goto_start / goto_last
- start_follow      (joue les waypoints une fois)
- start_patrol      (boucle sur les waypoints)
- naive_on          (avance + tourne en continu)
- stop              (arret commande)
- clear             (supprime waypoints)

Quand record_on est actif, le node enregistre aussi la pose reelle du rover
pendant le pilotage manuel afin de pouvoir rejouer un tour "naif".
"""

import math
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from slam_toolbox.srv import Clear, Reset


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_vec(px: float, py: float, pz: float, q):
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)
    cx = qy * tz - qz * ty
    cy = qz * tx - qx * tz
    cz = qx * ty - qy * tx
    return px + qw * tx + cx, py + qw * ty + cy, pz + qw * tz + cz


@dataclass
class Waypoint:
    x: float
    y: float


class NavManager(Node):
    def __init__(self):
        super().__init__("nav_manager")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_period_s", 0.1)
        self.declare_parameter("distance_tolerance_m", 0.25)
        self.declare_parameter("max_linear_cmd", 0.55)
        self.declare_parameter("max_angular_cmd", 1.0)
        self.declare_parameter("linear_k", 0.8)
        self.declare_parameter("angular_k", 1.4)
        self.declare_parameter("pivot_angle_threshold_rad", 1.0)
        self.declare_parameter("naive_linear", 0.20)
        self.declare_parameter("naive_angular", 0.35)
        self.declare_parameter("slam_point_min_spacing_m", 0.35)
        self.declare_parameter("record_spacing_m", 0.20)
        self.declare_parameter("contact_front_warn_m", 0.28)
        self.declare_parameter("contact_front_danger_m", 0.16)
        self.declare_parameter("contact_side_warn_m", 0.22)
        self.declare_parameter("contact_side_danger_m", 0.12)
        self.declare_parameter("contact_front_angle_deg", 70.0)
        self.declare_parameter("contact_side_angle_deg", 90.0)

        self.target_frame = self.get_parameter("target_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_click = self.create_subscription(PointStamped, "/clicked_point", self.on_clicked, qos)
        self.sub_goal = self.create_subscription(PoseStamped, "/move_base_simple/goal", self.on_goal, qos)
        self.sub_ctrl = self.create_subscription(String, "/nav_control", self.on_control, 10)
        self.sub_scan = self.create_subscription(LaserScan, "/lidar_scan", self.on_scan, 10)
        self.sub_slam_graph = self.create_subscription(
            MarkerArray, "/slam_toolbox/graph_visualization", self.on_slam_graph, 10
        )

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/nav_state", 10)
        self.pub_contact_markers = self.create_publisher(MarkerArray, "/nav/contact_zones", 10)
        self.pub_contact_state = self.create_publisher(String, "/nav/contact_state", 10)
        self.cli_slam_clear = self.create_client(Clear, "/slam_toolbox/clear_changes")
        self.cli_slam_reset = self.create_client(Reset, "/slam_toolbox/reset")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._lock = threading.Lock()
        self.record_on = False
        self.record_start_pending = False
        self.mode = "idle"  # idle|goto|follow|patrol|naive
        self.waypoints = []
        self.slam_waypoints = []
        self.wp_idx = 0
        self.active_list = "manual"  # manual|slam
        self.goal = None  # Waypoint in target frame
        self.in_tol_count = 0
        self._scan_frame = "lidar"
        self._contact_summary = {
            "front": float("inf"),
            "left": float("inf"),
            "right": float("inf"),
        }

        self.timer = self.create_timer(float(self.get_parameter("cmd_period_s").value), self.on_timer)
        self.state_timer = self.create_timer(0.5, self.publish_state)
        self.get_logger().info(f"NavManager actif. frame={self.target_frame}, commandes via /nav_control")

    def _call_slam_service(self, which: str):
        if which == "clear":
            client = self.cli_slam_clear
            req = Clear.Request()
            name = "/slam_toolbox/clear_changes"
        else:
            client = self.cli_slam_reset
            req = Reset.Request()
            name = "/slam_toolbox/reset"

        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.8):
                self.get_logger().warn(f"Service indisponible: {name}")
                return
        future = client.call_async(req)

        def _done_cb(fut):
            try:
                _ = fut.result()
                self.get_logger().info(f"Service OK: {name}")
            except Exception as e:
                self.get_logger().warn(f"Echec appel {name}: {e}")

        future.add_done_callback(_done_cb)

    def publish_state(self):
        with self._lock:
            txt = (
                f"mode={self.mode} record_on={str(self.record_on).lower()} "
                f"wp_count={len(self.waypoints)} slam_wp_count={len(self.slam_waypoints)} wp_idx={self.wp_idx} "
                f"front_min={self._contact_summary['front']:.2f} "
                f"left_min={self._contact_summary['left']:.2f} "
                f"right_min={self._contact_summary['right']:.2f}"
            )
        m = String()
        m.data = txt
        self.pub_state.publish(m)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return wrap_pi(a - b)

    def _sector_min(self, scan: LaserScan, center_rad: float, width_rad: float) -> float:
        best = float("inf")
        angle = float(scan.angle_min)
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                if abs(self._angle_diff(angle, center_rad)) <= width_rad * 0.5:
                    best = min(best, float(r))
            angle += float(scan.angle_increment)
        return best

    @staticmethod
    def _contact_level(distance: float, warn_m: float, danger_m: float) -> str:
        if not math.isfinite(distance):
            return "none"
        if distance <= danger_m:
            return "danger"
        if distance <= warn_m:
            return "warn"
        return "safe"

    @staticmethod
    def _contact_color(level: str):
        if level == "danger":
            return (0.95, 0.15, 0.15, 0.90)
        if level == "warn":
            return (0.98, 0.62, 0.10, 0.88)
        if level == "safe":
            return (0.15, 0.85, 0.25, 0.72)
        return (0.60, 0.60, 0.60, 0.45)

    @staticmethod
    def _format_contact_distance(distance: float) -> str:
        if not math.isfinite(distance):
            return "--"
        return f"{distance:.2f}m"

    def _dominant_contact(self, front_min: float, left_min: float, right_min: float):
        front_warn = float(self.get_parameter("contact_front_warn_m").value)
        front_danger = float(self.get_parameter("contact_front_danger_m").value)
        side_warn = float(self.get_parameter("contact_side_warn_m").value)
        side_danger = float(self.get_parameter("contact_side_danger_m").value)
        candidates = [
            ("front", front_min, self._contact_level(front_min, front_warn, front_danger)),
            ("left", left_min, self._contact_level(left_min, side_warn, side_danger)),
            ("right", right_min, self._contact_level(right_min, side_warn, side_danger)),
        ]
        priority = {"danger": 3, "warn": 2, "safe": 1, "none": 0}
        candidates.sort(key=lambda item: (-priority[item[2]], item[1]))
        best_side, best_distance, best_level = candidates[0]
        if best_level in ("safe", "none"):
            return "none", "safe", best_distance
        return best_side, best_level, best_distance

    def _make_sector_marker(self, marker_id: int, center_rad: float, width_rad: float, radius: float, color, scale_x: float = 0.025):
        marker = Marker()
        marker.header.frame_id = self._scan_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nav_contact_zone"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = scale_x
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.orientation.w = 1.0
        start = center_rad - width_rad * 0.5
        end = center_rad + width_rad * 0.5
        pts = [Point(x=0.0, y=0.0, z=0.0)]
        steps = 12
        for i in range(steps + 1):
            a = start + (end - start) * (i / steps)
            pts.append(Point(x=radius * math.cos(a), y=radius * math.sin(a), z=0.0))
        pts.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points = pts
        return marker

    def _make_text_marker(self, marker_id: int, x: float, y: float, text: str, color):
        marker = Marker()
        marker.header.frame_id = self._scan_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nav_contact_text"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.08
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        return marker

    def _publish_contact_markers(self):
        front_warn = float(self.get_parameter("contact_front_warn_m").value)
        front_danger = float(self.get_parameter("contact_front_danger_m").value)
        side_warn = float(self.get_parameter("contact_side_warn_m").value)
        side_danger = float(self.get_parameter("contact_side_danger_m").value)
        front_width = math.radians(float(self.get_parameter("contact_front_angle_deg").value))
        side_width = math.radians(float(self.get_parameter("contact_side_angle_deg").value))
        with self._lock:
            front_min = self._contact_summary["front"]
            left_min = self._contact_summary["left"]
            right_min = self._contact_summary["right"]
        front_level = self._contact_level(front_min, front_warn, front_danger)
        left_level = self._contact_level(left_min, side_warn, side_danger)
        right_level = self._contact_level(right_min, side_warn, side_danger)
        dominant_side, dominant_level, dominant_distance = self._dominant_contact(front_min, left_min, right_min)
        front_color = self._contact_color(front_level)
        left_color = self._contact_color(left_level)
        right_color = self._contact_color(right_level)
        danger_outline = (0.95, 0.15, 0.15, 0.30)
        markers = MarkerArray()
        markers.markers.append(self._make_sector_marker(0, 0.0, front_width, front_warn, front_color, 0.03))
        markers.markers.append(self._make_sector_marker(1, math.pi * 0.5, side_width, side_warn, left_color, 0.03))
        markers.markers.append(self._make_sector_marker(2, -math.pi * 0.5, side_width, side_warn, right_color, 0.03))
        markers.markers.append(self._make_sector_marker(3, 0.0, front_width, front_danger, danger_outline, 0.018))
        markers.markers.append(self._make_sector_marker(4, math.pi * 0.5, side_width, side_danger, danger_outline, 0.018))
        markers.markers.append(self._make_sector_marker(5, -math.pi * 0.5, side_width, side_danger, danger_outline, 0.018))
        markers.markers.append(
            self._make_text_marker(
                10,
                front_warn + 0.07,
                0.0,
                f"front {front_level} {self._format_contact_distance(front_min)}",
                front_color,
            )
        )
        markers.markers.append(
            self._make_text_marker(
                11,
                0.0,
                side_warn + 0.10,
                f"left {left_level} {self._format_contact_distance(left_min)}",
                left_color,
            )
        )
        markers.markers.append(
            self._make_text_marker(
                12,
                0.0,
                -(side_warn + 0.10),
                f"right {right_level} {self._format_contact_distance(right_min)}",
                right_color,
            )
        )
        dominant_color = self._contact_color(dominant_level)
        dominant_label = (
            f"active {dominant_side} {dominant_level} {self._format_contact_distance(dominant_distance)}"
            if dominant_side != "none"
            else "active none"
        )
        markers.markers.append(self._make_text_marker(20, 0.0, 0.0, dominant_label, dominant_color))
        self.pub_contact_markers.publish(markers)
        state_msg = String()
        state_msg.data = (
            f"active={dominant_side} level={dominant_level} "
            f"front={self._format_contact_distance(front_min)} "
            f"left={self._format_contact_distance(left_min)} "
            f"right={self._format_contact_distance(right_min)}"
        )
        self.pub_contact_state.publish(state_msg)

    def on_scan(self, msg: LaserScan):
        front_width = math.radians(float(self.get_parameter("contact_front_angle_deg").value))
        side_width = math.radians(float(self.get_parameter("contact_side_angle_deg").value))
        front_min = self._sector_min(msg, 0.0, front_width)
        left_min = self._sector_min(msg, math.pi * 0.5, side_width)
        right_min = self._sector_min(msg, -math.pi * 0.5, side_width)
        with self._lock:
            self._scan_frame = msg.header.frame_id or "lidar"
            self._contact_summary["front"] = front_min
            self._contact_summary["left"] = left_min
            self._contact_summary["right"] = right_min
        self._publish_contact_markers()

    def _record_waypoint_locked(self, x: float, y: float, force: bool = False):
        spacing = float(self.get_parameter("record_spacing_m").value)
        if force or not self.waypoints:
            self.waypoints.append(Waypoint(x, y))
            return
        last = self.waypoints[-1]
        if math.hypot(x - last.x, y - last.y) >= spacing:
            self.waypoints.append(Waypoint(x, y))

    def on_slam_graph(self, msg: MarkerArray):
        pts = []
        min_spacing = float(self.get_parameter("slam_point_min_spacing_m").value)

        def append_if_far(x: float, y: float):
            for p in pts:
                if math.hypot(x - p.x, y - p.y) < min_spacing:
                    return
            pts.append(Waypoint(x, y))

        for marker in msg.markers:
            frame = marker.header.frame_id or self.target_frame
            for p in marker.points:
                t = self.to_target(float(p.x), float(p.y), float(p.z), frame)
                if t is None:
                    continue
                append_if_far(t[0], t[1])

        if pts:
            with self._lock:
                self.slam_waypoints = pts

    def to_target(self, x: float, y: float, z: float, from_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, from_frame, rclpy.time.Time())
        except Exception:
            return None
        rx, ry, rz = rotate_vec(x, y, z, tf.transform.rotation)
        rx += tf.transform.translation.x
        ry += tf.transform.translation.y
        rz += tf.transform.translation.z
        return rx, ry, rz

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y, quat_to_yaw(tf.transform.rotation)

    def _set_goal_from_point(self, x: float, y: float, z: float, frame_id: str):
        pt = self.to_target(x, y, z, frame_id or self.target_frame)
        if pt is None:
            self.get_logger().warn("Goal ignore: TF indisponible")
            return
        gx, gy, _ = pt
        with self._lock:
            if self.record_on:
                self.waypoints.append(Waypoint(gx, gy))
            self.goal = Waypoint(gx, gy)
            self.mode = "goto"
            self.in_tol_count = 0
        self.get_logger().info(f"Goal fige en {self.target_frame}: x={gx:.2f} y={gy:.2f}")

    def on_clicked(self, msg: PointStamped):
        self._set_goal_from_point(msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id)

    def on_goal(self, msg: PoseStamped):
        self._set_goal_from_point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.frame_id)

    def on_control(self, msg: String):
        cmd = msg.data.strip().lower()
        with self._lock:
            if cmd == "record_on":
                self.record_on = True
                self.record_start_pending = True
            elif cmd == "record_off":
                self.record_on = False
            elif cmd == "clear":
                self.waypoints.clear()
                self.wp_idx = 0
                self.active_list = "manual"
                self.goal = None
                self.mode = "idle"
                self.record_start_pending = False
            elif cmd == "clear_slam":
                self.slam_waypoints.clear()
            elif cmd == "slam_clear":
                self._call_slam_service("clear")
            elif cmd == "slam_reset":
                self._call_slam_service("reset")
            elif cmd == "goto_start":
                if self.waypoints:
                    self.goal = self.waypoints[0]
                    self.mode = "goto"
                    self.in_tol_count = 0
            elif cmd == "goto_last":
                if self.waypoints:
                    self.goal = self.waypoints[-1]
                    self.mode = "goto"
                    self.in_tol_count = 0
            elif cmd == "start_follow":
                if self.waypoints:
                    self.wp_idx = 0
                    self.active_list = "manual"
                    self.goal = self.waypoints[0]
                    self.mode = "follow"
                    self.in_tol_count = 0
            elif cmd == "start_follow_slam":
                if self.slam_waypoints:
                    self.wp_idx = 0
                    self.active_list = "slam"
                    self.goal = self.slam_waypoints[0]
                    self.mode = "follow"
                    self.in_tol_count = 0
            elif cmd == "start_patrol":
                if self.waypoints:
                    self.wp_idx = 0
                    self.active_list = "manual"
                    self.goal = self.waypoints[0]
                    self.mode = "patrol"
                    self.in_tol_count = 0
            elif cmd == "start_patrol_slam":
                if self.slam_waypoints:
                    self.wp_idx = 0
                    self.active_list = "slam"
                    self.goal = self.slam_waypoints[0]
                    self.mode = "patrol"
                    self.in_tol_count = 0
            elif cmd == "naive_on":
                self.mode = "naive"
            elif cmd == "naive_off":
                self.mode = "idle"
            elif cmd == "stop":
                self.mode = "idle"
            else:
                self.get_logger().warn(f"Commande inconnue: {cmd}")
        self.get_logger().info(f"/nav_control => {cmd}")

    def _publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.pub_cmd.publish(t)

    def on_timer(self):
        with self._lock:
            mode = self.mode
            goal = self.goal
            record_on = self.record_on
            record_start_pending = self.record_start_pending

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, yaw = pose

        if record_on:
            with self._lock:
                if self.record_on:
                    self._record_waypoint_locked(rx, ry, force=self.record_start_pending)
                    self.record_start_pending = False

        if mode == "idle":
            return

        if mode == "naive":
            t = Twist()
            t.linear.x = float(self.get_parameter("naive_linear").value)
            t.angular.z = float(self.get_parameter("naive_angular").value)
            self.pub_cmd.publish(t)
            return

        if goal is None:
            return

        dx = goal.x - rx
        dy = goal.y - ry
        dist = math.hypot(dx, dy)
        ang = wrap_pi(math.atan2(dy, dx) - yaw)

        tol = float(self.get_parameter("distance_tolerance_m").value)
        if dist <= tol:
            self.in_tol_count += 1
        else:
            self.in_tol_count = 0

        if self.in_tol_count >= 3:
            # point atteint
            self._publish_stop()
            with self._lock:
                self.in_tol_count = 0
                if self.mode == "goto":
                    self.mode = "idle"
                    self.goal = None
                elif self.mode == "follow":
                    self.wp_idx += 1
                    cur = self.waypoints if self.active_list == "manual" else self.slam_waypoints
                    if self.wp_idx >= len(cur):
                        self.mode = "idle"
                        self.goal = None
                    else:
                        self.goal = cur[self.wp_idx]
                elif self.mode == "patrol":
                    cur = self.waypoints if self.active_list == "manual" else self.slam_waypoints
                    if len(cur) > 0:
                        self.wp_idx = (self.wp_idx + 1) % len(cur)
                        self.goal = cur[self.wp_idx]
            return

        # controlleur P
        lin = float(self.get_parameter("linear_k").value) * dist
        ang_cmd = float(self.get_parameter("angular_k").value) * ang
        if dist < 0.8:
            lin *= 0.6
        if abs(ang) > float(self.get_parameter("pivot_angle_threshold_rad").value):
            lin = 0.0

        lin = clamp(lin, -float(self.get_parameter("max_linear_cmd").value), float(self.get_parameter("max_linear_cmd").value))
        ang_cmd = clamp(ang_cmd, -float(self.get_parameter("max_angular_cmd").value), float(self.get_parameter("max_angular_cmd").value))

        t = Twist()
        t.linear.x = lin
        t.angular.z = ang_cmd
        self.pub_cmd.publish(t)


def main():
    rclpy.init()
    node = NavManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

