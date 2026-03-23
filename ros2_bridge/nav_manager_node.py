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

import heapq
import math
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
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
        self.declare_parameter("robot_length_m", 0.20)
        self.declare_parameter("robot_width_m", 0.18)
        self.declare_parameter("robot_height_m", 0.08)
        self.declare_parameter("robot_safety_margin_m", 0.001)
        self.declare_parameter("lidar_visual_offset_x_m", -0.02)
        self.declare_parameter("lidar_visual_offset_y_m", 0.0)
        self.declare_parameter("lidar_visual_offset_z_m", 0.05)
        self.declare_parameter("map_occupied_threshold", 100)
        self.declare_parameter("map_unknown_is_blocked", False)
        self.declare_parameter("planned_path_spacing_m", 0.10)
        self.declare_parameter("path_lookahead_m", 0.28)
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
        self.sub_map = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.sub_scan = self.create_subscription(LaserScan, "/lidar_scan", self.on_scan, 10)
        self.sub_slam_graph = self.create_subscription(
            MarkerArray, "/slam_toolbox/graph_visualization", self.on_slam_graph, 10
        )

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/nav_state", 10)
        self.pub_contact_markers = self.create_publisher(MarkerArray, "/nav/contact_zones", 10)
        self.pub_contact_state = self.create_publisher(String, "/nav/contact_state", 10)
        self.pub_robot_markers = self.create_publisher(MarkerArray, "/nav/robot_markers", 10)
        self.pub_planned_path = self.create_publisher(Path, "/nav/planned_path", 10)
        self.pub_inflated_map = self.create_publisher(OccupancyGrid, "/nav/inflated_map", 10)
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
        self.planned_waypoints = []
        self.wp_idx = 0
        self.active_list = "manual"  # manual|slam|planned
        self.goal = None  # Waypoint in target frame
        self.in_tol_count = 0
        self._scan_frame = "lidar"
        self._contact_summary = {
            "front": float("inf"),
            "left": float("inf"),
            "right": float("inf"),
        }
        self._map_msg = None
        self._grid_width = 0
        self._grid_height = 0
        self._grid_resolution = 0.0
        self._grid_origin_x = 0.0
        self._grid_origin_y = 0.0
        self._occupancy_grid = []
        self._inflated_grid = []

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
                f"active_list={self.active_list} "
                f"wp_count={len(self.waypoints)} slam_wp_count={len(self.slam_waypoints)} wp_idx={self.wp_idx} "
                f"planned_wp_count={len(self.planned_waypoints)} "
                f"front_min={self._contact_summary['front']:.2f} "
                f"left_min={self._contact_summary['left']:.2f} "
                f"right_min={self._contact_summary['right']:.2f}"
            )
        m = String()
        m.data = txt
        self.pub_state.publish(m)

    def _robot_planner_radius_m(self) -> float:
        half_length = 0.5 * float(self.get_parameter("robot_length_m").value)
        half_width = 0.5 * float(self.get_parameter("robot_width_m").value)
        safety_margin = float(self.get_parameter("robot_safety_margin_m").value)
        return math.hypot(half_length + safety_margin, half_width + safety_margin)

    def _planner_spacing_m(self) -> float:
        return float(self.get_parameter("planned_path_spacing_m").value)

    def _path_lookahead_m(self) -> float:
        return float(self.get_parameter("path_lookahead_m").value)

    def _map_index(self, gx: int, gy: int) -> int:
        return gy * self._grid_width + gx

    def _world_to_grid(self, x: float, y: float):
        if self._grid_resolution <= 0.0:
            return None
        gx = int(math.floor((x - self._grid_origin_x) / self._grid_resolution))
        gy = int(math.floor((y - self._grid_origin_y) / self._grid_resolution))
        if gx < 0 or gy < 0 or gx >= self._grid_width or gy >= self._grid_height:
            return None
        return gx, gy

    def _grid_to_world(self, gx: int, gy: int):
        x = self._grid_origin_x + (gx + 0.5) * self._grid_resolution
        y = self._grid_origin_y + (gy + 0.5) * self._grid_resolution
        return x, y

    def _downsample_waypoints(self, points):
        if not points:
            return []
        spacing = max(0.01, self._planner_spacing_m())
        out = [points[0]]
        last = points[0]
        for pt in points[1:-1]:
            if math.hypot(pt.x - last.x, pt.y - last.y) >= spacing:
                out.append(pt)
                last = pt
        if len(points) > 1:
            out.append(points[-1])
        dedup = []
        for pt in out:
            if not dedup or math.hypot(pt.x - dedup[-1].x, pt.y - dedup[-1].y) > 1e-6:
                dedup.append(pt)
        return dedup

    def _publish_planned_path_msg(self, points):
        msg = Path()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        poses = []
        for i, pt in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = pt.x
            pose.pose.position.y = pt.y
            pose.pose.position.z = 0.0
            if i + 1 < len(points):
                nx, ny = points[i + 1].x, points[i + 1].y
                yaw = math.atan2(ny - pt.y, nx - pt.x)
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
            else:
                pose.pose.orientation.w = 1.0
            poses.append(pose)
        msg.poses = poses
        self.pub_planned_path.publish(msg)

    def _publish_inflated_map_msg(self):
        if self._map_msg is None or not self._inflated_grid:
            return
        msg = OccupancyGrid()
        msg.header = self._map_msg.header
        msg.info = self._map_msg.info
        unknown_blocked = bool(self.get_parameter("map_unknown_is_blocked").value)
        data = []
        for idx, blocked in enumerate(self._inflated_grid):
            if blocked:
                data.append(100)
            elif unknown_blocked and self._occupancy_grid and self._occupancy_grid[idx] < 0:
                data.append(-1)
            else:
                data.append(0)
        msg.data = data
        self.pub_inflated_map.publish(msg)

    def _inflate_occupancy_grid(self):
        if not self._occupancy_grid or self._grid_width <= 0 or self._grid_height <= 0:
            self._inflated_grid = []
            return
        radius_cells = max(1, int(math.ceil(self._robot_planner_radius_m() / self._grid_resolution)))
        occupied_threshold = int(self.get_parameter("map_occupied_threshold").value)
        unknown_blocked = bool(self.get_parameter("map_unknown_is_blocked").value)
        inflated = [False] * len(self._occupancy_grid)
        occupied_cells = []
        for gy in range(self._grid_height):
            row_base = gy * self._grid_width
            for gx in range(self._grid_width):
                val = self._occupancy_grid[row_base + gx]
                if val >= occupied_threshold or (unknown_blocked and val < 0):
                    occupied_cells.append((gx, gy))
        for ox, oy in occupied_cells:
            for dy in range(-radius_cells, radius_cells + 1):
                ny = oy + dy
                if ny < 0 or ny >= self._grid_height:
                    continue
                for dx in range(-radius_cells, radius_cells + 1):
                    nx = ox + dx
                    if nx < 0 or nx >= self._grid_width:
                        continue
                    if dx * dx + dy * dy <= radius_cells * radius_cells:
                        inflated[self._map_index(nx, ny)] = True
        self._inflated_grid = inflated

    def on_map(self, msg: OccupancyGrid):
        self._map_msg = msg
        self._grid_width = int(msg.info.width)
        self._grid_height = int(msg.info.height)
        self._grid_resolution = float(msg.info.resolution)
        self._grid_origin_x = float(msg.info.origin.position.x)
        self._grid_origin_y = float(msg.info.origin.position.y)
        self._occupancy_grid = list(msg.data)
        self._inflate_occupancy_grid()
        self._publish_inflated_map_msg()

    def _is_grid_blocked(self, gx: int, gy: int) -> bool:
        if gx < 0 or gy < 0 or gx >= self._grid_width or gy >= self._grid_height:
            return True
        if not self._inflated_grid:
            return True
        return self._inflated_grid[self._map_index(gx, gy)]

    def _nearest_free_cell(self, start_cell):
        if start_cell is None:
            return None
        sx, sy = start_cell
        if not self._is_grid_blocked(sx, sy):
            return start_cell
        max_radius = max(self._grid_width, self._grid_height)
        for radius in range(1, max_radius):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    gx = sx + dx
                    gy = sy + dy
                    if gx < 0 or gy < 0 or gx >= self._grid_width or gy >= self._grid_height:
                        continue
                    if not self._is_grid_blocked(gx, gy):
                        return gx, gy
        return None

    def _reconstruct_path_cells(self, came_from, current):
        out = [current]
        while current in came_from:
            current = came_from[current]
            out.append(current)
        out.reverse()
        return out

    def _plan_grid_path(self, start_xy, goal_xy):
        if not self._inflated_grid:
            return []
        start_cell = self._nearest_free_cell(self._world_to_grid(start_xy[0], start_xy[1]))
        goal_cell = self._nearest_free_cell(self._world_to_grid(goal_xy[0], goal_xy[1]))
        if start_cell is None or goal_cell is None:
            return []
        open_heap = []
        heapq.heappush(open_heap, (0.0, start_cell))
        came_from = {}
        g_score = {start_cell: 0.0}
        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        closed = set()
        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal_cell:
                return self._reconstruct_path_cells(came_from, current)
            closed.add(current)
            for dx, dy, cost in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
                nxt = (nx, ny)
                if self._is_grid_blocked(nx, ny):
                    continue
                new_cost = g_score[current] + cost
                if new_cost < g_score.get(nxt, float("inf")):
                    came_from[nxt] = current
                    g_score[nxt] = new_cost
                    heapq.heappush(open_heap, (new_cost + heuristic(nxt, goal_cell), nxt))
        return []

    def _plan_waypoints(self, start_xy, goal_xy):
        cells = self._plan_grid_path(start_xy, goal_xy)
        if not cells:
            return []
        pts = [Waypoint(*self._grid_to_world(gx, gy)) for gx, gy in cells]
        return self._downsample_waypoints(pts)

    def _current_waypoint_list_locked(self):
        if self.active_list == "slam":
            return self.slam_waypoints
        if self.active_list == "planned":
            return self.planned_waypoints
        return self.waypoints

    def _contact_levels_locked(self):
        front = self._contact_summary["front"]
        left = self._contact_summary["left"]
        right = self._contact_summary["right"]
        front_warn = float(self.get_parameter("contact_front_warn_m").value)
        front_danger = float(self.get_parameter("contact_front_danger_m").value)
        side_warn = float(self.get_parameter("contact_side_warn_m").value)
        side_danger = float(self.get_parameter("contact_side_danger_m").value)
        return (
            self._contact_level(front, front_warn, front_danger),
            self._contact_level(left, side_warn, side_danger),
            self._contact_level(right, side_warn, side_danger),
        )

    def _planned_lookahead_goal_locked(self, rx: float, ry: float):
        if not self.planned_waypoints:
            return None, None
        start_idx = max(0, min(self.wp_idx, len(self.planned_waypoints) - 1))
        search_start = max(0, start_idx - 2)
        best_idx = start_idx
        best_dist = float("inf")
        for idx in range(search_start, len(self.planned_waypoints)):
            pt = self.planned_waypoints[idx]
            d = math.hypot(pt.x - rx, pt.y - ry)
            if d < best_dist:
                best_dist = d
                best_idx = idx
        self.wp_idx = best_idx
        lookahead_m = max(self._path_lookahead_m(), self._planner_spacing_m())
        target_idx = best_idx
        for idx in range(best_idx, len(self.planned_waypoints)):
            pt = self.planned_waypoints[idx]
            if math.hypot(pt.x - rx, pt.y - ry) >= lookahead_m:
                target_idx = idx
                break
            target_idx = idx
        return self.planned_waypoints[target_idx], self.planned_waypoints[-1]

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
        marker.header.frame_id = self.base_frame
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

    def _make_annular_sector_marker(
        self,
        marker_id: int,
        center_rad: float,
        width_rad: float,
        inner_radius: float,
        outer_radius: float,
        color,
        ns: str,
        scale_x: float = 0.02,
    ):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_x
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        start = center_rad - width_rad * 0.5
        end = center_rad + width_rad * 0.5
        steps = 18
        pts = []
        for i in range(steps + 1):
            a = start + (end - start) * (i / steps)
            pts.append(Point(x=inner_radius * math.cos(a), y=inner_radius * math.sin(a), z=0.01))
        pts.append(Point(x=outer_radius * math.cos(end), y=outer_radius * math.sin(end), z=0.01))
        for i in range(steps, -1, -1):
            a = start + (end - start) * (i / steps)
            pts.append(Point(x=outer_radius * math.cos(a), y=outer_radius * math.sin(a), z=0.01))
        pts.append(Point(x=inner_radius * math.cos(start), y=inner_radius * math.sin(start), z=0.01))
        marker.points = pts
        return marker

    def _make_text_marker(self, marker_id: int, x: float, y: float, text: str, color):
        marker = Marker()
        marker.header.frame_id = self.base_frame
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

    def _make_cube_marker(self, marker_id: int, frame_id: str, x: float, y: float, z: float, sx: float, sy: float, sz: float, color, ns: str):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    def _make_line_strip_marker(self, marker_id: int, frame_id: str, points, color, scale_x: float, ns: str):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_x
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = points
        return marker

    def _make_arrow_marker(self, marker_id: int, frame_id: str, start: Point, end: Point, color, ns: str):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.04
        marker.scale.z = 0.06
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = [start, end]
        return marker

    @staticmethod
    def _rectangle_outline(length_m: float, width_m: float, z: float):
        hx = 0.5 * length_m
        hy = 0.5 * width_m
        return [
            Point(x=-hx, y=-hy, z=z),
            Point(x=hx, y=-hy, z=z),
            Point(x=hx, y=hy, z=z),
            Point(x=-hx, y=hy, z=z),
            Point(x=-hx, y=-hy, z=z),
        ]

    @staticmethod
    def _zone_label_point(center_rad: float, radius: float, z: float = 0.05):
        return Point(x=radius * math.cos(center_rad), y=radius * math.sin(center_rad), z=z)

    def _publish_robot_markers(self):
        base_frame = self.base_frame
        robot_length = float(self.get_parameter("robot_length_m").value)
        robot_width = float(self.get_parameter("robot_width_m").value)
        robot_height = float(self.get_parameter("robot_height_m").value)
        safety_margin = float(self.get_parameter("robot_safety_margin_m").value)
        lidar_x = float(self.get_parameter("lidar_visual_offset_x_m").value)
        lidar_y = float(self.get_parameter("lidar_visual_offset_y_m").value)
        lidar_z = float(self.get_parameter("lidar_visual_offset_z_m").value)
        safety_length = robot_length + 2.0 * safety_margin
        safety_width = robot_width + 2.0 * safety_margin

        markers = MarkerArray()
        markers.markers.append(
            self._make_cube_marker(
                0,
                base_frame,
                0.0,
                0.0,
                0.5 * robot_height,
                robot_length,
                robot_width,
                robot_height,
                (0.12, 0.35, 0.88, 0.48),
                "nav_robot_body",
            )
        )
        markers.markers.append(
            self._make_line_strip_marker(
                1,
                base_frame,
                self._rectangle_outline(safety_length, safety_width, 0.01),
                (1.0, 0.82, 0.10, 1.0),
                0.024,
                "nav_robot_safety",
            )
        )
        markers.markers.append(
            self._make_text_marker(
                5,
                0.0,
                0.0,
                "BODY",
                (0.90, 0.95, 1.0, 0.9),
            )
        )
        markers.markers.append(
            self._make_text_marker(
                6,
                0.0,
                0.5 * safety_width + 0.05,
                "SAFETY",
                (1.0, 0.86, 0.2, 0.95),
            )
        )
        markers.markers.append(
            self._make_arrow_marker(
                2,
                base_frame,
                Point(x=0.0, y=0.0, z=robot_height + 0.01),
                Point(x=0.5 * robot_length + 0.06, y=0.0, z=robot_height + 0.01),
                (0.15, 0.95, 0.25, 0.95),
                "nav_robot_heading",
            )
        )
        markers.markers.append(
            self._make_cube_marker(
                3,
                base_frame,
                lidar_x,
                lidar_y,
                lidar_z,
                0.035,
                0.035,
                0.035,
                (0.95, 0.15, 0.95, 0.95),
                "nav_robot_lidar",
            )
        )
        markers.markers.append(
            self._make_line_strip_marker(
                4,
                base_frame,
                [
                    Point(x=lidar_x, y=lidar_y, z=0.0),
                    Point(x=lidar_x, y=lidar_y, z=lidar_z),
                ],
                (0.95, 0.15, 0.95, 0.55),
                0.01,
                "nav_robot_lidar",
            )
        )
        self.pub_robot_markers.publish(markers)

    def _publish_contact_markers(self):
        front_warn = float(self.get_parameter("contact_front_warn_m").value)
        front_danger = float(self.get_parameter("contact_front_danger_m").value)
        side_warn = float(self.get_parameter("contact_side_warn_m").value)
        side_danger = float(self.get_parameter("contact_side_danger_m").value)
        front_width = math.radians(float(self.get_parameter("contact_front_angle_deg").value))
        side_width = math.radians(float(self.get_parameter("contact_side_angle_deg").value))
        robot_length = float(self.get_parameter("robot_length_m").value)
        robot_width = float(self.get_parameter("robot_width_m").value)
        safety_margin = float(self.get_parameter("robot_safety_margin_m").value)
        inner_radius = 0.5 * max(robot_length + 2.0 * safety_margin, robot_width + 2.0 * safety_margin)
        front_stop_outer = inner_radius + front_danger
        front_slow_outer = inner_radius + front_warn
        side_stop_outer = inner_radius + side_danger
        side_slow_outer = inner_radius + side_warn
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
        slow_outline = (0.18, 0.85, 0.25, 0.78)
        stop_outline = (0.98, 0.20, 0.20, 0.92)
        front_label_x = front_slow_outer + 0.12
        front_label_y = 0.0
        stop_label_x = front_stop_outer - 0.02
        stop_label_y = 0.0
        slow_label_x = front_slow_outer - 0.02
        slow_label_y = 0.0
        side_label_x = -0.06
        left_label_y = side_slow_outer + 0.14
        right_label_y = -(side_slow_outer + 0.14)
        active_label_x = -(inner_radius + 0.12)
        active_label_y = 0.0
        markers = MarkerArray()
        markers.markers.append(
            self._make_annular_sector_marker(
                0, 0.0, front_width, inner_radius, front_slow_outer, slow_outline, "nav_contact_slow", 0.024
            )
        )
        markers.markers.append(
            self._make_annular_sector_marker(
                1, math.pi * 0.5, side_width, inner_radius, side_slow_outer, slow_outline, "nav_contact_slow", 0.024
            )
        )
        markers.markers.append(
            self._make_annular_sector_marker(
                2, -math.pi * 0.5, side_width, inner_radius, side_slow_outer, slow_outline, "nav_contact_slow", 0.024
            )
        )
        markers.markers.append(
            self._make_annular_sector_marker(
                3, 0.0, front_width, inner_radius, front_stop_outer, stop_outline, "nav_contact_stop", 0.026
            )
        )
        markers.markers.append(
            self._make_annular_sector_marker(
                4, math.pi * 0.5, side_width, inner_radius, side_stop_outer, stop_outline, "nav_contact_stop", 0.026
            )
        )
        markers.markers.append(
            self._make_annular_sector_marker(
                5, -math.pi * 0.5, side_width, inner_radius, side_stop_outer, stop_outline, "nav_contact_stop", 0.026
            )
        )
        markers.markers.append(
            self._make_text_marker(
                10,
                front_label_x,
                front_label_y,
                f"front {front_level} {self._format_contact_distance(front_min)}",
                front_color,
            )
        )
        markers.markers.append(
            self._make_text_marker(
                11,
                side_label_x,
                left_label_y,
                f"left {left_level} {self._format_contact_distance(left_min)}",
                left_color,
            )
        )
        markers.markers.append(
            self._make_text_marker(
                12,
                side_label_x,
                right_label_y,
                f"right {right_level} {self._format_contact_distance(right_min)}",
                right_color,
            )
        )
        markers.markers.append(
            self._make_text_marker(
                13,
                stop_label_x,
                stop_label_y,
                "STOP",
                stop_outline,
            )
        )
        markers.markers.append(
            self._make_text_marker(
                14,
                slow_label_x,
                slow_label_y,
                "SLOW",
                slow_outline,
            )
        )
        dominant_color = self._contact_color(dominant_level)
        dominant_label = (
            f"active {dominant_side} {dominant_level} {self._format_contact_distance(dominant_distance)}"
            if dominant_side != "none"
            else "active none"
        )
        markers.markers.append(self._make_text_marker(20, active_label_x, active_label_y, dominant_label, dominant_color))
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
        self._publish_robot_markers()
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
        pose = self.get_robot_pose()
        planned = []
        if pose is not None and self._inflated_grid:
            planned = self._plan_waypoints((pose[0], pose[1]), (gx, gy))
        with self._lock:
            if self.record_on:
                self.waypoints.append(Waypoint(gx, gy))
            if planned:
                self.planned_waypoints = planned
                self.active_list = "planned"
                self.wp_idx = 1 if len(planned) > 1 else 0
                self.goal = planned[self.wp_idx]
                self.mode = "follow"
            else:
                self.planned_waypoints = []
                self.active_list = "manual"
                self.wp_idx = 0
                self.goal = Waypoint(gx, gy)
                self.mode = "goto"
            self.in_tol_count = 0
        if planned:
            self._publish_planned_path_msg(planned)
            self.get_logger().info(
                f"Chemin planifie vers {self.target_frame}: x={gx:.2f} y={gy:.2f}, {len(planned)} points"
            )
        else:
            self._publish_planned_path_msg([])
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
                self.planned_waypoints.clear()
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
        if cmd == "clear":
            self._publish_planned_path_msg([])

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

        final_goal = goal
        with self._lock:
            if mode == "follow" and self.active_list == "planned":
                planned_goal, final_planned_goal = self._planned_lookahead_goal_locked(rx, ry)
                if planned_goal is not None:
                    self.goal = planned_goal
                    goal = planned_goal
                    final_goal = final_planned_goal

        if goal is None:
            return

        dx = goal.x - rx
        dy = goal.y - ry
        dist = math.hypot(dx, dy)
        ang = wrap_pi(math.atan2(dy, dx) - yaw)
        final_dist = math.hypot(final_goal.x - rx, final_goal.y - ry) if final_goal is not None else dist

        tol = float(self.get_parameter("distance_tolerance_m").value)
        reached_dist = final_dist if (mode == "follow" and self.active_list == "planned") else dist
        if reached_dist <= tol:
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
                    if self.active_list == "planned":
                        self.mode = "idle"
                        self.goal = None
                        self.planned_waypoints = []
                        self.active_list = "manual"
                        self.wp_idx = 0
                        self._publish_planned_path_msg([])
                    else:
                        self.wp_idx += 1
                        cur = self._current_waypoint_list_locked()
                        if self.wp_idx >= len(cur):
                            self.mode = "idle"
                            self.goal = None
                        else:
                            self.goal = cur[self.wp_idx]
                elif self.mode == "patrol":
                    cur = self._current_waypoint_list_locked()
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

        if mode == "follow" and self.active_list == "planned":
            lin *= 0.85
            ang_cmd *= 0.75

        with self._lock:
            front_level, left_level, right_level = self._contact_levels_locked()
        if lin > 0.0:
            if front_level == "danger":
                lin = 0.0
            elif front_level == "warn":
                lin *= 0.35
            elif left_level == "danger" or right_level == "danger":
                lin *= 0.15
            elif left_level == "warn" or right_level == "warn":
                lin *= 0.6

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

