#!/usr/bin/env python3
"""
Estimateur de trajectoire (odométrie) basé uniquement sur le LiDAR.
Utilise un appariement de scans successifs (style ICP 2D) et publie la pose via ROS2 TF.
Topic: /lidar_scan  ->  TF: odom -> base_link, topic /odom (optionnel).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


def scan_to_xy(scan: LaserScan, max_range=5.0, min_range=0.05):
    """Convertit un LaserScan en nuage de points 2D (x, y) dans le repère du lidar."""
    pts = []
    a = float(scan.angle_min)
    da = float(scan.angle_increment)
    for i, r in enumerate(scan.ranges):
        if min_range <= r <= max_range and math.isfinite(r):
            angle = a + i * da
            # repère lidar: x avant, y gauche (ROS convention)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            pts.append([x, y])
    return np.array(pts) if pts else np.empty((0, 2))


def icp_2d_step(src: np.ndarray, dst: np.ndarray, tree: "cKDTree"):
    """Une itération ICP: renvoie (dx, dy, dtheta) pour aligner src sur dst."""
    if len(src) < 3 or tree is None:
        return 0.0, 0.0, 0.0
    idx = tree.query(src, k=1)[1]
    matched_dst = dst[idx]
    cs = src.mean(axis=0)
    cd = matched_dst.mean(axis=0)
    S = src - cs
    D = matched_dst - cd
    H = S.T @ D
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt = Vt.copy()
        Vt[-1] *= -1
        R = Vt.T @ U.T
    dtheta = math.atan2(R[1, 0], R[0, 0])
    # Translation: cd - R @ cs (pour que R @ src + (dx,dy) ait centre cd)
    Rcs = R @ cs
    dx = cd[0] - Rcs[0]
    dy = cd[1] - Rcs[1]
    return dx, dy, dtheta


def apply_transform_2d(pts: np.ndarray, dx: float, dy: float, dtheta: float) -> np.ndarray:
    """Applique (dx, dy, dtheta) aux points."""
    if len(pts) == 0:
        return pts
    c, s = math.cos(dtheta), math.sin(dtheta)
    R = np.array([[c, -s], [s, c]])
    return (pts @ R.T) + np.array([dx, dy])


def laserscan_to_xy(scan: LaserScan, max_range=5.0, min_range=0.05):
    return scan_to_xy(scan, max_range, min_range)


class LidarOdomNode(Node):
    def __init__(self):
        super().__init__("lidar_odom")
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("icp_iterations", 5)
        self.declare_parameter("min_points", 20)
        self.declare_parameter("publish_odom_topic", True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            LaserScan, "/lidar_scan", self.scan_cb, qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.get_parameter("publish_odom_topic").value:
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        else:
            self.odom_pub = None

        self.prev_xy = None
        self.prev_stamp = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.get_logger().info("LidarOdom (LiDAR-only) démarré, publication TF odom -> base_link")

    def scan_cb(self, msg: LaserScan):
        max_r = self.get_parameter("max_range").value
        min_r = self.get_parameter("min_range").value
        n_iter = self.get_parameter("icp_iterations").value
        min_pts = self.get_parameter("min_points").value

        xy = scan_to_xy(msg, max_range=max_r, min_range=min_r)
        if len(xy) < min_pts:
            return

        stamp = msg.header.stamp
        if self.prev_xy is None:
            self.prev_xy = xy.copy()
            self.prev_stamp = stamp
            return

        # ICP: aligner xy (current) sur prev_xy (reference)
        if cKDTree is None:
            self.get_logger().warn("scipy non installé: pas d'ICP, pose inchangée.")
            self.prev_xy = xy.copy()
            self.prev_stamp = stamp
            return

        tree = cKDTree(self.prev_xy)
        src = xy.copy()
        dx_tot, dy_tot, dtheta_tot = 0.0, 0.0, 0.0

        for _ in range(n_iter):
            dx, dy, dtheta = icp_2d_step(src, self.prev_xy, tree)
            dx_tot += dx
            dy_tot += dy
            dtheta_tot += dtheta
            src = apply_transform_2d(src, dx, dy, dtheta)

        # Mouvement dans le repère monde (odom): on a déplacé le robot de (dx_tot, dy_tot, dtheta_tot) dans le repère du lidar au temps t-1
        # Dans odom, la pose du robot à t-1 est (pose_x, pose_y, pose_theta). Le déplacement dans le repère robot est (dx_tot, dy_tot, dtheta_tot).
        # Nouvelle pose:
        c, s = math.cos(self.pose_theta), math.sin(self.pose_theta)
        self.pose_x += c * dx_tot - s * dy_tot
        self.pose_y += s * dx_tot + c * dy_tot
        self.pose_theta += dtheta_tot
        self.pose_theta = math.atan2(math.sin(self.pose_theta), math.cos(self.pose_theta))

        self.prev_xy = xy.copy()
        self.prev_stamp = stamp

        self.publish_tf(stamp)
        if self.odom_pub:
            self.publish_odom(stamp)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        q = self._yaw_to_quat(self.pose_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0
        q = self._yaw_to_quat(self.pose_theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

    @staticmethod
    def _yaw_to_quat(yaw):
        return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def main():
    rclpy.init()
    node = LidarOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
