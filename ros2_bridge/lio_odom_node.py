#!/usr/bin/env python3
"""
LIO simplifié : LiDAR + IMU.
- Position (x, y) : déplacement issu du LiDAR (ICP 2D entre scans).
- Orientation (theta) : yaw de l’IMU (/imu/rpy_deg).
Publie TF odom -> base_link et optionnellement /odom.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

from lidar_odom_node import (
    scan_to_xy,
    icp_2d_step,
    apply_transform_2d,
)

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


class LioOdomNode(Node):
    def __init__(self):
        super().__init__("lio_odom")
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
        self.sub_scan = self.create_subscription(
            LaserScan, "/lidar_scan", self.scan_cb, qos
        )
        self.sub_imu = self.create_subscription(
            Vector3Stamped, "/imu/rpy_deg", self.imu_cb, 10
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
        self.imu_yaw_deg = 0.0
        self.imu_stamp = None
        self.get_logger().info("LIO (LiDAR+IMU) démarré, TF odom -> base_link")

    def imu_cb(self, msg: Vector3Stamped):
        self.imu_yaw_deg = msg.vector.z
        self.imu_stamp = msg.header.stamp

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

        if cKDTree is None:
            self.prev_xy = xy.copy()
            self.prev_stamp = stamp
            return

        tree = cKDTree(self.prev_xy)
        src = xy.copy()
        dx_tot, dy_tot = 0.0, 0.0

        for _ in range(n_iter):
            dx, dy, _ = icp_2d_step(src, self.prev_xy, tree)
            dx_tot += dx
            dy_tot += dy
            src = apply_transform_2d(src, dx, dy, 0.0)

        theta_prev = self.pose_theta
        c, s = math.cos(theta_prev), math.sin(theta_prev)
        self.pose_x += c * dx_tot - s * dy_tot
        self.pose_y += s * dx_tot + c * dy_tot
        self.pose_theta = math.radians(self.imu_yaw_deg)
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
        q = (0.0, 0.0, math.sin(self.pose_theta / 2), math.cos(self.pose_theta / 2))
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0
        q = (0.0, 0.0, math.sin(self.pose_theta / 2), math.cos(self.pose_theta / 2))
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = q[0], q[1]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q[2], q[3]
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = LioOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
