#!/usr/bin/env python3
"""
Monitoring IMU pour Foxglove:
- /imu/monitor/angles_deg       (Vector3Stamped): roll/pitch/yaw en deg
- /imu/monitor/accel_ms2        (Vector3Stamped): ax/ay/az en m/s^2
- /imu/monitor/summary          (String): stats fenetre glissante (std/drift/stationnaire)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ImuMonitor(Node):
    def __init__(self):
        super().__init__("imu_monitor")
        self.declare_parameter("window_seconds", 6.0)
        self.declare_parameter("publish_period_s", 0.1)
        self.declare_parameter("stationary_accel_std_th", 0.08)
        self.declare_parameter("stationary_yaw_drift_deg_s_th", 1.2)

        self.window_s = float(self.get_parameter("window_seconds").value)
        period = float(self.get_parameter("publish_period_s").value)

        self.pub_angles = self.create_publisher(Vector3Stamped, "/imu/monitor/angles_deg", 10)
        self.pub_accel = self.create_publisher(Vector3Stamped, "/imu/monitor/accel_ms2", 10)
        self.pub_summary = self.create_publisher(String, "/imu/monitor/summary", 10)

        self.sub_rpy = self.create_subscription(
            Vector3Stamped, "/imu/rpy_deg", self.rpy_cb, 20
        )
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.imu_cb, 20)

        self.rpy_hist = deque()   # (t, roll, pitch, yaw)
        self.acc_hist = deque()   # (t, ax, ay, az)
        self.last_rpy = None
        self.last_acc = None

        self.timer = self.create_timer(period, self.publish_cb)
        self.get_logger().info("ImuMonitor actif: /imu/monitor/angles_deg, /imu/monitor/accel_ms2, /imu/monitor/summary")

    def _trim(self, now_s: float):
        tmin = now_s - self.window_s
        while self.rpy_hist and self.rpy_hist[0][0] < tmin:
            self.rpy_hist.popleft()
        while self.acc_hist and self.acc_hist[0][0] < tmin:
            self.acc_hist.popleft()

    def rpy_cb(self, msg: Vector3Stamped):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        r = float(msg.vector.x)
        p = float(msg.vector.y)
        y = float(msg.vector.z)
        self.last_rpy = (r, p, y)
        self.rpy_hist.append((now_s, r, p, y))
        self._trim(now_s)

    def imu_cb(self, msg: Imu):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        self.last_acc = (ax, ay, az)
        self.acc_hist.append((now_s, ax, ay, az))
        self._trim(now_s)

    @staticmethod
    def _std(vals):
        if not vals:
            return 0.0
        m = sum(vals) / len(vals)
        return math.sqrt(sum((v - m) * (v - m) for v in vals) / len(vals))

    def publish_cb(self):
        now = self.get_clock().now().to_msg()

        if self.last_rpy is not None:
            r, p, y = self.last_rpy
            msg = Vector3Stamped()
            msg.header.stamp = now
            msg.header.frame_id = "imu_link"
            msg.vector.x = r
            msg.vector.y = p
            msg.vector.z = y
            self.pub_angles.publish(msg)

        if self.last_acc is not None:
            ax, ay, az = self.last_acc
            msg = Vector3Stamped()
            msg.header.stamp = now
            msg.header.frame_id = "imu_link"
            msg.vector.x = ax
            msg.vector.y = ay
            msg.vector.z = az
            self.pub_accel.publish(msg)

        if not self.rpy_hist or not self.acc_hist:
            return

        # Stats accelerations
        axs = [v[1] for v in self.acc_hist]
        ays = [v[2] for v in self.acc_hist]
        azs = [v[3] for v in self.acc_hist]
        ax_std = self._std(axs)
        ay_std = self._std(ays)
        az_std = self._std(azs)
        acc_std_norm = math.sqrt(ax_std * ax_std + ay_std * ay_std + az_std * az_std)

        # Drift yaw deg/s sur fenetre
        t0, _, _, y0 = self.rpy_hist[0]
        t1, _, _, y1 = self.rpy_hist[-1]
        dt = max(1e-3, t1 - t0)
        dy = y1 - y0
        while dy > 180.0:
            dy -= 360.0
        while dy < -180.0:
            dy += 360.0
        yaw_drift_deg_s = dy / dt

        acc_th = float(self.get_parameter("stationary_accel_std_th").value)
        yaw_th = float(self.get_parameter("stationary_yaw_drift_deg_s_th").value)
        stationary = (acc_std_norm < acc_th) and (abs(yaw_drift_deg_s) < yaw_th)

        summary = String()
        summary.data = (
            f"stationary={str(stationary).lower()} "
            f"acc_std_norm={acc_std_norm:.3f}m/s2 "
            f"yaw_drift={yaw_drift_deg_s:.2f}deg/s "
            f"window={self.window_s:.1f}s"
        )
        self.pub_summary.publish(summary)


def main():
    rclpy.init()
    node = ImuMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

