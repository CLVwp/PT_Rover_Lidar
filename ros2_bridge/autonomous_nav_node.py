#!/usr/bin/env python3
"""
Navigation autonome basée uniquement sur le LiDAR.
- Lit /lidar_scan, évite les obstacles (secteurs avant / gauche / droite).
- Envoie les commandes de vitesse au rover via WebSocket (rover_cmd).
À lancer avec le bridge actif (LiDAR + IMU).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Import local (même dossier que rover_cmd.py)
try:
    from rover_cmd import set_speed, stop, ROVER_IP
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from rover_cmd import set_speed, stop, ROVER_IP

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# --- Logique raycasts inspirée de la page web ---
# Angles en degrés, 0 = devant, + à gauche, - à droite (convention ROS)
RAYCAST_ANGLES = [-45, -30, -15, 0, 15, 30, 45]
RAYCAST_ANGLES_FRONT = [-15, 0, 15]
OBSTACLE_M = 0.40       # seuil “obstacle” pour l'équilibrage
HARD_OBST_M = 0.25      # urgence : pivot si en-dessous
MAX_RAY_M = 2.5         # distance max prise en compte pour le centrage
TRACK_CURVATURE_GAIN = 0.28  # même idée que sur la page (empattement / gain courbure)

class AutonomousNavNode(Node):
    def __init__(self):
        super().__init__("autonomous_nav")
        self.declare_parameter("base_speed", 0.4)
        self.declare_parameter("min_dist_m", 0.4)
        self.declare_parameter("safe_dist_m", 0.7)
        self.declare_parameter("sector_deg", 50.0)
        self.declare_parameter("cmd_period_s", 0.1)
        self.declare_parameter("enabled", True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.sub = self.create_subscription(
            LaserScan, "/lidar_scan", self.scan_cb, qos
        )
        self.timer = self.create_timer(
            self.get_parameter("cmd_period_s").value, self.cmd_timer_cb
        )

        self.latest_scan = None
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.get_logger().info(
            f"Autonomous nav démarré (commandes -> rover {ROVER_IP})"
        )

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # --- Helpers raycasts façon page web ---
    def _build_rays(self, msg: LaserScan) -> dict:
        """
        Construit un dico angle(deg) -> distance (m) en utilisant la même
        logique que la page : rayons fixes autour de l'avant, distance min
        dans un petit cône autour de chaque angle.
        """
        rays = {a: float("inf") for a in RAYCAST_ANGLES}
        angle = float(msg.angle_min)
        da = float(msg.angle_increment)

        # Demi-ouverture du cône pour chaque rayon (en degrés)
        sector_half_deg = 10.0

        for r in msg.ranges:
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                angle += da
                continue

            # ROS: angle en rad, 0 devant, + à gauche ; on passe en degrés [-180, 180]
            deg = math.degrees(angle)
            if deg > 180.0:
                deg -= 360.0

            # On ne regarde que +-55° autour de l'avant, comme sur la page
            if abs(deg) > 55.0:
                angle += da
                continue

            for a in RAYCAST_ANGLES:
                if abs(deg - a) <= sector_half_deg and r < rays[a]:
                    rays[a] = r

            angle += da

        # Valeur "loin" par défaut si rien vu
        for a in rays:
            if rays[a] == float("inf"):
                rays[a] = 9999.0
        return rays

    @staticmethod
    def _avg_valid(values):
        vals = [v for v in values if v < MAX_RAY_M]
        if not vals:
            return 0.0
        return sum(vals) / len(vals)

    def cmd_timer_cb(self):
        if not self.get_parameter("enabled").value:
            stop()
            return
        if self.latest_scan is None:
            return

        msg = self.latest_scan
        base_fast = self.get_parameter("base_speed").value  # ~0.4 par défaut

        # 1) Construire les rayons façon page web
        rays = self._build_rays(msg)
        eff = lambda a: min(rays[a], MAX_RAY_M)

        # 2) Zone frontale
        min_front = min(eff(a) for a in RAYCAST_ANGLES_FRONT)
        front_hard_blocked = min_front < HARD_OBST_M

        # 3) Cas urgence : pivot sur place vers le côté le plus dégagé
        if front_hard_blocked:
            best_angle = 0
            best_dist = 0.0
            for a in RAYCAST_ANGLES:
                d = eff(a)
                if d > best_dist:
                    best_dist = d
                    best_angle = a

            # même logique que la page : si c'est plus libre à droite (angle négatif),
            # on pivote vers la droite, sinon vers la gauche
            turn = -0.35 if best_angle < 0 else 0.35
            L = turn
            R = -turn

            self.left_speed = max(-1.0, min(1.0, L))
            self.right_speed = max(-1.0, min(1.0, R))
            set_speed(self.left_speed, self.right_speed)
            return

        # 4) Cas normal : rester centré + suivre la courbure (comme sur la page)
        left_clear = self._avg_valid([eff(-45), eff(-30), eff(-15)])
        right_clear = self._avg_valid([eff(15), eff(30), eff(45)])

        left_slope = (eff(-15) - eff(-45)) / 30.0
        right_slope = (eff(15) - eff(45)) / 30.0

        kappa_center = (left_clear - right_clear) * 0.00045
        kappa_slope = (left_slope - right_slope) * 0.0012
        kappa = kappa_center + kappa_slope
        kappa = max(-0.9, min(0.9, kappa))

        # Adapter la vitesse en fonction de ce qu'on a devant
        base_slow = 0.22
        if min_front < 0.6:
            # interpolation linéaire entre slow et fast
            alpha = max(0.0, min(1.0, (min_front - HARD_OBST_M) / (0.6 - HARD_OBST_M)))
            base = base_slow + (base_fast - base_slow) * alpha
        else:
            base = base_fast

        L = base - kappa * TRACK_CURVATURE_GAIN
        R = base + kappa * TRACK_CURVATURE_GAIN

        L = max(-1.0, min(1.0, L))
        R = max(-1.0, min(1.0, R))
        self.left_speed = L
        self.right_speed = R
        set_speed(L, R)


def main():
    rclpy.init()
    node = AutonomousNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
