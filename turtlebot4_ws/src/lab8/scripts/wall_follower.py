#!/usr/bin/env python3
"""
Wall Follower - Lab 8 (TurtleBot4 real robot)
Right-wall-following PID.  Enable/disable via /wall_follow_active (std_msgs/Bool).
/scan subscribed with BEST_EFFORT (qos_profile_sensor_data).
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(x, lo, hi):
    return max(lo, min(hi, float(x)))


class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.setpoint = float(setpoint)
        self._prev_err = 0.0
        self._integral = 0.0
        self._initialized = False

    def reset(self):
        self._prev_err = 0.0
        self._integral = 0.0
        self._initialized = False

    def __call__(self, measurement, dt):
        dt = max(float(dt), 1e-3)
        err = self.setpoint - float(measurement)
        self._integral = clamp(self._integral + err * dt, -1.0, 1.0)
        d_out = 0.0
        if self._initialized:
            d_out = self.kd * (err - self._prev_err) / dt
        self._initialized = True
        self._prev_err = err
        return self.kp * err + self.ki * self._integral + d_out


class WallFollowerNode(Node):
    """
    Right-wall-following node.
    Listens to /wall_follow_active (Bool).  Only drives when active=True.
    When active=False, publishes a zero Twist to stop the robot.
    """

    def __init__(self):
        super().__init__('wall_follower')

        # ---------- parameters (mirrors classmate lab7/wall_follower.py) ----------
        self.declare_parameter('target_dist', 0.40)        # desired right-wall distance [m]
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('min_speed', 0.08)
        self.declare_parameter('max_ang', 2.0)
        self.declare_parameter('front_stop_dist', 0.40)    # turn left when front < this
        self.declare_parameter('corner_turn_ang', 1.75)    # angular vel when turning corner
        self.declare_parameter('kp', 1.8)
        self.declare_parameter('ki', 0.10)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('hard_min_right', 0.22)     # emergency left-turn threshold
        self.declare_parameter('hard_turn_gain', 10.0)
        self.declare_parameter('hard_turn_max', 1.8)
        self.declare_parameter('lost_wall_dist', 0.70)     # steer right to re-acquire wall
        self.declare_parameter('lookahead', 0.35)
        self.declare_parameter('control_hz', 20.0)

        def _p(name):
            return self.get_parameter(name).value

        self.target_dist     = float(_p('target_dist'))
        self.linear_speed    = float(_p('linear_speed'))
        self.min_speed       = float(_p('min_speed'))
        self.max_ang         = float(_p('max_ang'))
        self.front_stop_dist = float(_p('front_stop_dist'))
        self.corner_turn_ang = float(_p('corner_turn_ang'))
        self.hard_min_right  = float(_p('hard_min_right'))
        self.hard_turn_gain  = float(_p('hard_turn_gain'))
        self.hard_turn_max   = float(_p('hard_turn_max'))
        self.lost_wall_dist  = float(_p('lost_wall_dist'))
        self.lookahead       = float(_p('lookahead'))
        control_hz           = float(_p('control_hz'))

        self.pid = PIDController(_p('kp'), _p('ki'), _p('kd'), self.target_dist)
        self.active = False
        self.last_scan: LaserScan | None = None
        self.prev_time = self.get_clock().now()

        # ---------- publishers / subscribers ----------
        cmd_qos = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)
        # /scan uses BEST_EFFORT (Create3 sensor data)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/wall_follow_active', self._active_cb, 10)
        self.create_timer(1.0 / max(control_hz, 1.0), self._control_loop)

        self.get_logger().info(
            f'WallFollower ready  target={self.target_dist}m  '
            f'front_stop={self.front_stop_dist}m  hard_min={self.hard_min_right}m'
        )

    # ------------------------------------------------------------------
    def _active_cb(self, msg: Bool):
        if msg.data != self.active:
            self.get_logger().info(
                f'wall_follow_active → {"ENABLED" if msg.data else "DISABLED"}'
            )
            if not msg.data:
                self.pid.reset()
        self.active = msg.data

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    # ------------------------------------------------------------------
    def _range_at(self, scan: LaserScan, angle_rad: float, window_deg: float = 10.0) -> float:
        """Robust range at a given beam angle (20th percentile in ±window)."""
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)
        n = len(ranges)
        win = max(1, int(math.radians(window_deg) / max(scan.angle_increment, 1e-6)))
        center = int(clamp((angle_rad - scan.angle_min) / scan.angle_increment, 0, n - 1))
        i0 = max(0, center - win)
        i1 = min(n - 1, center + win)
        w = ranges[i0 : i1 + 1]
        w = w[(w >= scan.range_min) & (w <= scan.range_max)]
        if w.size == 0:
            return float(scan.range_max)
        return float(np.percentile(w, 20))

    def _predict_right(self, d_right: float, d_fr: float) -> float:
        """Lookahead prediction of right distance (from classmate algorithm)."""
        theta = math.pi / 4.0
        d_r  = max(0.05, d_right)
        d_fr = max(0.05, d_fr)
        alpha = math.atan2(d_fr * math.cos(theta) - d_r, d_fr * math.sin(theta))
        d_now = d_r * math.cos(alpha)
        return d_now + self.lookahead * math.sin(alpha)

    # ------------------------------------------------------------------
    def _control_loop(self):
        if not self.active:
            self.cmd_pub.publish(Twist())   # keep robot stopped
            return

        if self.last_scan is None:
            self.get_logger().warn(
                'Active but no /scan yet — waiting...', throttle_duration_sec=3.0)
            return

        scan = self.last_scan
        now  = self.get_clock().now()
        dt   = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        dt = max(dt, 1e-3)

        d_front = self._range_at(scan, 0.0,            window_deg=10.0)
        d_fr    = self._range_at(scan, -math.pi / 4,   window_deg=10.0)
        d_right = self._range_at(scan, -math.pi / 2,   window_deg=12.0)

        cmd = Twist()

        # ── Corner / obstacle ahead → turn left ───────────────────────
        if d_front < self.front_stop_dist or d_fr < self.front_stop_dist:
            cmd.linear.x  = self.min_speed
            cmd.angular.z = self.corner_turn_ang
            self.pid.reset()
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'CORNER  front={d_front:.2f}  fr={d_fr:.2f}',
                throttle_duration_sec=1.0,
            )
            return

        # ── PID right-wall follow ──────────────────────────────────────
        d_pred = self._predict_right(d_right, d_fr)
        z = self.pid(d_pred, dt)

        # Emergency: too close to right wall → hard left correction
        if d_right < self.hard_min_right:
            extra = clamp(self.hard_turn_gain * (self.hard_min_right - d_right),
                          0.0, self.hard_turn_max)
            z += extra
            cmd.linear.x = self.min_speed
        else:
            cmd.linear.x = self.linear_speed

        # Lost wall on right → gentle right steer to re-acquire
        if d_right > self.lost_wall_dist:
            t = clamp((d_right - self.lost_wall_dist) / 1.0, 0.0, 1.0)
            z -= 0.25 * t

        cmd.angular.z = clamp(z, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'right={d_right:.2f}  front={d_front:.2f}  v={cmd.linear.x:.2f}  w={cmd.angular.z:+.2f}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # stop robot on exit
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
