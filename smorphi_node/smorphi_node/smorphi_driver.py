#!/usr/bin/env python3
# ================================================================
# smorphi_driver.py  —  Complete Final Version
# ROS2 Humble  |  Smorphi 4-wheel Mecanum robot
# ================================================================
#
# WHAT THIS NODE DOES:
#   1. Opens serial port to ESP32
#   2. Reads encoder tick counts from ESP32 at 50 Hz
#   3. Computes wheel displacements via mecanum forward kinematics
#   4. Integrates pose using mid-point RK2 (dead-reckoning odom)
#   5. Publishes /odom  (nav_msgs/Odometry)
#   6. Broadcasts TF:  odom → base_footprint
#   7. Subscribes /cmd_vel, converts to per-wheel PWM via IK
#   8. Sends "d1,d2,d3,d4,p1,p2,p3,p4\n" to ESP32
#
# SERIAL PROTOCOL:
#   RX from ESP32 (50 Hz):   "c1,c2,c3,c4\n"
#     c1=FL  c2=FR  c3=RR  c4=RL   (signed cumulative tick counts)
#
#   TX to ESP32:              "d1,d2,d3,d4,p1,p2,p3,p4\n"
#     d = wheel direction:  0=STOP  1=FORWARD  2=BACKWARD
#     p = wheel PWM:        0-255
#     Order: FL, FR, RR, RL
#
# WHEEL LAYOUT (top view):
#   M1(FL) ╲  front  ╱ M2(FR)
#   M4(RL) ╱  rear   ╲ M3(RR)
#
#   (\) diagonal: FL and RR  →  M1, M3
#   (/) diagonal: FR and RL  →  M2, M4
#
# MECANUM FORWARD KINEMATICS (FK):
#   Δx  = ( FL + FR + RR + RL) / 4      physical forward = odom +X ✓
#   Δy  = (-FL + FR - RR + RL) / 4      physical left    = odom +Y ✓
#   Δθ  = (-FL + FR + RR - RL) / (4×wd) CCW = +Z ✓
#
# MECANUM INVERSE KINEMATICS (IK):
#   wFL = ( vx - vy - wz×wd) / r    vx=+0.2 → all wheels FWD ✓
#   wFR = ( vx + vy + wz×wd) / r    wz=+0.5 → CCW rotation ✓
#   wRR = ( vx - vy + wz×wd) / r
#   wRL = ( vx + vy - wz×wd) / r
#
# CALIBRATION:
#   ticks_per_rev: drive 1m, check odom.x. Adjust:
#     new = old × (odom_reading / real_distance)
#   wheel_dist: rotate 360°, check yaw. Adjust:
#     new = old × (odom_degrees / 360)
#
# TUNING SPEED:
#   max_wheel_vel: increase if robot is too slow at max Nav2 speed
#                  decrease if robot oscillates / overshoots
#   min_motor_pwm: increase if motors don't start at low speeds
#                  decrease if robot jerks on start
#
# TF TREE:
#   map → odom → base_footprint → base_link → laser_link
#   (base_footprint→base_link and base_link→laser_link
#    come from robot_state_publisher, NOT this node)
# ================================================================

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial


# ── Helpers ───────────────────────────────────────────────────

def quat_from_yaw(yaw: float):
    """Return (x, y, z, w) quaternion for a pure yaw rotation."""
    h = yaw * 0.5
    return 0.0, 0.0, math.sin(h), math.cos(h)


def wrap(a: float) -> float:
    """Wrap angle to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


# ── Node ──────────────────────────────────────────────────────

class SmorphiDriver(Node):

    def __init__(self):
        super().__init__('smorphi_driver')

        # ── ROS parameters ────────────────────────────────────
        self.declare_parameter('serial_port',   '/dev/smorphi_mb')
        self.declare_parameter('baud_rate',     115200)
        self.declare_parameter('odom_frame',    'odom')
        self.declare_parameter('base_frame',    'base_footprint')
        self.declare_parameter('publish_tf',    True)

        # ── Physical constants (calibrate these) ──────────────
        # wheel_radius: from URDF = 0.03 m
        self.declare_parameter('wheel_radius',  0.03)

        # ticks_per_rev: GEARING(45) × ENCODERMULT(13) = 585
        # ISR is RISING only so no ×2.
        # Calibrate: drive 1m, if odom shows X metres:
        #   new_ticks_per_rev = 585 × X
        self.declare_parameter('ticks_per_rev', 585.0)

        # wheel_dist = Lx + Ly
        # From URDF: wheel_offset_x(0.035) + wheel_offset_y(0.065) = 0.10
        # Calibrate: rotate 360°, if odom shows D degrees:
        #   new_wheel_dist = 0.10 × (D / 360)
        self.declare_parameter('wheel_dist',    0.10)

        # ── Motor PWM mapping ─────────────────────────────────
        # max_wheel_vel: wheel rad/s that maps to PWM 255
        # Tune: if robot too slow → decrease; too fast → increase
        self.declare_parameter('max_wheel_vel', 6.0)

        # min_motor_pwm: minimum PWM that overcomes static friction
        # Tune: if motors don't spin at low cmd_vel → increase
        self.declare_parameter('min_motor_pwm', 70)

        # ── Thresholds ────────────────────────────────────────
        # cmd_threshold: below this velocity (m/s or rad/s) = stop
        self.declare_parameter('cmd_threshold', 0.001)

        # ── Debug ─────────────────────────────────────────────
        # debug_cmd_vel: print every cmd_vel + wheel PWMs to terminal
        # Run with: --ros-args -p debug_cmd_vel:=true
        self.declare_parameter('debug_cmd_vel', False)

        # ── Read all parameters ───────────────────────────────
        port            = self.get_parameter('serial_port').value
        baud            = self.get_parameter('baud_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pub_tf     = self.get_parameter('publish_tf').value

        r               = self.get_parameter('wheel_radius').value
        ticks           = self.get_parameter('ticks_per_rev').value
        self.wd         = self.get_parameter('wheel_dist').value
        self.r          = r
        self.max_wv     = self.get_parameter('max_wheel_vel').value
        self.min_pwm    = self.get_parameter('min_motor_pwm').value
        self.cmd_thr    = self.get_parameter('cmd_threshold').value
        self.debug      = self.get_parameter('debug_cmd_vel').value

        # metres of wheel arc per encoder tick
        self.mpt = (2.0 * math.pi * r) / ticks

        # ── Odometry state ────────────────────────────────────
        self.x      = 0.0
        self.y      = 0.0
        self.yaw    = 0.0
        self.vx     = 0.0
        self.vy     = 0.0
        self.wz_out = 0.0
        self.prev_ticks = [0, 0, 0, 0]
        self.prev_time  = self.get_clock().now()

        # ── Command state ─────────────────────────────────────
        self._cmd_str  = '0,0,0,0,0,0,0,0\n'  # all stop
        self._cmd_lock = threading.Lock()

        # ── QoS ──────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        # ── ROS interfaces ────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_br    = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos)

        # ── Serial port ───────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(
                f'[smorphi_driver] Serial opened: {port} @ {baud} baud\n'
                f'  mpt          = {self.mpt:.7f} m/tick\n'
                f'  wheel_dist   = {self.wd} m\n'
                f'  max_wheel_vel= {self.max_wv} rad/s\n'
                f'  min_pwm      = {self.min_pwm}\n'
                f'  debug        = {self.debug}')
        except serial.SerialException as e:
            self.get_logger().fatal(
                f'[smorphi_driver] Cannot open {port}: {e}\n'
                f'  → Check: ls /dev/ttyUSB*  or  ls /dev/smorphi_mb')
            raise SystemExit(1)

        # ── Start background serial thread ────────────────────
        self._running = True
        threading.Thread(target=self._serial_loop, daemon=True).start()

    # ─────────────────────────────────────────────────────────
    # /cmd_vel callback — Mecanum Inverse Kinematics
    # ─────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        if self.debug:
            self.get_logger().info(
                f'CMD_VEL  vx={vx:+.4f}  vy={vy:+.4f}  wz={wz:+.4f}')

        # All below threshold → full stop
        if abs(vx) < self.cmd_thr and \
           abs(vy) < self.cmd_thr and \
           abs(wz) < self.cmd_thr:
            with self._cmd_lock:
                self._cmd_str = '0,0,0,0,0,0,0,0\n'
            return

        # ── Mecanum inverse kinematics ────────────────────────
        # DO NOT negate vx/vy here.
        # The FK negation in _serial_loop handles the odom frame flip.
        # The IK must use raw cmd_vel values so motors spin in the
        # correct physical direction:
        #   vx=+0.2 → all wheels FORWARD → robot moves physically forward
        #   vy=+0.2 → strafe right physically
        #   wz=+0.5 → CCW rotation (already correct, never negated)
        wFL = ( vx - vy - wz * self.wd) / self.r
        wFR = ( vx + vy + wz * self.wd) / self.r
        wRR = ( vx - vy + wz * self.wd) / self.r
        wRL = ( vx + vy - wz * self.wd) / self.r

        wheels = [wFL, wFR, wRR, wRL]

        # Proportional scaling — preserve motion ratios if any wheel
        # exceeds max_wheel_vel
        max_w = max(abs(w) for w in wheels)
        if max_w > self.max_wv:
            wheels = [w * (self.max_wv / max_w) for w in wheels]

        dirs, pwms = [], []
        for w in wheels:
            d, p = self._w_to_dp(w)
            dirs.append(d)
            pwms.append(p)

        cmd = (f'{dirs[0]},{dirs[1]},{dirs[2]},{dirs[3]},'
               f'{pwms[0]},{pwms[1]},{pwms[2]},{pwms[3]}\n')

        if self.debug:
            self.get_logger().info(
                f'WHEELS   FL={wheels[0]:+.2f}  FR={wheels[1]:+.2f}  '
                f'RR={wheels[2]:+.2f}  RL={wheels[3]:+.2f} rad/s')
            self.get_logger().info(f'ESP32 TX {cmd.strip()}')

        with self._cmd_lock:
            self._cmd_str = cmd

    def _w_to_dp(self, w: float):
        """Wheel angular velocity (rad/s) → (direction, PWM)."""
        if abs(w) < 0.01:
            return 0, 0
        direction = 1 if w > 0 else 2
        ratio = min(abs(w) / self.max_wv, 1.0)
        pwm   = int(self.min_pwm + (255 - self.min_pwm) * ratio)
        return direction, max(0, min(255, pwm))

    # ─────────────────────────────────────────────────────────
    # Serial loop — reads encoders, integrates odom, sends cmd
    # ─────────────────────────────────────────────────────────

    def _serial_loop(self):
        while self._running:

            # Read one line from ESP32
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break

            if not raw:
                continue
            try:
                line = raw.decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            # Skip boot messages and blank lines
            if not line or not (line[0].isdigit() or line[0] == '-'):
                continue

            # Parse "c1,c2,c3,c4"
            parts = line.split(',')
            if len(parts) != 4:
                continue
            try:
                ticks = [int(p) for p in parts]
            except ValueError:
                continue

            # ── Δt ────────────────────────────────────────────
            now = self.get_clock().now()
            dt  = (now - self.prev_time).nanoseconds * 1e-9

            if dt <= 0.0 or dt > 1.0:
                self.prev_ticks = ticks
                self.prev_time  = now
                self._send()
                continue

            # ── Wheel displacements [m] ───────────────────────
            # ESP32 serial order: FL, FR, RR, RL
            d = [(ticks[i] - self.prev_ticks[i]) * self.mpt
                 for i in range(4)]
            self.prev_ticks = ticks
            self.prev_time  = now

            dFL, dFR, dRR, dRL = d[0], d[1], d[2], d[3]

            # ── Mecanum forward kinematics ────────────────────
            # NO negation — encoder counts are positive when robot
            # moves physically forward, and odom +X = physical forward.
            delta_x   = ( dFL + dFR + dRR + dRL) / 4.0
            delta_y   = (-dFL + dFR - dRR + dRL) / 4.0
            delta_yaw = (-dFL + dFR + dRR - dRL) / (4.0 * self.wd)

            # ── Mid-point RK2 pose integration ────────────────
            yaw_mid   = self.yaw + delta_yaw * 0.5
            self.x   += delta_x * math.cos(yaw_mid) - delta_y * math.sin(yaw_mid)
            self.y   += delta_x * math.sin(yaw_mid) + delta_y * math.cos(yaw_mid)
            self.yaw  = wrap(self.yaw + delta_yaw)

            # Body-frame velocities for twist field
            self.vx     = delta_x   / dt
            self.vy     = delta_y   / dt
            self.wz_out = delta_yaw / dt

            # ── Publish odom + TF ─────────────────────────────
            self._publish(now)

            # ── Send motor command to ESP32 ───────────────────
            self._send()

    # ─────────────────────────────────────────────────────────
    # Publish /odom and odom → base_footprint TF
    # ─────────────────────────────────────────────────────────

    def _publish(self, now_ros):
        stamp           = now_ros.to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (6×6 row-major, all float)
        odom.pose.covariance = [
            5e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  5e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  5e-3,
        ]

        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.linear.y  = self.vy
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.z = self.wz_out

        odom.twist.covariance = [
            5e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  5e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  5e-3,
        ]

        self.odom_pub.publish(odom)

        # TF: odom → base_footprint
        if self.pub_tf:
            tf = TransformStamped()
            tf.header.stamp            = stamp
            tf.header.frame_id         = self.odom_frame
            tf.child_frame_id          = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x    = qx
            tf.transform.rotation.y    = qy
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self.tf_br.sendTransform(tf)

    # ─────────────────────────────────────────────────────────
    # Send motor command to ESP32
    # ─────────────────────────────────────────────────────────

    def _send(self):
        with self._cmd_lock:
            s = self._cmd_str
        try:
            self.ser.write(s.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ─────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.write(b'0,0,0,0,0,0,0,0\n')
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmorphiDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
