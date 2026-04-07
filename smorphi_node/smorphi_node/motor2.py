#!/usr/bin/env python3
# ================================================================
# smorphi_driver.py  —  Definitive Final Version
# ROS2 Humble  |  Smorphi 4-wheel Mecanum
# ================================================================
#
# SERIAL PROTOCOL:
#   RX from ESP32 (50 Hz):  "c1,c2,c3,c4\n"  (encoder counts)
#   TX to   ESP32:          "d1,d2,d3,d4,p1,p2,p3,p4\n"
#     d = direction: 0=STOP  1=FORWARD  2=BACKWARD
#     p = PWM:       0-255
#     Order: FL, FR, RR, RL
#
# MECANUM INVERSE KINEMATICS:
#   wFL = (vx - vy - wz*wd) / r
#   wFR = (vx + vy + wz*wd) / r
#   wRR = (vx - vy + wz*wd) / r   (note: RR same sign as FL for vx,vy)
#   wRL = (vx + vy - wz*wd) / r
#
# WHEEL LAYOUT:
#   M1(FL)╲  ╱M2(FR)    serial index: FL=0 FR=1 RR=2 RL=3
#   M4(RL)╱  ╲M3(RR)
#
# DIAGNOSTIC:
#   Set debug_cmd_vel:=true to log every cmd_vel and PWM output.
#   Run:  ros2 topic echo /cmd_vel   to see what Nav2 sends.
#   Run:  ros2 topic echo /odom      to see odometry.
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


def quat_from_yaw(yaw: float):
    h = yaw * 0.5
    return 0.0, 0.0, math.sin(h), math.cos(h)

def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class SmorphiDriver(Node):

    def __init__(self):
        super().__init__('smorphi_driver')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('serial_port',    '/dev/smorphi_mb')
        self.declare_parameter('baud_rate',      115200)
        self.declare_parameter('odom_frame',     'odom')
        self.declare_parameter('base_frame',     'base_footprint')
        self.declare_parameter('publish_tf',     True)

        # Calibrated physical constants
        self.declare_parameter('wheel_radius',   0.03)    # metres
        # ticks_per_rev: odom distance is correct (1m real = 1m odom confirmed).
        # Only the direction (sign) was wrong, fixed by negating FK/IK above.
        self.declare_parameter('ticks_per_rev',  292.5)
        self.declare_parameter('wheel_dist',     0.10)    # Lx+Ly metres

        # Motor speed mapping
        # max_wheel_vel: wheel angular velocity (rad/s) that maps to PWM 255
        # Estimate: motor 110RPM = 11.5 rad/s output shaft
        # At wheel: 11.5 / GEARING... but we calibrate empirically.
        # If robot moves too slow: decrease max_wheel_vel
        # If robot moves too fast: increase max_wheel_vel
        self.declare_parameter('max_wheel_vel',  6.0)     # rad/s → PWM 255
        self.declare_parameter('min_motor_pwm',  70)      # min PWM to overcome friction

        # Zero threshold for cmd_vel components
        self.declare_parameter('cmd_threshold',  0.001)

        # initial_yaw: set this if the robot moves BACKWARD when Nav2
        # commands forward. It corrects the mismatch between the robot's
        # physical forward direction and Nav2's +X odom axis.
        #
        # How to find the right value:
        #   1. Place robot, launch driver, launch Nav2
        #   2. Send a Nav2 goal directly in front of the robot
        #   3. If robot goes BACKWARD  → set initial_yaw: 3.14159  (π)
        #   4. If robot goes LEFT      → set initial_yaw: -1.5708  (-π/2)
        #   5. If robot goes RIGHT     → set initial_yaw:  1.5708  (π/2)
        #   6. If robot goes correctly → keep initial_yaw: 0.0
        self.declare_parameter('initial_yaw',    0.0)

        # Print every cmd_vel and resulting PWM to terminal (for debugging)
        self.declare_parameter('debug_cmd_vel',  False)

        # ── Read parameters ───────────────────────────────────
        port           = self.get_parameter('serial_port').value
        baud           = self.get_parameter('baud_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pub_tf     = self.get_parameter('publish_tf').value

        r              = self.get_parameter('wheel_radius').value
        ticks          = self.get_parameter('ticks_per_rev').value
        self.wd        = self.get_parameter('wheel_dist').value
        self.r         = r
        self.max_wv    = self.get_parameter('max_wheel_vel').value
        self.min_pwm   = self.get_parameter('min_motor_pwm').value
        self.cmd_thr   = self.get_parameter('cmd_threshold').value
        self.debug     = self.get_parameter('debug_cmd_vel').value
        initial_yaw    = self.get_parameter('initial_yaw').value

        self.mpt = (2.0 * math.pi * r) / ticks

        # ── Odometry state ────────────────────────────────────
        self.x = self.y = 0.0
        self.yaw        = initial_yaw   # starts at initial_yaw, not 0
        self.vx = self.vy = self.wz_vel = 0.0
        self.prev_ticks = [0, 0, 0, 0]
        self.prev_time  = self.get_clock().now()

        # ── Command state ─────────────────────────────────────
        # "0,0,0,0,0,0,0,0\n" = all stop
        self._cmd_str  = '0,0,0,0,0,0,0,0\n'
        self._cmd_lock = threading.Lock()

        # ── ROS interfaces ────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_br    = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos)

        # ── Serial port ───────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(
                f'Serial: {port} @ {baud} | '
                f'mpt={self.mpt:.6f} m/tick | '
                f'wheel_dist={self.wd} m | '
                f'max_wheel_vel={self.max_wv} rad/s | '
                f'min_pwm={self.min_pwm} | '
                f'initial_yaw={initial_yaw:.4f} rad')
            if self.debug:
                self.get_logger().info(
                    'DEBUG MODE ON — every cmd_vel will be printed')
        except serial.SerialException as e:
            self.get_logger().fatal(
                f'Cannot open {port}: {e}\n'
                f'  Run: ls /dev/ttyUSB*  to find port')
            raise SystemExit(1)

        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    # ── wheel angular velocity → direction + PWM ──────────────

    def _w_to_dir_pwm(self, w: float):
        """
        Convert a wheel angular velocity (rad/s) to (direction, PWM).
        direction: 0=STOP  1=FORWARD  2=BACKWARD
        PWM: 0-255 linearly scaled, minimum min_pwm when moving.
        """
        if abs(w) < 0.01:
            return 0, 0

        direction = 1 if w > 0 else 2
        ratio     = min(abs(w) / self.max_wv, 1.0)
        pwm       = int(self.min_pwm + (255 - self.min_pwm) * ratio)
        pwm       = max(0, min(255, pwm))
        return direction, pwm

    # ── /cmd_vel → mecanum IK → serial command ────────────────

    def _cmd_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # ── Debug: print what Nav2 is commanding ──────────────
        if self.debug:
            self.get_logger().info(
                f'CMD_VEL → vx={vx:.4f}  vy={vy:.4f}  wz={wz:.4f}')

        # Full stop
        if abs(vx) < self.cmd_thr and abs(vy) < self.cmd_thr and abs(wz) < self.cmd_thr:
            with self._cmd_lock:
                self._cmd_str = '0,0,0,0,0,0,0,0\n'
            return

        # ── Mecanum inverse kinematics ─────────────────────────
        #
        # vx and vy are negated here because the robot's physical
        # forward = -X in the odom frame (encoder wiring convention).
        # Negating here means Nav2's positive vx correctly drives
        # the robot physically forward, matching the negated FK above.
        vx_ik = -vx
        vy_ik = -vy

        wFL = (vx_ik - vy_ik - wz * self.wd) / self.r
        wFR = (vx_ik + vy_ik + wz * self.wd) / self.r
        wRR = (vx_ik - vy_ik + wz * self.wd) / self.r
        wRL = (vx_ik + vy_ik - wz * self.wd) / self.r

        wheels = [wFL, wFR, wRR, wRL]

        # Scale all wheels proportionally if any exceeds max_wheel_vel
        max_w = max(abs(w) for w in wheels)
        if max_w > self.max_wv:
            scale  = self.max_wv / max_w
            wheels = [w * scale for w in wheels]

        dirs, pwms = [], []
        for w in wheels:
            d, p = self._w_to_dir_pwm(w)
            dirs.append(d)
            pwms.append(p)

        cmd = (f'{dirs[0]},{dirs[1]},{dirs[2]},{dirs[3]},'
               f'{pwms[0]},{pwms[1]},{pwms[2]},{pwms[3]}\n')

        if self.debug:
            self.get_logger().info(
                f'WHEELS rad/s → FL={wFL:.2f} FR={wFR:.2f} '
                f'RR={wRR:.2f} RL={wRL:.2f}')
            self.get_logger().info(
                f'ESP32 CMD  → {cmd.strip()}')

        with self._cmd_lock:
            self._cmd_str = cmd

    # ── Serial loop: read encoders, integrate odom, send cmd ──

    def _loop(self):
        while self._running:
            # ── Read encoder line from ESP32 ──────────────────
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

            # Skip non-numeric lines (boot messages etc.)
            if not line or not (line[0].isdigit() or line[0] == '-'):
                continue

            parts = line.split(',')
            if len(parts) != 4:
                continue
            try:
                ticks = [int(p) for p in parts]
            except ValueError:
                continue

            # ── Time delta ────────────────────────────────────
            now = self.get_clock().now()
            dt  = (now - self.prev_time).nanoseconds * 1e-9

            if dt <= 0.0 or dt > 1.0:
                self.prev_ticks = ticks
                self.prev_time  = now
                self._send()
                continue

            # ── Wheel displacements [m] ───────────────────────
            # Serial order from ESP32: FL, FR, RR, RL
            d = [(ticks[i] - self.prev_ticks[i]) * self.mpt
                 for i in range(4)]
            self.prev_ticks = ticks
            self.prev_time  = now

            dFL, dFR, dRR, dRL = d[0], d[1], d[2], d[3]

            # ── Mecanum forward kinematics ────────────────────
            #   Δx  = ( FL + FR + RR + RL) / 4
            #   Δy  = (-FL + FR - RR + RL) / 4
            #   Δθ  = (-FL + FR + RR - RL) / (4 × wd)
            #
            # Signs are NEGATED for delta_x and delta_y because the
            # robot's physical forward direction corresponds to -X in
            # the odom frame as wired. Negating aligns physical forward
            # with odom +X so Nav2 navigation works correctly.
            delta_x   = -( dFL + dFR + dRR + dRL) / 4.0
            delta_y   = -(-dFL + dFR - dRR + dRL) / 4.0
            delta_yaw = (-dFL + dFR + dRR - dRL) / (4.0 * self.wd)

            # ── Mid-point RK2 integration ─────────────────────
            yaw_mid   = self.yaw + delta_yaw * 0.5
            self.x   += delta_x * math.cos(yaw_mid) - delta_y * math.sin(yaw_mid)
            self.y   += delta_x * math.sin(yaw_mid) + delta_y * math.cos(yaw_mid)
            self.yaw  = wrap(self.yaw + delta_yaw)

            self.vx     = delta_x   / dt
            self.vy     = delta_y   / dt
            self.wz_vel = delta_yaw / dt

            # ── Publish odom + TF ─────────────────────────────
            self._publish(now)

            # ── Send command to ESP32 ─────────────────────────
            self._send()

    # ── Publish /odom and TF ──────────────────────────────────

    def _publish(self, now_ros):
        stamp           = now_ros.to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id  = self.base_frame

        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance = [
            5e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  5e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  5e-3,
        ]

        msg.twist.twist.linear.x  = self.vx
        msg.twist.twist.linear.y  = self.vy
        msg.twist.twist.linear.z  = 0.0
        msg.twist.twist.angular.z = self.wz_vel

        msg.twist.covariance = [
            5e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  5e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  5e-3,
        ]

        self.odom_pub.publish(msg)

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

    # ── Send command to ESP32 ─────────────────────────────────

    def _send(self):
        with self._cmd_lock:
            s = self._cmd_str
        try:
            self.ser.write(s.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ── Cleanup ───────────────────────────────────────────────

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
