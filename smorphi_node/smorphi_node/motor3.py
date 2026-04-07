#!/usr/bin/env python3
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

        # PARAMETERS
        self.declare_parameter('serial_port',   '/dev/smorphi_mb')
        self.declare_parameter('baud_rate',     115200)
        self.declare_parameter('odom_frame',    'odom')
        self.declare_parameter('base_frame',    'base_footprint')
        self.declare_parameter('publish_tf',    True)

        self.declare_parameter('wheel_radius',  0.03)
        self.declare_parameter('ticks_per_rev', 292.50)
        self.declare_parameter('wheel_base',    0.20)

        self.declare_parameter('max_wheel_vel', 6.0)
        self.declare_parameter('min_motor_pwm', 70)
        self.declare_parameter('cmd_threshold', 0.001)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pub_tf     = self.get_parameter('publish_tf').value

        self.r      = self.get_parameter('wheel_radius').value
        ticks       = self.get_parameter('ticks_per_rev').value
        self.base_w = self.get_parameter('wheel_base').value

        self.max_wv = self.get_parameter('max_wheel_vel').value
        self.min_pwm = self.get_parameter('min_motor_pwm').value
        self.cmd_thr = self.get_parameter('cmd_threshold').value

        # meters per tick
        self.mpt = (2.0 * math.pi * self.r) / ticks

        # ODOM STATE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_ticks = [0,0,0,0]
        self.prev_time = self.get_clock().now()

        # COMMAND
        self._cmd_str = '0,0,0,0,0,0,0,0\n'
        self._cmd_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_br = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos)

        # SERIAL
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self._running = True
        threading.Thread(target=self._serial_loop, daemon=True).start()

    # CMD VEL → DIFFERENTIAL DRIVE IK
    def _cmd_cb(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z

        if abs(vx) < self.cmd_thr and abs(wz) < self.cmd_thr:
            with self._cmd_lock:
                self._cmd_str = '0,0,0,0,0,0,0,0\n'
            return

        # Differential drive IK
        v_left  = vx - wz * (self.base_w / 2.0)
        v_right = vx + wz * (self.base_w / 2.0)

        wFL = v_left  / self.r
        wRL = v_left  / self.r
        wFR = v_right / self.r
        wRR = v_right / self.r

        wheels = [wFL, wFR, wRR, wRL]

        max_w = max(abs(w) for w in wheels)
        if max_w > self.max_wv:
            wheels = [w * (self.max_wv / max_w) for w in wheels]

        dirs, pwms = [], []
        for w in wheels:
            d, p = self._w_to_dp(w)
            dirs.append(d)
            pwms.append(p)

        cmd = f'{dirs[0]},{dirs[1]},{dirs[2]},{dirs[3]},{pwms[0]},{pwms[1]},{pwms[2]},{pwms[3]}\n'

        with self._cmd_lock:
            self._cmd_str = cmd

    def _w_to_dp(self, w: float):
        if abs(w) < 0.01:
            return 0, 0
        direction = 1 if w > 0 else 2
        ratio = min(abs(w) / self.max_wv, 1.0)
        pwm = int(self.min_pwm + (255 - self.min_pwm) * ratio)
        return direction, max(0, min(255, pwm))

    # SERIAL LOOP → DIFFERENTIAL ODOMETRY
    def _serial_loop(self):
        while self._running:
            raw = self.ser.readline()
            if not raw:
                continue

            line = raw.decode('ascii', errors='ignore').strip()
            parts = line.split(',')
            if len(parts) != 4:
                continue

            ticks = [int(p) for p in parts]

            now = self.get_clock().now()
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt <= 0:
                continue

            d = [(ticks[i] - self.prev_ticks[i]) * self.mpt for i in range(4)]
            self.prev_ticks = ticks
            self.prev_time = now

            dFL, dFR, dRR, dRL = d

            # Differential drive FK
            d_left  = (dFL + dRL) / 2.0
            d_right = (dFR + dRR) / 2.0

            delta_s = (d_left + d_right) / 2.0
            delta_yaw = (d_right - d_left) / self.base_w

            yaw_mid = self.yaw + delta_yaw * 0.5

            self.x += delta_s * math.cos(yaw_mid)
            self.y += delta_s * math.sin(yaw_mid)
            self.yaw = wrap(self.yaw + delta_yaw)

            self._publish(now)
            self._send()

    def _publish(self, now_ros):
        stamp = now_ros.to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        if self.pub_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_br.sendTransform(tf)

    def _send(self):
        with self._cmd_lock:
            s = self._cmd_str
        self.ser.write(s.encode('ascii'))

    def destroy_node(self):
        self._running = False
        if self.ser.is_open:
            self.ser.write(b'0,0,0,0,0,0,0,0\n')
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
