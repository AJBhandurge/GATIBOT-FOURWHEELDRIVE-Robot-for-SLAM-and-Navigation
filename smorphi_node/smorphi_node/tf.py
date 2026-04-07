#!/usr/bin/env python3
"""
fix_tf.py  —  Run this RIGHT NOW to make the robot appear in RViz.

This publishes the missing TF transforms that robot_state_publisher
should be providing:
  base_footprint → base_link
  base_link      → laser_link

Usage:
    python3 fix_tf.py

Run this in a separate terminal alongside your existing nodes.
The robot should appear in RViz within 2-3 seconds.

PERMANENT FIX: Add robot_state_publisher to your launch file.
See smorphi_bringup.launch.py for the complete solution.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class TfFix(Node):
    def __init__(self):
        super().__init__('smorphi_tf_fix')
        self.br = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self.get_logger().info(
            'Static TF published: base_footprint→base_link→laser_link\n'
            'Robot should now appear in RViz.\n'
            'Keep this node running — static TFs need to be latched.')

    def _make_tf(self, parent, child, x=0.0, y=0.0, z=0.0,
                 roll=0.0, pitch=0.0, yaw=0.0):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        # Convert RPY to quaternion
        cy = math.cos(yaw   * 0.5)
        sy = math.sin(yaw   * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll  * 0.5)
        sr = math.sin(roll  * 0.5)
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        return t

    def _publish_static_transforms(self):
        transforms = [
            # base_footprint → base_link
            # z=0.06 matches URDF base_joint origin
            self._make_tf('base_footprint', 'base_link', z=0.06),

            # base_link → laser_link
            # Lidar mounted at top centre of robot body
            # Adjust x,y if your lidar is offset from centre
            self._make_tf('base_link', 'laser_link', z=0.05),

            # base_link → wheel frames (needed for full robot model)
            self._make_tf('base_link', 'front_left_wheel',
                          x=0.035,  y=0.065,  z=-0.06),
            self._make_tf('base_link', 'front_right_wheel',
                          x=0.035,  y=-0.065, z=-0.06),
            self._make_tf('base_link', 'rear_left_wheel',
                          x=-0.035, y=0.065,  z=-0.06),
            self._make_tf('base_link', 'rear_right_wheel',
                          x=-0.035, y=-0.065, z=-0.06),
        ]
        self.br.sendTransform(transforms)


def main():
    rclpy.init()
    node = TfFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
