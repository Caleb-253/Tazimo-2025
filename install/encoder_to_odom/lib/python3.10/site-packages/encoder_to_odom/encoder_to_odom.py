#!/usr/bin/env python3
import math
import re
import serial
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def yaw_to_quat(z):
    """Return (x,y,z,w) quaternion for yaw angle z (rad)."""
    s = math.sin(z * 0.5)
    c = math.cos(z * 0.5)
    return (0.0, 0.0, s, c)


class EncoderToOdom(Node):
    def __init__(self):
        super().__init__('encoder_to_odom')

        # ---- Parameters (all override-able via launch/YAML) ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('ticks_per_rev', 1440)       # set to your encoder CPR
        self.declare_parameter('wheel_radius', 0.033)       # meters
        self.declare_parameter('wheel_base', 0.160)         # meters (axle length)
        self.declare_parameter('left_inverted', False)
        self.declare_parameter('right_inverted', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.tpr  = float(self.get_parameter('ticks_per_rev').value)
        self.r    = float(self.get_parameter('wheel_radius').value)
        self.L    = float(self.get_parameter('wheel_base').value)
        self.left_inv  = bool(self.get_parameter('left_inverted').value)
        self.right_inv = bool(self.get_parameter('right_inverted').value)
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_left = None   # last absolute tick counts
        self.last_right = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Serial reader thread (non-blocking)
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            raise

        self._stop = threading.Event()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # Accepts any of these line formats from Arduino (absolute counts):
    #   "L:123 R:456"
    #   "123,456"
    #   "123 456"
    #   "L=123,R=456"
    def parse_line(self, line):
        line = line.strip()
        if not line:
            return None, None
        # Try labeled
        m = re.search(r'[-+]?\d+', line)
        nums = [int(n) for n in re.findall(r'[-+]?\d+', line)]
        if len(nums) >= 2:
            return nums[0], nums[1]
        return None, None

    def read_loop(self):
        while not self._stop.is_set():
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore')
            except Exception:
                continue
            left, right = self.parse_line(raw)
            if left is None or right is None:
                continue
            if self.left_inv:
                left = -left
            if self.right_inv:
                right = -right
            self.update_odometry(left, right)

    def update_odometry(self, left_ticks, right_ticks):
        now = self.get_clock().now()

        if self.last_left is None or self.last_right is None:
            self.last_left = left_ticks
            self.last_right = right_ticks
            return

        d_left_ticks  = left_ticks  - self.last_left
        d_right_ticks = right_ticks - self.last_right
        self.last_left = left_ticks
        self.last_right = right_ticks

        # ticks -> meters
        meters_per_tick = (2.0 * math.pi * self.r) / self.tpr
        dl = d_left_ticks  * meters_per_tick
        dr = d_right_ticks * meters_per_tick

        d = (dr + dl) * 0.5
        dth = (dr - dl) / self.L

        # integrate (runge-kutta midpoint)
        self.x += d * math.cos(self.th + 0.5 * dth)
        self.y += d * math.sin(self.th + 0.5 * dth)
        self.th += dth

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quat(self.th)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # (Optional) simple covariance — tune if needed
        odom.pose.covariance[0] = 1e-3
        odom.pose.covariance[7] = 1e-3
        odom.pose.covariance[35] = 1e-2

        self.odom_pub.publish(odom)

        # Publish TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._stop.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = EncoderToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
