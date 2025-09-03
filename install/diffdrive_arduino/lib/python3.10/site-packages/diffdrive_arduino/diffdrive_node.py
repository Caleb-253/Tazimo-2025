#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf_transformations
import tf2_ros
import serial
import math
import time

class DiffDriveArduino(Node):
    def __init__(self):
        super().__init__('diffdrive_arduino')

        # Declare parameters (with defaults)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('ticks_per_revolution', 620)
        self.declare_parameter('publish_rate', 30.0)  # Hz

        # Load parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value
        self.dt = 1.0 / self.get_parameter('publish_rate').get_parameter_value().double_value

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.initialized = False

        # ROS publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Setup serial
        try:
            self.arduino = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            time.sleep(2)  # wait for Arduino reset
            self.get_logger().info(f"Connected to Arduino on {self.serial_port} at {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.arduino = None

        # Timer for periodic updates
        self.create_timer(self.dt, self.update)

    def cmd_vel_callback(self, msg: Twist):
        """Send velocity commands to Arduino."""
        v = msg.linear.x
        omega = msg.angular.z

        v_left = v - (omega * self.wheel_separation / 2.0)
        v_right = v + (omega * self.wheel_separation / 2.0)

        # Convert linear velocities to ticks per second
        left_cmd = int((v_left / (2 * math.pi * self.wheel_radius)) * self.ticks_per_revolution)
        right_cmd = int((v_right / (2 * math.pi * self.wheel_radius)) * self.ticks_per_revolution)

        if self.arduino:
            cmd_str = f"{left_cmd},{right_cmd}\n"
            self.arduino.write(cmd_str.encode())

    def read_encoders(self):
        """Read encoder ticks from Arduino."""
        if not self.arduino:
            return None, None

        try:
            line = self.arduino.readline().decode('utf-8').strip()
            if line.startswith("E"):
                parts = line.split()
                left_ticks = int(parts[1])
                right_ticks = int(parts[2])
                return left_ticks, right_ticks
        except Exception:
            return None, None
        return None, None

    def update(self):
        """Update odometry, joint states, and publish."""
        left_ticks, right_ticks = self.read_encoders()
        if left_ticks is None or right_ticks is None:
            return

        if not self.initialized:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.initialized = True
            return

        # Delta ticks
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # Distance traveled by each wheel
        dist_left = (2 * math.pi * self.wheel_radius) * (delta_left / self.ticks_per_revolution)
        dist_right = (2 * math.pi * self.wheel_radius) * (delta_right / self.ticks_per_revolution)

        # Robot motion
        delta_s = (dist_left + dist_right) / 2.0
        delta_th = (dist_right - dist_left) / self.wheel_separation

        # Update pose
        self.x += delta_s * math.cos(self.th + delta_th / 2.0)
        self.y += delta_s * math.sin(self.th + delta_th / 2.0)
        self.th += delta_th

        # Create quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)

        # Publish odom message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = delta_s / self.dt
        odom.twist.twist.angular.z = delta_th / self.dt
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # --- Publish wheel joint states ---
        left_angle = 2 * math.pi * (left_ticks / self.ticks_per_revolution)
        right_angle = 2 * math.pi * (right_ticks / self.ticks_per_revolution)
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [left_angle, right_angle]
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
