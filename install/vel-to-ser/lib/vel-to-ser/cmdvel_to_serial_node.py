#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToSerial(Node):
    def __init__(self):``
        super().__init__('cmdvel_to_serial_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.2)  # meters
        self.declare_parameter('wheel_radius', 0.0325)     # meters

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Opened serial port {port} at {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.ser = None

        # ROS subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdvel_callback,
            10
        )

    def cmdvel_callback(self, msg: Twist):
        if self.ser is None:
            return

        # Extract linear & angular velocities
        linear = msg.linear.x      # m/s
        angular = msg.angular.z    # rad/s

        wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        wheel_r   = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Convert to wheel velocities (rad/s)
        v_left  = (linear - (angular * wheel_sep / 2.0)) / wheel_r
        v_right = (linear + (angular * wheel_sep / 2.0)) / wheel_r

        # Scale to integers for Arduino (e.g., motor driver speed)
        left_cmd  = int(v_left * 6.315789)   # scale factor for your setup
        right_cmd = int(v_right * 6.315789)

        # Create command string
        cmd_str = f"V {left_cmd}, {right_cmd}\n"
        self.ser.write(cmd_str.encode('utf-8'))

        self.get_logger().info(f"Sent: {cmd_str.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
