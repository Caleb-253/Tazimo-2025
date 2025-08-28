#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    topic_name = LaunchConfiguration('topic_name')

    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP210x_USB_to_UART_Bridge_Controller_XXXXXXXX-if00-port0',
            description='Specifying usb port to connect lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Specifying usb port baudrate'),

        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_frame',
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Enable angle compensation'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value='Standard',
            description='Scan mode of lidar'),

        DeclareLaunchArgument(
            'range_min',
            default_value='0.15',
            description='Minimum range of lidar'),

        DeclareLaunchArgument(
            'range_max',
            default_value='12.0',
            description='Maximum range of lidar'),

        DeclareLaunchArgument(
            'topic_name',
            default_value='scan',
            description='Topic name for lidar data'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'range_min': range_min,
                'range_max': range_max,
                'topic_name': topic_name,
            }]
        )
    ])

