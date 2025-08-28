from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encoder_to_odom',
            executable='encoder_to_odom',
            name='encoder_to_odom',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',     # change if needed
                'baud': 57600,
                'ticks_per_rev': 3436,      # <-- set yours
                'wheel_radius': 0.0325,      # meters
                'wheel_base': 0.04,        # meters
                'left_inverted': False,
                'right_inverted': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }]
        )
    ])
