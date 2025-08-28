from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('my_bot').find('my_bot')
    
    # Point to the correct URDF file
    urdf_file = PathJoinSubstitution([pkg_share, 'description', 'robot.urdf.xacro'])
    
    # Controller config file
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'my_controllers.yaml'])

    return LaunchDescription([

        # Declare sim_mode arg
        DeclareLaunchArgument("sim_mode", default_value="true"),

        # Robot description publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command([
                    "xacro ",
                    urdf_file,
                    " sim_mode:=",
                    LaunchConfiguration("sim_mode")
                ])
            }]
        ),

        # Controller manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_file],
            output="screen"
        ),

        # Spawners
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"]
        )
    ])
