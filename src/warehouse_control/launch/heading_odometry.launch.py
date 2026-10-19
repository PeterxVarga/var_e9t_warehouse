"""Start opt-in IMU/wheel feedback; controllers remain separate, zero until ready."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'startup_timeout', default_value='10.0',
            description='Wall seconds to wait for fresh advancing IMU, wheel odometry and clock.'),
        Node(
            package='warehouse_control', executable='heading_odometry', output='screen',
            parameters=[{'use_sim_time': True, 'startup_timeout': ParameterValue(
                LaunchConfiguration('startup_timeout'), value_type=float)}]),
    ])
