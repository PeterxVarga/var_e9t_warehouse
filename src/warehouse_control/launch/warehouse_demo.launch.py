"""Launch the existing warehouse simulation, IMU feedback, and mission in one session.

This demo uses an ideal simulated IMU and manual logical aisle closures.
"""

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _included_launch(package, filename, arguments):
    source = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare(package), 'launch', filename,
    ]))
    return IncludeLaunchDescription(source, launch_arguments=arguments.items())


def _mission_node(context):
    raw = LaunchConfiguration('pickup_nodes').perform(context)
    try:
        pickups = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError('pickup_nodes must be a JSON string array') from exc
    if not isinstance(pickups, list) or any(type(node) is not str for node in pickups):
        raise ValueError('pickup_nodes must be a JSON string array')
    strategy = LaunchConfiguration('strategy').perform(context)
    if strategy not in ('fifo', 'nn', 'nn-2opt'):
        raise ValueError('strategy must be fifo, nn, or nn-2opt')
    return [Node(
        package='warehouse_control',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pickup_nodes': pickups,
            'strategy': strategy,
        }],
        remappings=[('/warehouse/odom', '/warehouse/imu_odom')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true',
                              description='Run Gazebo without its GUI.'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Show the existing RViz mission-plan view.'),
        DeclareLaunchArgument('startup_timeout', default_value='30.0',
                              description='Estimator startup deadline in wall seconds.'),
        DeclareLaunchArgument('strategy', default_value='fifo',
                              description='Mission pickup order: fifo, nn, nn-2opt.'),
        DeclareLaunchArgument('pickup_nodes',
                              default_value='["r1c0","r1c0","r0c0"]',
                              description='JSON string array of original pickup requests.'),
        OpaqueFunction(function=_mission_node),
        _included_launch('warehouse_sim', 'simulation.launch.py', {
            'headless': LaunchConfiguration('headless'),
        }),
        _included_launch('warehouse_control', 'heading_odometry.launch.py', {
            'startup_timeout': LaunchConfiguration('startup_timeout'),
        }),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('warehouse_control'), 'rviz', 'mission_plans.rviz',
            ])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
