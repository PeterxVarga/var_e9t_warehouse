"""Launch the installed warehouse world and its directional ROS bridges."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_simulation(context):
    share = Path(get_package_share_directory("warehouse_sim"))
    model_path = str(share / "models")
    resource_path = os.pathsep.join(
        path for path in (model_path, os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""))
        if path
    )
    command = ["ign", "gazebo", "-r", "-v", "3"]
    if IfCondition(LaunchConfiguration("headless")).evaluate(context):
        command.append("-s")
    command.append(str(share / "worlds" / "warehouse.sdf"))

    return [
        ExecuteProcess(
            cmd=command,
            output="screen",
            additional_env={"IGN_GAZEBO_RESOURCE_PATH": resource_path},
            on_exit=Shutdown(reason="Gazebo exited"),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="warehouse_bridge",
            parameters=[{"config_file": str(share / "config" / "bridge.yaml")}],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run only the Gazebo server, without the GUI.",
        ),
        OpaqueFunction(function=launch_simulation),
    ])
