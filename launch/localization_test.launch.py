#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("hit25_auv_ros2")
    rov_launch_file = os.path.join(package_share, "launch", "rov_start.launch.py")

    setup_delay = LaunchConfiguration("setup_delay")
    odom_topic = LaunchConfiguration("odom_topic")
    path_topic = LaunchConfiguration("path_topic")
    path_frame = LaunchConfiguration("path_frame")
    base_frame = LaunchConfiguration("base_frame")
    min_translation = LaunchConfiguration("min_translation")
    max_poses = LaunchConfiguration("max_poses")
    use_buoy_control = LaunchConfiguration("use_buoy_control")
    buoy_topic = LaunchConfiguration("buoy_topic")
    buoy_arrival_radius = LaunchConfiguration("buoy_arrival_radius")
    use_buoy_z = LaunchConfiguration("use_buoy_z")
    buoy_hold_mode = LaunchConfiguration("buoy_hold_mode")
    buoy_guided_mode = LaunchConfiguration("buoy_guided_mode")
    enable_battery_dynamic_id_server = LaunchConfiguration("enable_battery_dynamic_id_server")

    rov_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rov_launch_file),
        launch_arguments={
            "use_buoy_control": use_buoy_control,
            "buoy_topic": buoy_topic,
            "buoy_arrival_radius": buoy_arrival_radius,
            "use_buoy_z": use_buoy_z,
            "buoy_hold_mode": buoy_hold_mode,
            "buoy_guided_mode": buoy_guided_mode,
            "enable_battery_dynamic_id_server": enable_battery_dynamic_id_server,
        }.items(),
    )

    localization_debug_node = Node(
        package="hit25_auv_ros2",
        executable="localization_debug",
        name="localization_debug_node",
        output="screen",
        parameters=[
            {
                "odom_topic": odom_topic,
                "path_topic": path_topic,
                "path_frame": path_frame,
                "base_frame": base_frame,
                "min_translation": min_translation,
                "max_poses": max_poses,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("setup_delay", default_value="5.0"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("path_topic", default_value="/localization/path"),
            DeclareLaunchArgument("path_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("min_translation", default_value="0.0"),
            DeclareLaunchArgument("max_poses", default_value="5000"),
            DeclareLaunchArgument("use_buoy_control", default_value="false"),
            DeclareLaunchArgument("buoy_topic", default_value="/buoy"),
            DeclareLaunchArgument("buoy_arrival_radius", default_value="0.10"),
            DeclareLaunchArgument("use_buoy_z", default_value="false"),
            DeclareLaunchArgument("buoy_hold_mode", default_value="ALT_HOLD"),
            DeclareLaunchArgument("buoy_guided_mode", default_value="GUIDED"),
            DeclareLaunchArgument("enable_battery_dynamic_id_server", default_value="true"),
            LogInfo(
                msg=[
                    "[localization_test] Starting rov_start. Waiting ",
                    setup_delay,
                    " seconds before localization_debug.",
                ]
            ),
            rov_launch,
            TimerAction(
                period=setup_delay,
                actions=[
                    LogInfo(
                        msg=[
                            "[localization_test] Starting localization_debug. ",
                            "DVL commands are controlled from the web GUI.",
                        ]
                    ),
                    localization_debug_node,
                ],
            ),
        ]
    )
