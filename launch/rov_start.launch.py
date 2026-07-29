#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_launch_file(package_name: str, relative_path: str) -> str:
    try:
        return os.path.join(get_package_share_directory(package_name), relative_path)
    except PackageNotFoundError:
        return ""


def _geographiclib_geoid_path() -> str:
    configured = os.environ.get("GEOGRAPHICLIB_GEOID_PATH", "")
    if configured:
        return configured

    candidates = [
        os.path.expanduser("~/.local/share/GeographicLib/geoids"),
        "/usr/share/GeographicLib/geoids",
        "/usr/local/share/GeographicLib/geoids",
    ]
    for candidate in candidates:
        if os.path.isfile(os.path.join(candidate, "egm96-5.pgm")):
            return candidate
    return ""


def _static_tf_node(name, parent_frame, child_frame, x, y, z, roll, pitch, yaw) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x", x,
            "--y", y,
            "--z", z,
            "--roll", roll,
            "--pitch", pitch,
            "--yaw", yaw,
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )


def generate_launch_description() -> LaunchDescription:
    dvl_default = _default_launch_file(
        "auv_dvl_a50", os.path.join("launch", "dvl_a50.launch.py")
    )
    mavros_default = _default_launch_file(
        "auv", os.path.join("launch", "mavros_auv.launch")
    )
    localization_default = os.path.join(
        get_package_share_directory("auv"), "config", "auv_ekf.yaml")
    dronecan_python_default = os.path.expanduser("~/miniconda3/envs/auv_ros2/bin/python")
    if not os.path.exists(dronecan_python_default):
        dronecan_python_default = "python3"
    geoid_path = _geographiclib_geoid_path()

    launch_arguments = [
        # Core connections
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:57600"),
        DeclareLaunchArgument("mavros_launch_file", default_value=mavros_default),
        DeclareLaunchArgument("configure_mavros_imu_rate", default_value="true"),
        DeclareLaunchArgument("mavros_imu_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("mavros_raw_imu_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("mavros_baro_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("dronecan_python", default_value=dronecan_python_default),
        DeclareLaunchArgument("use_external_baro_bridge", default_value="false"),
        DeclareLaunchArgument(
            "external_baro_connection_url",
            default_value="udpin:0.0.0.0:14500",
        ),
        # Frames
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("fcu_frame", default_value="fcu_link"),
        DeclareLaunchArgument("dvl_frame", default_value="dvl_link"),
        DeclareLaunchArgument("depth_frame", default_value="depth_link"),
        DeclareLaunchArgument("imu_frame", default_value="imu_link"),
        # base_link -> fcu_link (FCU/IMU) static TF
        DeclareLaunchArgument("base_to_fcu_x", default_value="0.13135"),
        DeclareLaunchArgument("base_to_fcu_y", default_value="0.0"),
        DeclareLaunchArgument("base_to_fcu_z", default_value="0.08541"),
        DeclareLaunchArgument("base_to_fcu_roll", default_value="0.0"),
        DeclareLaunchArgument("base_to_fcu_pitch", default_value="0.0"),
        DeclareLaunchArgument("base_to_fcu_yaw", default_value="0.0"),
        # base_link -> DVL static TF
        DeclareLaunchArgument("dvl_x", default_value="-0.00488"),
        DeclareLaunchArgument("dvl_y", default_value="0.0"),
        DeclareLaunchArgument("dvl_z", default_value="-0.03910"),
        DeclareLaunchArgument("dvl_roll", default_value="3.141592653589793"),
        DeclareLaunchArgument("dvl_pitch", default_value="0.0"),
        DeclareLaunchArgument("dvl_yaw", default_value="0.0"),
        # base_link -> depth static TF
        DeclareLaunchArgument("depth_x", default_value="-0.17364"),
        DeclareLaunchArgument("depth_y", default_value="-0.03034"),
        DeclareLaunchArgument("depth_z", default_value="0.0536"),
        DeclareLaunchArgument("depth_roll", default_value="0.0"),
        DeclareLaunchArgument("depth_pitch", default_value="0.0"),
        DeclareLaunchArgument("depth_yaw", default_value="0.0"),
        # fcu_link -> imu_link static TF
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),
        DeclareLaunchArgument("dvl_ip", default_value="192.168.194.95"),
        DeclareLaunchArgument("use_dvl", default_value="true"),
        DeclareLaunchArgument("dvl_launch_file", default_value=dvl_default),
        DeclareLaunchArgument("configure_dvl_acoustic_on_startup", default_value="true"),
        DeclareLaunchArgument("dvl_startup_acoustic_enabled", default_value="true"),
        DeclareLaunchArgument("request_dvl_config_on_startup", default_value="true"),
        DeclareLaunchArgument("use_localization", default_value="true"),
        DeclareLaunchArgument("localization_params_file", default_value=localization_default),
        DeclareLaunchArgument("dvl_twist_min_linear_variance", default_value="0.0"),
        DeclareLaunchArgument("dvl_twist_max_linear_variance", default_value="1.0"),
        DeclareLaunchArgument("dvl_twist_covariance_scale", default_value="1.0"),
        DeclareLaunchArgument("dvl_twist_max_fom", default_value="0.05"),
        DeclareLaunchArgument("dvl_twist_min_altitude", default_value="0.05"),
        DeclareLaunchArgument("dvl_twist_min_valid_beams", default_value="3"),
        DeclareLaunchArgument("dvl_twist_reacquire_good_samples", default_value="1"),
        DeclareLaunchArgument("dvl_twist_reacquire_duration", default_value="0.0"),
        DeclareLaunchArgument("dvl_twist_max_velocity", default_value="0.8"),
        DeclareLaunchArgument("dvl_twist_max_acceleration", default_value="1.0"),
        DeclareLaunchArgument("dvl_twist_jump_tolerance", default_value="0.03"),
        DeclareLaunchArgument("dvl_twist_max_rate_dt", default_value="0.5"),
        DeclareLaunchArgument("dvl_twist_recovery_trigger_gap", default_value="0.25"),
        DeclareLaunchArgument("dvl_twist_recovery_initial_variance", default_value="1.0"),
        DeclareLaunchArgument("dvl_twist_recovery_variance_decay", default_value="0.8"),
        DeclareLaunchArgument("dvl_twist_recovery_variance_samples", default_value="56"),
        DeclareLaunchArgument("use_dvl_position_odom", default_value="true"),
        DeclareLaunchArgument("dvl_position_odom_topic", default_value="/dvl/odometry"),
        DeclareLaunchArgument("dvl_position_frame", default_value="dvl_odom"),
        DeclareLaunchArgument("dvl_position_child_frame", default_value="dvl_link"),
        DeclareLaunchArgument("dvl_position_zero_on_start", default_value="false"),
        DeclareLaunchArgument("dvl_position_zero_orientation_on_start", default_value="false"),
        DeclareLaunchArgument("dvl_position_orientation_in_degrees", default_value="true"),
        DeclareLaunchArgument("dvl_position_estimate_twist", default_value="false"),
        DeclareLaunchArgument("dvl_position_use_pos_std", default_value="true"),
        DeclareLaunchArgument("dvl_position_variance_xy", default_value="0.25"),
        DeclareLaunchArgument("dvl_position_variance_z", default_value="100.0"),
        DeclareLaunchArgument("dvl_position_max_norm", default_value="0.0"),
        DeclareLaunchArgument("dvl_position_max_speed", default_value="0.0"),
        DeclareLaunchArgument("dvl_position_reset_origin_on_jump", default_value="false"),
        DeclareLaunchArgument("pressure_topic", default_value="/mavros/imu/static_pressure"),
        DeclareLaunchArgument("pressure_input_mode", default_value="pressure_pa"),
        DeclareLaunchArgument("fluid_density", default_value="1000.0"),
        DeclareLaunchArgument("enable_depth_gate", default_value="true"),
        DeclareLaunchArgument("depth_gate_max_vertical_speed", default_value="2.0"),
        DeclareLaunchArgument("depth_gate_jump_tolerance", default_value="0.10"),
        DeclareLaunchArgument("depth_gate_max_rate_dt", default_value="0.5"),
        DeclareLaunchArgument("depth_gate_reacquire_good_samples", default_value="3"),
        DeclareLaunchArgument("depth_gate_reacquire_variance_samples", default_value="3"),
        DeclareLaunchArgument("depth_gate_reacquire_variance_scale", default_value="10.0"),
        DeclareLaunchArgument("joy_axis_deadzone", default_value="0.08"),
        DeclareLaunchArgument("joy_vertical_axis_deadzone", default_value="0.10"),
        DeclareLaunchArgument("joy_pwm_range", default_value="300.0"),
        DeclareLaunchArgument("alt_hold_entry_neutral_sec", default_value="1.0"),
        DeclareLaunchArgument("alt_hold_post_entry_neutral_sec", default_value="0.3"),
        DeclareLaunchArgument("use_buoy_control", default_value="false"),
        DeclareLaunchArgument("buoy_topic", default_value="/buoy"),
        DeclareLaunchArgument("buoy_arrival_radius", default_value="0.10"),
        DeclareLaunchArgument("use_buoy_z", default_value="false"),
        DeclareLaunchArgument("buoy_hold_mode", default_value="ALT_HOLD"),
        DeclareLaunchArgument("buoy_guided_mode", default_value="GUIDED"),
        DeclareLaunchArgument("enable_battery_dynamic_id_server", default_value="true"),
    ]

    fcu_url = LaunchConfiguration("fcu_url")
    dvl_ip = LaunchConfiguration("dvl_ip")
    use_dvl = LaunchConfiguration("use_dvl")
    dvl_launch_file = LaunchConfiguration("dvl_launch_file")
    configure_dvl_acoustic_on_startup = LaunchConfiguration("configure_dvl_acoustic_on_startup")
    dvl_startup_acoustic_enabled = LaunchConfiguration("dvl_startup_acoustic_enabled")
    request_dvl_config_on_startup = LaunchConfiguration("request_dvl_config_on_startup")
    use_localization = LaunchConfiguration("use_localization")
    localization_params_file = LaunchConfiguration("localization_params_file")
    dvl_twist_min_linear_variance = LaunchConfiguration("dvl_twist_min_linear_variance")
    dvl_twist_max_linear_variance = LaunchConfiguration("dvl_twist_max_linear_variance")
    dvl_twist_covariance_scale = LaunchConfiguration("dvl_twist_covariance_scale")
    dvl_twist_max_fom = LaunchConfiguration("dvl_twist_max_fom")
    dvl_twist_min_altitude = LaunchConfiguration("dvl_twist_min_altitude")
    dvl_twist_min_valid_beams = LaunchConfiguration("dvl_twist_min_valid_beams")
    dvl_twist_reacquire_good_samples = LaunchConfiguration("dvl_twist_reacquire_good_samples")
    dvl_twist_reacquire_duration = LaunchConfiguration("dvl_twist_reacquire_duration")
    dvl_twist_max_velocity = LaunchConfiguration("dvl_twist_max_velocity")
    dvl_twist_max_acceleration = LaunchConfiguration("dvl_twist_max_acceleration")
    dvl_twist_jump_tolerance = LaunchConfiguration("dvl_twist_jump_tolerance")
    dvl_twist_max_rate_dt = LaunchConfiguration("dvl_twist_max_rate_dt")
    dvl_twist_recovery_trigger_gap = LaunchConfiguration(
        "dvl_twist_recovery_trigger_gap")
    dvl_twist_recovery_initial_variance = LaunchConfiguration(
        "dvl_twist_recovery_initial_variance")
    dvl_twist_recovery_variance_decay = LaunchConfiguration(
        "dvl_twist_recovery_variance_decay")
    dvl_twist_recovery_variance_samples = LaunchConfiguration(
        "dvl_twist_recovery_variance_samples")
    use_dvl_position_odom = LaunchConfiguration("use_dvl_position_odom")
    dvl_position_odom_topic = LaunchConfiguration("dvl_position_odom_topic")
    dvl_position_frame = LaunchConfiguration("dvl_position_frame")
    dvl_position_child_frame = LaunchConfiguration("dvl_position_child_frame")
    dvl_position_zero_on_start = LaunchConfiguration("dvl_position_zero_on_start")
    dvl_position_zero_orientation_on_start = LaunchConfiguration(
        "dvl_position_zero_orientation_on_start")
    dvl_position_orientation_in_degrees = LaunchConfiguration(
        "dvl_position_orientation_in_degrees")
    dvl_position_estimate_twist = LaunchConfiguration("dvl_position_estimate_twist")
    dvl_position_use_pos_std = LaunchConfiguration("dvl_position_use_pos_std")
    dvl_position_variance_xy = LaunchConfiguration("dvl_position_variance_xy")
    dvl_position_variance_z = LaunchConfiguration("dvl_position_variance_z")
    dvl_position_max_norm = LaunchConfiguration("dvl_position_max_norm")
    dvl_position_max_speed = LaunchConfiguration("dvl_position_max_speed")
    dvl_position_reset_origin_on_jump = LaunchConfiguration(
        "dvl_position_reset_origin_on_jump")
    pressure_topic = LaunchConfiguration("pressure_topic")
    pressure_input_mode = LaunchConfiguration("pressure_input_mode")
    fluid_density = LaunchConfiguration("fluid_density")
    enable_depth_gate = LaunchConfiguration("enable_depth_gate")
    depth_gate_max_vertical_speed = LaunchConfiguration("depth_gate_max_vertical_speed")
    depth_gate_jump_tolerance = LaunchConfiguration("depth_gate_jump_tolerance")
    depth_gate_max_rate_dt = LaunchConfiguration("depth_gate_max_rate_dt")
    depth_gate_reacquire_good_samples = LaunchConfiguration(
        "depth_gate_reacquire_good_samples")
    depth_gate_reacquire_variance_samples = LaunchConfiguration(
        "depth_gate_reacquire_variance_samples")
    depth_gate_reacquire_variance_scale = LaunchConfiguration(
        "depth_gate_reacquire_variance_scale")
    joy_axis_deadzone = LaunchConfiguration("joy_axis_deadzone")
    joy_vertical_axis_deadzone = LaunchConfiguration("joy_vertical_axis_deadzone")
    joy_pwm_range = LaunchConfiguration("joy_pwm_range")
    alt_hold_entry_neutral_sec = LaunchConfiguration("alt_hold_entry_neutral_sec")
    alt_hold_post_entry_neutral_sec = LaunchConfiguration("alt_hold_post_entry_neutral_sec")
    mavros_launch_file = LaunchConfiguration("mavros_launch_file")
    configure_mavros_imu_rate = LaunchConfiguration("configure_mavros_imu_rate")
    mavros_imu_rate_hz = LaunchConfiguration("mavros_imu_rate_hz")
    mavros_raw_imu_rate_hz = LaunchConfiguration("mavros_raw_imu_rate_hz")
    mavros_baro_rate_hz = LaunchConfiguration("mavros_baro_rate_hz")
    dronecan_python = LaunchConfiguration("dronecan_python")
    use_external_baro_bridge = LaunchConfiguration("use_external_baro_bridge")
    external_baro_connection_url = LaunchConfiguration("external_baro_connection_url")
    use_buoy_control = LaunchConfiguration("use_buoy_control")
    buoy_topic = LaunchConfiguration("buoy_topic")
    buoy_arrival_radius = LaunchConfiguration("buoy_arrival_radius")
    use_buoy_z = LaunchConfiguration("use_buoy_z")
    buoy_hold_mode = LaunchConfiguration("buoy_hold_mode")
    buoy_guided_mode = LaunchConfiguration("buoy_guided_mode")
    enable_battery_dynamic_id_server = LaunchConfiguration("enable_battery_dynamic_id_server")

    base_frame = LaunchConfiguration("base_frame")
    fcu_frame = LaunchConfiguration("fcu_frame")
    dvl_frame = LaunchConfiguration("dvl_frame")
    depth_frame = LaunchConfiguration("depth_frame")
    imu_frame = LaunchConfiguration("imu_frame")

    base_to_fcu_x = LaunchConfiguration("base_to_fcu_x")
    base_to_fcu_y = LaunchConfiguration("base_to_fcu_y")
    base_to_fcu_z = LaunchConfiguration("base_to_fcu_z")
    base_to_fcu_roll = LaunchConfiguration("base_to_fcu_roll")
    base_to_fcu_pitch = LaunchConfiguration("base_to_fcu_pitch")
    base_to_fcu_yaw = LaunchConfiguration("base_to_fcu_yaw")

    dvl_x = LaunchConfiguration("dvl_x")
    dvl_y = LaunchConfiguration("dvl_y")
    dvl_z = LaunchConfiguration("dvl_z")
    dvl_roll = LaunchConfiguration("dvl_roll")
    dvl_pitch = LaunchConfiguration("dvl_pitch")
    dvl_yaw = LaunchConfiguration("dvl_yaw")

    depth_x = LaunchConfiguration("depth_x")
    depth_y = LaunchConfiguration("depth_y")
    depth_z = LaunchConfiguration("depth_z")
    depth_roll = LaunchConfiguration("depth_roll")
    depth_pitch = LaunchConfiguration("depth_pitch")
    depth_yaw = LaunchConfiguration("depth_yaw")

    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_yaw = LaunchConfiguration("imu_yaw")

    dvl_enabled = IfCondition(
        PythonExpression(["'", use_dvl, "' == 'true' and '", dvl_launch_file, "' != ''"]))
    localization_enabled = IfCondition(PythonExpression(["'", use_localization, "' == 'true'"]))
    dvl_position_odom_enabled = IfCondition(
        PythonExpression([
            "'", use_localization, "' == 'true' and '",
            use_dvl, "' == 'true' and '",
            use_dvl_position_odom, "' == 'true'",
        ]))
    mavros_enabled = IfCondition(PythonExpression(["'", mavros_launch_file, "' != ''"]))
    mavros_imu_rate_config_enabled = IfCondition(
        PythonExpression([
            "'", mavros_launch_file, "' != '' and '",
            configure_mavros_imu_rate, "' == 'true'",
        ]))
    buoy_control_enabled = IfCondition(PythonExpression(["'", use_buoy_control, "' == 'true'"]))

    environment_actions = []
    if geoid_path:
        environment_actions.extend(
            [
                SetEnvironmentVariable(
                    name="GEOGRAPHICLIB_GEOID_PATH",
                    value=geoid_path,
                ),
                LogInfo(msg=f"[rov_start] GeographicLib geoid path: {geoid_path}"),
            ]
        )

    launch_actions = [
        # LogInfo(
        #     condition=IfCondition(
        #         PythonExpression(
        #             ["'", use_dvl, "' == 'true' and '", dvl_launch_file, "' == ''"]
        #         )
        #     ),
        #     msg="[rov_start] DVL launch file not found. Skipping DVL include.",
        # ),
        LogInfo(
            condition=IfCondition(PythonExpression(["'", mavros_launch_file, "' == ''"])),
            msg="[rov_start] MAVROS launch file not found. Skipping MAVROS include.",
        ),
        LogInfo(
            condition=IfCondition(
                PythonExpression(["'", use_dvl, "' == 'true' and '", dvl_launch_file, "' == ''"])
            ),
            msg="[rov_start] DVL launch file not found. Skipping DVL include.",
        ),
        # 1) DVL
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(dvl_launch_file),
            launch_arguments={
                "ip_address": dvl_ip,
                "velocity_frame_id": dvl_frame,
                "position_frame_id": dvl_frame,
                "configure_acoustic_on_startup": configure_dvl_acoustic_on_startup,
                "startup_acoustic_enabled": dvl_startup_acoustic_enabled,
                "request_config_on_startup": request_dvl_config_on_startup,
            }.items(),
            condition=dvl_enabled,
        ),
        # 2) MAVROS
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch_file),
            launch_arguments={"fcu_url": fcu_url}.items(),
            condition=mavros_enabled,
        ),
        Node(
            package="auv",
            executable="mavros_imu_rate_config.py",
            name="mavros_imu_rate_config",
            output="screen",
            parameters=[
                {
                    "attitude_rate_hz": ParameterValue(
                        mavros_imu_rate_hz, value_type=float),
                    "raw_imu_rate_hz": ParameterValue(
                        mavros_raw_imu_rate_hz, value_type=float),
                    "baro_rate_hz": ParameterValue(
                        mavros_baro_rate_hz, value_type=float),
                }
            ],
            condition=mavros_imu_rate_config_enabled,
        ),
        # 3) Vehicle and sensor static TFs
        _static_tf_node(
            "base_to_fcu_static_tf",
            base_frame,
            fcu_frame,
            base_to_fcu_x,
            base_to_fcu_y,
            base_to_fcu_z,
            base_to_fcu_roll,
            base_to_fcu_pitch,
            base_to_fcu_yaw,
        ),
        _static_tf_node(
            "dvl_static_tf",
            base_frame,
            dvl_frame,
            dvl_x,
            dvl_y,
            dvl_z,
            dvl_roll,
            dvl_pitch,
            dvl_yaw,
        ),
        _static_tf_node(
            "depth_static_tf",
            base_frame,
            depth_frame,
            depth_x,
            depth_y,
            depth_z,
            depth_roll,
            depth_pitch,
            depth_yaw,
        ),
        _static_tf_node(
            "imu_static_tf",
            fcu_frame,
            imu_frame,
            imu_x,
            imu_y,
            imu_z,
            imu_roll,
            imu_pitch,
            imu_yaw,
        ),
        # 4) AUV nodes in this package
        Node(
            package="auv",
            executable="joy2mavros",
            name="joy2mavros",
            output="screen",
            respawn=True,
            parameters=[
                {
                    "axis_deadzone": ParameterValue(joy_axis_deadzone, value_type=float),
                    "vertical_axis_deadzone": ParameterValue(
                        joy_vertical_axis_deadzone, value_type=float),
                    "pwm_range": ParameterValue(joy_pwm_range, value_type=float),
                    "alt_hold_entry_neutral_sec": ParameterValue(
                        alt_hold_entry_neutral_sec, value_type=float),
                    "alt_hold_post_entry_neutral_sec": ParameterValue(
                        alt_hold_post_entry_neutral_sec, value_type=float),
                }
            ],
        ),
        Node(
            package="auv",
            executable="dvl_to_twist_bridge",
            name="dvl_to_twist_bridge",
            output="screen",
            respawn=True,
            parameters=[
                {"output_frame_id": dvl_frame},
                {
                    "min_linear_variance": ParameterValue(
                        dvl_twist_min_linear_variance, value_type=float),
                    "max_linear_variance": ParameterValue(
                        dvl_twist_max_linear_variance, value_type=float),
                    "covariance_scale": ParameterValue(
                        dvl_twist_covariance_scale, value_type=float),
                    "max_fom": ParameterValue(dvl_twist_max_fom, value_type=float),
                    "min_altitude": ParameterValue(
                        dvl_twist_min_altitude, value_type=float),
                    "min_valid_beams": ParameterValue(
                        dvl_twist_min_valid_beams, value_type=int),
                    "reacquire_good_samples": ParameterValue(
                        dvl_twist_reacquire_good_samples, value_type=int),
                    "reacquire_duration_s": ParameterValue(
                        dvl_twist_reacquire_duration, value_type=float),
                    "max_velocity_mps": ParameterValue(
                        dvl_twist_max_velocity, value_type=float),
                    "max_acceleration_mps2": ParameterValue(
                        dvl_twist_max_acceleration, value_type=float),
                    "velocity_jump_tolerance_mps": ParameterValue(
                        dvl_twist_jump_tolerance, value_type=float),
                    "max_rate_dt_s": ParameterValue(
                        dvl_twist_max_rate_dt, value_type=float),
                    "recovery_trigger_gap_s": ParameterValue(
                        dvl_twist_recovery_trigger_gap, value_type=float),
                    "recovery_initial_variance": ParameterValue(
                        dvl_twist_recovery_initial_variance, value_type=float),
                    "recovery_variance_decay": ParameterValue(
                        dvl_twist_recovery_variance_decay, value_type=float),
                    "recovery_variance_samples": ParameterValue(
                        dvl_twist_recovery_variance_samples, value_type=int),
                },
            ],
            condition=localization_enabled,
        ),
        Node(
            package="auv",
            executable="dvl_position_to_odom_bridge",
            name="dvl_position_to_odom_bridge",
            output="screen",
            respawn=True,
            parameters=[
                {
                    "output_topic": dvl_position_odom_topic,
                    "frame_id": dvl_position_frame,
                    "child_frame_id": dvl_position_child_frame,
                    "zero_position_on_start": ParameterValue(
                        dvl_position_zero_on_start, value_type=bool),
                    "zero_orientation_on_start": ParameterValue(
                        dvl_position_zero_orientation_on_start, value_type=bool),
                    "orientation_in_degrees": ParameterValue(
                        dvl_position_orientation_in_degrees, value_type=bool),
                    "estimate_twist": ParameterValue(
                        dvl_position_estimate_twist, value_type=bool),
                    "use_dvl_position_stddev": ParameterValue(
                        dvl_position_use_pos_std, value_type=bool),
                    "position_variance_xy": ParameterValue(
                        dvl_position_variance_xy, value_type=float),
                    "position_variance_z": ParameterValue(
                        dvl_position_variance_z, value_type=float),
                    "max_position_norm": ParameterValue(
                        dvl_position_max_norm, value_type=float),
                    "max_position_speed": ParameterValue(
                        dvl_position_max_speed, value_type=float),
                    "reset_origin_on_jump": ParameterValue(
                        dvl_position_reset_origin_on_jump, value_type=bool),
                },
            ],
            condition=dvl_position_odom_enabled,
        ),
        Node(
            package="auv",
            executable="pressure_to_depth_pose",
            name="pressure_to_depth_pose",
            output="screen",
            respawn=True,
            parameters=[
                {"input_topic": pressure_topic},
                {"input_mode": pressure_input_mode},
                {"world_frame": "odom"},
                {"fluid_density": ParameterValue(fluid_density, value_type=float)},
                {"enable_depth_gate": ParameterValue(enable_depth_gate, value_type=bool)},
                {
                    "max_vertical_speed_mps": ParameterValue(
                        depth_gate_max_vertical_speed, value_type=float)
                },
                {
                    "jump_tolerance_m": ParameterValue(
                        depth_gate_jump_tolerance, value_type=float)
                },
                {
                    "max_rate_dt_s": ParameterValue(
                        depth_gate_max_rate_dt, value_type=float)
                },
                {
                    "reacquire_good_samples": ParameterValue(
                        depth_gate_reacquire_good_samples, value_type=int)
                },
                {
                    "reacquire_variance_samples": ParameterValue(
                        depth_gate_reacquire_variance_samples, value_type=int)
                },
                {
                    "reacquire_variance_scale": ParameterValue(
                        depth_gate_reacquire_variance_scale, value_type=float)
                },
            ],
            condition=localization_enabled,
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            respawn=True,
            parameters=[localization_params_file],
            condition=localization_enabled,
        ),
        Node(
            package="auv",
            executable="odom2mavros",
            name="odom2mavros",
            output="screen",
            respawn=True,
        ),
        Node(
            package="auv",
            executable="buoy_position_control",
            name="buoy_position_control",
            output="screen",
            respawn=True,
            parameters=[
                {"odom_topic": "/odometry/filtered"},
                {"buoy_topic": buoy_topic},
                {"arrival_radius": ParameterValue(buoy_arrival_radius, value_type=float)},
                {"use_buoy_z": ParameterValue(use_buoy_z, value_type=bool)},
                {"hold_mode": buoy_hold_mode},
                {"guided_mode": buoy_guided_mode},
            ],
            condition=buoy_control_enabled,
        ),
        # 5) DroneCAN battery bridge
        Node(
            package="auv",
            executable="dronecan2mavros_battery_v2.py",
            name="dronecan2mavros_battery",
            output="screen",
            respawn=True,
            prefix=dronecan_python,
            parameters=[
                {
                    "enable_dynamic_id_server": ParameterValue(
                        enable_battery_dynamic_id_server,
                        value_type=bool,
                    )
                }
            ],
        ),
    ]

    return LaunchDescription(launch_arguments + environment_actions + launch_actions)
