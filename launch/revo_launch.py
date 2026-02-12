#!/usr/bin/env python3

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    params = {
        "serial": "",
        "depth_profile": "",
        "rgb_profile": "",
        "depth_frame_id": "revo_depth_optical_frame",
        "rgb_frame_id": "revo_rgb_optical_frame",
        "depth_camera_info_url": "",
        "rgb_camera_info_url": "",
        "depth_auto_exposure": False,
        "depth_exposure_us": 3000.0,
        "depth_frame_time_us": 7500.0,
        "depth_gain": 1.0,
        "trigger_mode": 0,
        "depth_rgb_match_threshold": 0,
        "depth_rgb_match_rgb_offset": 0,
        "depth_rgb_match_force_rgb_after_depth": False,
        "initial_reset": False,
        "clear_frame_buffer": False,
        "laser_enable": True,
        "laser_luminance": 100,
        "ir_led_enable": False,
        "ir_led_luminance": 0,
        "rgb_led_mode": "disable",
        "depth_range_min_mm": 50,
        "depth_range_max_mm": 2000,
        "algorithm_contrast": 0,
        "publish_pointcloud": True,
        "publish_depth_viz": False,
        "qos_profile": "reliable",
        "enable_watchdog": True,
        "frame_timeout_seconds": 5.0,
    }

    namespace = LaunchConfiguration("namespace")
    publish_depth_viz = LaunchConfiguration("publish_depth_viz")
    qos_profile = LaunchConfiguration("qos_profile")

    # Update params with launch configuration
    params["publish_depth_viz"] = publish_depth_viz
    params["qos_profile"] = qos_profile

    return launch.LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument("publish_depth_viz", default_value="false"),
        DeclareLaunchArgument("qos_profile", default_value="reliable"),
        launch_ros.actions.Node(
            package="revopoint_camera",
            executable="revopoint_camera_node",
            namespace=namespace,
            name="revopoint_camera",
            parameters=[params],
            output="screen",
        ),
    ])
