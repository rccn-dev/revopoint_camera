#!/usr/bin/env python3

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    }

    namespace = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")

    tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "tfs.launch.py"])
        ),
        launch_arguments={"namespace": namespace, "tf_prefix": tf_prefix}.items(),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        launch_ros.actions.Node(
            package="revopoint_camera",
            executable="revopoint_camera_node",
            namespace=namespace,
            name="revopoint_camera",
            parameters=[params],
            output="screen",
        ),
        tfs,
    ])
