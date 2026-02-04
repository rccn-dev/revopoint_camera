#!/usr/bin/env python3

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    params = {
        "serial": "",
        "depth_mode": "",
        "rgb_mode": "",
        "depth_width": 0,
        "depth_height": 0,
        "depth_fps": 0.0,
        "rgb_width": 0,
        "rgb_height": 0,
        "rgb_fps": 0.0,
        "depth_frame_id": "revo_depth_optical_frame",
        "rgb_frame_id": "revo_rgb_optical_frame",
        "depth_camera_info_url": "",
        "rgb_camera_info_url": "",
        "depth_auto_exposure": False,
        "depth_exposure_us": 3000.0,
        "depth_frame_time_us": 7500.0,
        "laser_enable": True,
        "laser_luminance": 100,
        "publish_pointcloud": False,
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
