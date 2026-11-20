#!/usr/bin/env python3
"""Launch the Revopoint camera driver.

Parameters are resolved in three layers, each overriding the one before it:

  1. BASE_PARAMS below — the driver's baseline configuration.
  2. presets/<preset>.yaml — selected with `preset:=<name>`.
  3. Launch arguments explicitly given on the command line.

The three layers are merged in Python before the node is created, so the
precedence is exact rather than dependent on how launch orders parameter files.

That ordering is what makes `preset:=fast depth_profile:=1600x1200x6` behave the
way you would expect: the preset applies, and the explicit argument still wins.
An argument you do not pass is left at its sentinel and never reaches the node,
so it cannot silently overwrite a preset with a launch-file default.

NOTE ON FRAME RATE: the depth profile is a hardware ceiling no driver-side work
can raise. The device offers 1600x1200 at 6 Hz and 800x600 at 12 Hz, so
`preset:=fast` (or `depth_profile:=800x600x12`) is the biggest FPS lever there
is. Profile strings are "WxHxFPS", not "WxH@FPS".
"""

import os

import launch
import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions


# Sentinel for "not given on the command line". Any argument still holding this
# is dropped instead of being passed to the node.
UNSET = "__unset__"

# Presets live in presets/<name>.yaml:
#   default         no changes beyond BASE_PARAMS
#   fast            800x600x12, best effort, depth 1     -> highest frame rate
#   quality         1600x1200x6 + HDR + frame fusion     -> best depth
#   moveit_handeye  RGB only, reliable QoS               -> hand-eye calibration
#   lean            depth only, no cloud                 -> minimum bandwidth
PRESETS = ("default", "fast", "quality", "moveit_handeye", "lean")

# Overridable at launch time, with the type each one must be converted to.
OVERRIDABLE = {
    "serial": str,
    "depth_profile": str,
    "rgb_profile": str,
    "publish_pointcloud": bool,
    "align_depth": bool,
    "publish_depth_viz": bool,
    "publish_diagnostics": bool,
    "qos_profile": str,
    "qos_depth": int,
    "lazy_publish": bool,
    "pointcloud_async": bool,
    "hdr_mode": str,
    "depth_filter_level": int,
    "multiframe_fusion": int,
    "frame_timeout_seconds": float,
    "enable_watchdog": bool,
}

# Baseline. Presets and command-line arguments are layered on top of this.
BASE_PARAMS = {
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
    "enable_watchdog": True,
    "frame_timeout_seconds": 5.0,
    "publish_pointcloud": True,
    "align_depth": True,
    "publish_depth_viz": False,
    "publish_diagnostics": True,
    "qos_profile": "sensor_data",
    "qos_depth": 2,
    "lazy_publish": True,
    "pointcloud_async": True,
}


def _to_bool(text):
    lowered = text.strip().lower()
    if lowered in ("1", "true", "yes", "on"):
        return True
    if lowered in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(
        "expected a boolean (true/false), got '{}'".format(text))


def _convert(name, text, kind):
    try:
        if kind is bool:
            return _to_bool(text)
        return kind(text)
    except (ValueError, RuntimeError) as exc:
        raise RuntimeError(
            "launch argument {}:='{}' is invalid: {}".format(name, text, exc))


def _load_preset(preset):
    """Read presets/<preset>.yaml and return its ros__parameters mapping.

    The merge is done here rather than by handing launch a list of parameter
    sources, because launch does not guarantee that a later dict overrides an
    earlier file — it does not, which would silently make every command-line
    override lose to the preset.
    """
    share = FindPackageShare("revopoint_camera").find("revopoint_camera")
    path = os.path.join(share, "presets", preset + ".yaml")
    if not os.path.isfile(path):
        raise RuntimeError("preset file not found: {}".format(path))

    with open(path, "r") as handle:
        doc = yaml.safe_load(handle) or {}

    params = {}
    for node_key, body in doc.items():
        if not isinstance(body, dict):
            continue
        params.update(body.get("ros__parameters", {}) or {})
    if not params:
        raise RuntimeError(
            "preset '{}' contains no ros__parameters (expected a '/**:' block)".format(preset))
    return params


def _launch_setup(context, *_args, **_kwargs):
    preset = LaunchConfiguration("preset").perform(context)
    if preset not in PRESETS:
        raise RuntimeError(
            "unknown preset '{}'. Available: {}".format(preset, ", ".join(PRESETS)))

    overrides = {}
    for name, kind in OVERRIDABLE.items():
        text = LaunchConfiguration(name).perform(context)
        if text != UNSET:
            overrides[name] = _convert(name, text, kind)

    # Lowest to highest precedence, merged explicitly so the result is
    # unambiguous: baseline, then preset, then the command line.
    params = dict(BASE_PARAMS)
    params.update(_load_preset(preset))
    params.update(overrides)

    return [
        launch_ros.actions.Node(
            package="revopoint_camera",
            executable="revopoint_camera_node",
            namespace=LaunchConfiguration("namespace"),
            name="revopoint_camera",
            parameters=[params],
            output="screen",
        ),
    ]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument(
            "preset", default_value="default",
            description="Parameter preset: " + ", ".join(PRESETS)),
    ]
    declarations += [
        DeclareLaunchArgument(
            name, default_value=UNSET,
            description="Overrides the preset. Type: {}".format(kind.__name__))
        for name, kind in OVERRIDABLE.items()
    ]
    return launch.LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
