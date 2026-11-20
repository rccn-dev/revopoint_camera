# Revopoint Camera ROS2 Driver

ROS2 driver for Revopoint 3D cameras, built on the vendored closed-source
3DCamera SDK. Single node, `ament_cmake`.

## Documentation

- [Architecture](docs/architecture.md) — how the node is put together, and which
  parts are load-bearing.
- [Networking and camera discovery](docs/networking.md) — **read first if the
  driver reports no cameras.** These are network cameras and discovery is
  link-local.
- [Hardware notes](docs/hardware-notes.md) — measured device behaviour, known
  quirks, and open questions.
- [Development](docs/development.md) — build, run, Docker, and conventions.

## Features

- Depth and RGB camera stream publishing
- Camera info publishing with intrinsic/extrinsic calibration
- Point cloud generation
- Configurable camera parameters (exposure, gain, LED control, etc.)
- **Stream health monitoring and resilience**
- **Fused point cloud pipeline** (see [Performance](#performance))

## Performance

Point clouds are built in a single fused pass that projects depth, samples RGB and
writes straight into the `PointCloud2` payload. On a 1600x1200 depth map with ~18%
invalid pixels:

| Stage | Cost |
| --- | ---: |
| Point cloud construction | 6.6 ms |
| Depth unit -> millimetre conversion | 0.6 ms |
| Point cloud payload | 21.8 MB |

What keeps it there:

- **`CMakeLists.txt` forces `Release` when no build type is given.** `colcon` passes
  none by default, and the per-pixel pipeline is roughly 8x slower at `-O0`.
- **`cs::Pointcloud::generatePoints()` is not used.** Per frame it costs two 23 MB
  allocations and computes surface normals this driver has no use for, traversing a
  row-major array column-major, then pushes 1.92M elements across three `std::vector`s.
- **Clouds are built on a worker thread**, fed by a single-slot latest-frame-wins
  staging buffer, so a slow consumer drops frames instead of stalling capture.
  `pointcloud_async:=false` forces inline building.
- **Per-frame work is skipped for topics with no subscribers** (`lazy_publish`).
- **The depth unit -> millimetre conversion uses a lookup table.**

Measured end to end on a network unit (fw v5.5.62), 0 errors and 0 dropped clouds
at both profiles:

| `depth_profile` | Frame rate |
| --- | ---: |
| 1600x1200 (device default) | ~4.7 Hz |
| `800x600x12` | 12 Hz |

**The depth profile is a hard ceiling.** The device offers 1600x1200 at 6 Hz and
800x600 at 12 Hz. No driver-side optimization raises that, so if you need more than
6 FPS, launch with `depth_profile:=800x600x12`.

Note that `ros2 topic hz` substantially under-reports these rates - it cannot keep up
with multi-megabyte messages. Use `revo bench`, or `measured_fps` in `/diagnostics`.

### Point cloud semantics

- `points` is an **unordered** cloud marked `is_dense`: invalid depth pixels are
  dropped rather than published as `(0, 0, 0)`, as are points that project outside
  the RGB frame.
- Colour is sampled at the texel the depth point projects onto, using
  `p_rgb = R * (p_depth + T)` to match `cs::Pointcloud::insertPoints`.
- `qos_profile` defaults to `sensor_data` (best effort). Pass `qos_profile:=reliable`
  for reliable delivery.

## Stream Health Monitoring

The driver includes built-in stream health monitoring to detect and report silent failures:

### Features

1. **Watchdog Timer**: Monitors frame arrival and detects stream timeouts
2. **Exception Handling**: Catches and logs errors in frame callbacks
3. **Frame Statistics**: Tracks total frames received and error count
4. **Null Pointer Protection**: Validates frame data before processing
5. **Throttled Warnings**: Reports transient errors without flooding logs

### Parameters

- `enable_watchdog` (bool, default: true): Enable/disable stream health monitoring
- `frame_timeout_seconds` (double, default: 5.0): Timeout threshold for detecting stream failures

### Monitoring Behavior

- **Normal Operation**: No warnings
- **Stream Degraded**: Warning logged when no frames for >50% of timeout threshold
- **Stream Timeout**: Error logged with frame statistics when timeout threshold exceeded

### Example Output

```
[INFO] Stream watchdog enabled with 5.0 second timeout
[WARN] Stream degraded: No frames for 3 seconds (timeout threshold: 5.0 seconds)
[ERROR] Stream timeout! No frames received for 6 seconds. Camera may have failed.
[ERROR] Frame statistics - Total: 1234, Errors: 5, Last frame: 6 seconds ago
```

## Quick start

```bash
ros2 launch revopoint_camera revo_launch.py preset:=fast   # highest frame rate
ros2 run revopoint_camera revo watch                       # live FPS / temps
ros2 run revopoint_camera revo rviz                        # visualise
```

## Presets

`preset:=<name>` loads `presets/<name>.yaml`. Parameters resolve in three
layers, merged in that order — baseline, then the preset, then anything you pass
explicitly — so `preset:=fast depth_profile:=1600x1200x6` applies the fast preset
but keeps your resolution.

| Preset | For | Notable settings |
| --- | --- | --- |
| `default` | Unchanged baseline | — |
| `fast` | Highest frame rate | `800x600x12`, best effort, queue depth 1 |
| `quality` | Best depth | `1600x1200x6`, HDR, multiframe fusion, depth viz |
| `moveit_handeye` | Hand-eye calibration | RGB only, no cloud, reliable QoS |
| `lean` | Minimum bandwidth | Depth image only, no cloud |

Presets are plain YAML in `presets/`; copy one to make your own.

## The `revo` helper

A small CLI wrapping the common test and control commands:

```bash
ros2 run revopoint_camera revo help
```

| Command | Does |
| --- | --- |
| `revo fast` / `quality` / `handeye` | Launch with that preset |
| `revo rates [sec]` | Publish rate of every camera topic |
| `revo bench [sec]` | Frame rate + bandwidth summary |
| `revo diag` | Latest diagnostics: measured FPS, temperatures, drops |
| `revo watch` | Same, refreshed once a second |
| `revo topics` | Topics with their resolved QoS |
| `revo caps` | Which optional SDK features this unit accepted |
| `revo pause` / `resume` | Pause / resume the depth stream |
| `revo trigger` | Send one software trigger |
| `revo reset-stats` | Zero the frame / error / drop counters |
| `revo record [sec]` | Record a bag of all camera topics |
| `revo rviz` | RViz with the bundled config |

There is also a `Makefile` with the same shortcuts (`make fast`, `make bench`,
`make docker-build`).

## Diagnostics

The node publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` at 1 Hz
(`publish_diagnostics`, default true) with **measured** frame rate — counted in
the SDK callback, so it reflects what the camera actually delivered rather than
what any one topic published — plus error and dropped-cloud counters, stream
resolutions, and CPU / NTC / sensor temperatures where the model reports them.

`revo watch` is the easiest way to read it.

## Services

All are `std_srvs/Trigger` under the node's private namespace:

| Service | Does |
| --- | --- |
| `~/pause_stream` | Pause the depth stream |
| `~/resume_stream` | Resume it |
| `~/soft_trigger` | Fire one software trigger (needs `trigger_mode:=2`) |
| `~/reset_stats` | Zero the frame / error / drop counters |

## Usage

### Basic Launch

```bash
ros2 launch revopoint_camera revo_launch.py
```

### Launch at the higher frame rate

```bash
ros2 launch revopoint_camera revo_launch.py depth_profile:=800x600x12
```

### Launch with Custom Watchdog Settings

```bash
ros2 launch revopoint_camera revo_launch.py enable_watchdog:=true frame_timeout_seconds:=10.0
```

### Disable Watchdog (for debugging)

```bash
ros2 launch revopoint_camera revo_launch.py enable_watchdog:=false
```

## Parameters

### Stream Monitoring (NEW)
- `enable_watchdog`: Enable stream health monitoring (default: true)
- `frame_timeout_seconds`: Timeout for stream failure detection in seconds (default: 5.0)

### Camera Configuration
- `serial`: Camera serial number (empty = auto-detect)
- `depth_profile`: Depth stream profile, format `WxHxFPS` (e.g. `800x600x12`,
  `1600x1200x6`). Empty = device default.
- `rgb_profile`: RGB stream profile, format `WxHxFPS` (e.g. `1600x1200x20`).
  Empty = device default.

### Frame IDs
- `depth_frame_id`: TF frame ID for depth (default: "revo_depth_optical_frame")
- `rgb_frame_id`: TF frame ID for RGB (default: "revo_rgb_optical_frame")

### Exposure and Gain
- `depth_auto_exposure`: Enable auto-exposure (default: false)
- `depth_exposure_us`: Manual exposure time in microseconds (default: 3000.0)
- `depth_frame_time_us`: Frame time in microseconds (default: 7500.0)
- `depth_gain`: Sensor gain (default: 1.0)

### LED Control
- `laser_enable`: Enable laser projector (default: true)
- `laser_luminance`: Laser brightness 0-100 (default: 100)
- `ir_led_enable`: Enable IR LED (default: false)
- `ir_led_luminance`: IR LED brightness 0-100 (default: 0)
- `rgb_led_mode`: RGB LED mode: "disable", "enable", "often_bright", "twinkle" (default: "disable")

### Depth Processing
- `depth_range_min_mm`: Minimum depth range in mm (default: 50)
- `depth_range_max_mm`: Maximum depth range in mm (default: 2000)
- `algorithm_contrast`: Contrast threshold for depth algorithm (default: 0)

### Publishing
- `publish_pointcloud`: Publish point cloud (default: true)
- `publish_depth_viz`: Publish colorized depth visualization (default: false)
- `qos_profile`: QoS profile: "sensor_data", "best_effort", or "reliable"
  (default: "sensor_data")
- `qos_depth`: Publisher queue depth (default: 2). Keep this small; a deep queue
  on multi-megabyte messages adds latency without adding value.
- `lazy_publish`: Skip per-frame work for topics with no subscribers
  (default: true)
- `pointcloud_async`: Build point clouds on a worker thread instead of on the
  SDK capture thread (default: true). Latest-frame-wins: when a consumer cannot
  keep up, frames are dropped rather than stalling the driver.

### Advanced
- `initial_reset`: Perform camera reset on startup (default: false)
- `clear_frame_buffer`: Clear frame buffer on startup (default: false)
- `trigger_mode`: Trigger mode: 0 = off, 1 = hardware, 2 = software (default: 0)

### Optional SDK features

These reach device properties beyond the core depth exposure / laser / LED set.
Every one is **opt-in**: left at its sentinel (`-1`, `""`, or an empty list) the
driver issues no property write at all, so it cannot affect a device that does
not implement it. Not every Revopoint model implements every property — an
unsupported one logs a warning at startup and is skipped rather than failing the
launch. Use `revo caps` to see what your unit accepted.

Tri-state integers below mean: `-1` leave the device default, `0` off, `1` on.

- `hdr_mode`: `off`, `high_reflect`, `low_reflect`, `all_reflect`. Helps on
  scenes mixing bright/shiny and dark surfaces.
- `hdr_scale`: 4 ints `[high_count, high_scale, low_count, low_scale]` tuning
  auto-HDR.
- `depth_filter_level`: on-device depth filtering strength (int; valid range is
  model dependent).
- `depth_roi` / `rgb_roi`: 4 ints `[left, top, right, bottom]` as **percentages**
  (0-100). A depth ROI is a genuine throughput lever — it reduces what the
  sensor produces, shrinking every stage downstream.
- `multiframe_fusion`: dual-exposure fusion (tri-state). Better completeness,
  costs frame rate.
- `fast_scan_mode`: vendor fast scan mode (tri-state).
- `hardware_sync`: hardware sync switch, for multi-camera rigs (tri-state).
- `trigger_in_mode` / `trigger_out_enable`: external trigger routing.
- `max_frame_time_us` / `max_gain`: ceilings auto-exposure may use (vendor
  recommends 15000 and 48).
- `indicator_light_mode`: `off`, `red`, `red_flashing`, `green`.

RGB sensor controls:

- `rgb_auto_exposure`, `rgb_exposure_us`, `rgb_gain`
- `rgb_auto_white_balance`, `rgb_white_balance`
- `rgb_auto_focus`, `rgb_focus`

## Topics

### Published Topics
- `depth/image_raw` (sensor_msgs/Image): Raw depth image (16-bit)
- `rgb/image_raw` (sensor_msgs/Image): RGB image
- `depth/camera_info` (sensor_msgs/CameraInfo): Depth camera calibration
- `rgb/camera_info` (sensor_msgs/CameraInfo): RGB camera calibration
- `points` (sensor_msgs/PointCloud2): Colored point cloud (if enabled)
- `depth/image_viz` (sensor_msgs/Image): Colorized depth visualization (if enabled)
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): Measured FPS, counters,
  temperatures (if `publish_diagnostics`)

## Troubleshooting

### Camera Stream Stops Working

The watchdog will automatically detect this condition and log:
```
[ERROR] Stream timeout! No frames received for 5 seconds. Camera may have failed.
```

**Solutions:**
1. Check USB connection and cable quality
2. Check camera power supply
3. Restart the node
4. Try increasing `frame_timeout_seconds` if using low frame rates

### High Error Count

If you see frequent warnings about empty frames or data errors:
1. Check USB bandwidth (use USB 3.0+ port)
2. Verify camera firmware version
3. Try reducing resolution or frame rate
4. Check system CPU/memory usage

### Performance Impact

The watchdog timer runs at 1 Hz and has minimal performance impact:
- CPU: <0.1% overhead
- Memory: ~24 bytes for atomic variables plus minimal timer overhead
- No impact on frame callback performance

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select revopoint_camera
source install/setup.bash
```

`CMakeLists.txt` defaults `CMAKE_BUILD_TYPE` to `Release`. Do not override it
with an unoptimized build unless you are debugging — at `-O0` the frame pipeline
is roughly 8x slower.

## License

See package.xml for license information.

## Support

For issues related to stream monitoring or silent failures, please include:
- Frame statistics from error logs
- Timeout threshold used
- Camera model and firmware version
- USB connection type (2.0, 3.0, etc.)

## Known Issues

- The driver is Linux-only: `CMakeLists.txt` resolves the SDK under
  `lib/3dcamera/linux/{x64,aarch64}`, even though the vendor SDK also ships
  macOS and Windows binaries.
