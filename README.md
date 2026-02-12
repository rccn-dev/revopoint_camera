# Revopoint Camera ROS2 Driver

ROS2 driver for Revopoint 3D cameras.

## Features

- Depth and RGB camera stream publishing
- Camera info publishing with intrinsic/extrinsic calibration
- Point cloud generation
- Configurable camera parameters (exposure, gain, LED control, etc.)
- **Stream health monitoring and resilience** (NEW)

## Stream Health Monitoring

The driver now includes built-in stream health monitoring to detect and report silent failures:

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

## Usage

### Basic Launch

```bash
ros2 launch revopoint_camera revo_launch.py
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
- `depth_profile`: Depth stream profile (e.g., "640x480@30")
- `rgb_profile`: RGB stream profile (e.g., "1600x1200@20")

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
- `qos_profile`: QoS profile: "reliable", "best_effort", or "sensor_data" (default: "reliable")

### Advanced
- `initial_reset`: Perform camera reset on startup (default: false)
- `clear_frame_buffer`: Clear frame buffer on startup (default: false)
- `trigger_mode`: Hardware trigger mode (default: 0 = off)

## Topics

### Published Topics
- `depth/image_raw` (sensor_msgs/Image): Raw depth image (16-bit)
- `rgb/image_raw` (sensor_msgs/Image): RGB image
- `depth/camera_info` (sensor_msgs/CameraInfo): Depth camera calibration
- `rgb/camera_info` (sensor_msgs/CameraInfo): RGB camera calibration
- `points` (sensor_msgs/PointCloud2): Colored point cloud (if enabled)
- `depth/image_viz` (sensor_msgs/Image): Colorized depth visualization (if enabled)

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

## CI/CD

This repository includes GitHub Actions CI workflow for automated building and testing. See [.github/CI_SETUP.md](.github/CI_SETUP.md) for details on the CI configuration.

## License

See package.xml for license information.

## Support

For issues related to stream monitoring or silent failures, please include:
- Frame statistics from error logs
- Timeout threshold used
- Camera model and firmware version
- USB connection type (2.0, 3.0, etc.)
