# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`revopoint_camera` is a single-node ROS 2 (ament_cmake) driver for Revopoint 3D cameras, built on the
vendored closed-source **3DCamera SDK**. Essentially all logic lives in one file,
`src/revo_3dcamera_node.cpp` (~1600 lines, one class `RevopointCameraNode`).

## Build / run

```bash
# From the colcon workspace root (this package is one package inside it)
colcon build --packages-select revopoint_camera
source install/setup.bash
ros2 launch revopoint_camera revo_launch.py
```

There are no tests, no linters wired into the build (no `ament_lint_auto` in package.xml), and no CI.
"Verifying a change" means building and running against real hardware.

Without hardware, you can still get a long way in Docker (the dev machine is macOS; the build is
Linux-only). `colima start` then:

```bash
docker run --rm -v "$PWD":/ws/src/revopoint_camera -w /ws ros:humble-ros-base bash -lc '
  apt-get update -qq && apt-get install -y -qq ros-humble-camera-info-manager
  source /opt/ros/humble/setup.bash && colcon build --packages-select revopoint_camera'
```

This compiles, links against the vendored SDK, and the node will start, declare its parameters,
create publishers and fail cleanly at `queryCameras`. It is enough to check compilation, RPATH
resolution, launch-file wiring and publisher QoS. Colima only mounts paths under `/Users`, so stage
scratch files there, not in `/tmp`.

**Linux x86_64/aarch64 only.** `CMakeLists.txt` hardcodes `lib/3dcamera/linux/{x64,aarch64}/lib3DCamera.so`,
so the build will not link on macOS even though the SDK ships `macos/` dylibs. Development on a Mac is
edit-only; building and testing happen on the robot/Linux host (or in Docker, above).

`CMakeLists.txt` forces `CMAKE_BUILD_TYPE=Release` when none is given, because colcon passes no build
type by default and the per-pixel image pipeline is ~8x slower at `-O0`. Do not remove that block.

### One-time host setup (Linux, for USB access)

```bash
sudo ./OpenNISDK_v1.2.3.0137/install_openni_driver.sh   # udev rules + OpenNI2 driver plugin
```
This is independent of the ROS build (the directory carries `COLCON_IGNORE`). It installs
`/etc/udev/rules.d/cs_uvc.rules` (vendor 2207, product 110a) and is what makes the camera visible at all.

## Vendored SDK layout

Both `3DCameraSDK-v3.2.229.20251110/` and `OpenNISDK_v1.2.3.0137/` contain `COLCON_IGNORE` so colcon
does not treat them as packages. Do not remove those files.

The SDK is not installed system-wide, so `CMakeLists.txt` does RPATH gymnastics:
- **build** RPATH → the in-tree `lib/3dcamera/linux/<arch>/`
- **install** RPATH → `$ORIGIN/../share/revopoint_camera/3dcamera_sdk/lib/3dcamera/linux/<arch>/`, and the
  SDK's `lib/` and `inc/` are copied into `share/revopoint_camera/3dcamera_sdk/`.

If you change where the executable or the SDK is installed, both RPATH lines and both `install(DIRECTORY ...)`
lines must move together or the node will fail to load `lib3DCamera.so` at runtime.

Vendor sample code under `3DCameraSDK-*/src/Sample*/` is the best reference for SDK API usage
(`SampleDepthRGBFrameMatch`, `SampleLiveDepthCallback`, `SampleHandEye*`). SDK docs are the `.chm` files
under `3DCameraSDK-*/doc/`.

## Architecture of the node

### Startup sequence (`start_camera()`)
`queryCameras()` → connect (by `serial` param, else first camera) → optional `restart_and_reconnect()` if
`initial_reset` → optional frame-buffer clear → `wait_for_stream_infos()` → `select_streams()` →
`start_streams()` → `configure_depth_exposure/laser/leds/depth_controls()` → `update_camera_info()`.

If `queryCameras()` returns nothing there is a **fallback direct-connect path that duplicates this entire
sequence verbatim** inside the same function. Any change to the startup flow must be made in both halves of
`start_camera()`.

The camera is flaky on reconnect, hence the layered retries: `wait_for_stream_infos()` (10 × 500 ms),
`reconnect_after_restart()` (20 × 500 ms), `restart_and_reconnect()` (5 attempts, 8 s settle after restart).
Property setters (`configure_*`) log warnings on failure rather than aborting startup — a camera that
partially configures still streams.

### Data path (`frame_pair_callback` → `handle_frame_pair`)
The SDK calls `frame_pair_callback` on **its own thread**, not a ROS executor thread. The SDK does not
buffer, so time spent in this callback is time not spent draining USB and shows up directly as dropped
frames. It is a static trampoline that catches all exceptions (an escaping exception would cross the SDK
ABI boundary) and bumps `error_count_`.

Ordering inside `handle_frame_pair` is deliberate and load-bearing:
1. Watchdog atomics are updated first, so liveness is recorded even when everything below is skipped.
2. One `stamp` is taken for every message in the cycle.
3. Lazy publishing: each stage is skipped when its topic has no subscribers (`lazy_publish`, default on).
4. **RGB image + camera_info are published first**, before any depth work — MoveIt's hand-eye plugin
   synchronizer drops pairs if RGB lags. Do not move depth ahead of it.
5. The cloud job is staged from the **raw** depth buffer (units, not mm).
6. Only then is depth converted to millimeters, fused into the copy via `depth_mm_lut_`.

Swapping steps 5 and 6 silently produces a point cloud scaled by `depth_scale_sdk_`².

Messages are published as `unique_ptr` and `NodeOptions::use_intra_process_comms` is on, so a composed
subscriber receives them without a serialize/copy round trip. Image buffers are filled with
`std::vector::assign` rather than `resize` + `memcpy`, which avoids a pointless zero-fill pass over
several megabytes.

### Point cloud construction
`cs::Pointcloud::generatePoints()` is **not used** — it cost ~300 ms/frame at 1600x1200 (two 23 MB
mallocs, unused surface normals computed with a column-major traversal of a row-major array, then 1.92M
push_backs across three vectors). `build_and_publish_cloud()` replaces it with one fused pass that
projects depth, samples RGB and writes straight into the `PointCloud2` payload: ~6.6 ms, no allocation.
Constants are precomputed once in `initialize_projection_parameters()` (called at the end of
`update_camera_info()`, since it needs intrinsics, extrinsics and depth scale).

The kernel works in **millimeters**, matching the SDK, because `Extrinsics::translation` is in mm; the
mm → m conversion happens only when writing each point. Colour uses `p_rgb = R * (p_depth + T)`, matching
`cs::Pointcloud::insertPoints`. Invalid points and points that fall outside the RGB frame are dropped,
so the published cloud is `is_dense`.

Cloud building runs on a **worker thread** (`cloud_worker`), fed by a single-slot latest-frame-wins
staging buffer (`pending_job_`). The worker `std::swap`s the job out, which hands its buffers back to the
producer, so steady state allocates nothing. A slow consumer causes dropped frames (counted in
`cloud_dropped_`), never a stalled capture thread. `pointcloud_async:=false` forces inline building.

### Two depth scales
- `depth_scale_sdk_` — **mm per unit** (typ. 0.1), what `cs::Pointcloud::generatePoints()` expects.
- `depth_scale_` — **meters per unit**, published as the ROS `depth_scale` parameter.

Both come from `PROPERTY_EXT_DEPTH_SCALE`; if that read fails they fall back to 0.1 / 0.0001.
The SDK emits point coordinates in **mm**, converted to meters (`* 0.001f`) when copied into `PointCloud2`.

### TF
The SDK's `Extrinsics` describe **depth → RGB**. `publish_extrinsics_transform()` publishes the
**inverse** as a static transform with `frame_id = rgb_frame_id_`, `child_frame_id = depth_frame_id_`,
translation converted mm → m. Note the SDK stores rotation column-major; the `tf2::Matrix3x3` construction
transposes accordingly.

### Watchdog
A 1 Hz wall timer (`check_stream_health`) compares `last_frame_time_ns_` (a `std::atomic<int64_t>` of
`steady_clock` nanoseconds, written from the SDK callback thread) against `frame_timeout_seconds_`.
Warns past half the timeout, errors past it, and dumps `frame_count_`/`error_count_`. It only detects and
logs — it does not attempt recovery.

## Known quirks to be aware of before "fixing" things

- **Profile string format is `WxHxF`, not `WxH@F`.** `parse_profile_string()` uses
  `sscanf(..., "%dx%dx%lf", ...)`, so valid values are e.g. `1600x1200x6`, `800x600x12` (depth) and
  `1600x1200x20` (RGB). The README's `640x480@30` examples are wrong.
- **Empty `depth_profile` requires exactly 1600x1200 Z16** — the empty-string branch of
  `select_depth_stream()` returns false if the device does not offer it, rather than falling through to the
  fallback ladder that the non-empty branch uses.
- The README documents defaults for `depth_range_min_mm`/`depth_range_max_mm` (50/2000) that match the
  **launch file**, not `declare_parameter` in the node (0/0, which means "don't set the property").
- **1600x1200 depth is capped at 6 Hz by the device.** No driver-side work raises that ceiling; the
  answer to "why is FPS low" at full resolution is `depth_profile:=800x600x12`. Check this before
  optimizing anything else.
- The install RPATH needs `$ORIGIN/../../share/...` (two levels: the binary lands in `lib/<pkg>/`). A
  system-wide `lib3DCamera.so`, which `install_openni_driver.sh` installs, silently masks a wrong RPATH,
  so test with `ldd` on the installed binary rather than trusting that it launches.

## Network cameras and discovery (READ BEFORE DEBUGGING "no cameras found")

This unit is a **network camera** whose factory address is `192.168.3.99`. Established by
inspecting the SDK, not by guessing:

- `queryCameras()` discovers over **UDP broadcast** — `lib3DCamera.so` contains
  `ossCreateBroadcastSocket` and `"create broadcast socket failed"`. Broadcast is link-local, so
  **a router will not carry it**. Being able to `ping` the camera proves nothing about whether
  the SDK can find it; reachability and discoverability are separate problems here.
- **There is no connect-by-IP API.** `ICamera::connect()` takes only a `CameraInfo`, which holds
  `name`, `serial`, `uniqueId`, `firmwareVersion`, `algorithmVersion` — no address field. You
  cannot bypass discovery.
- `CameraIpSetting` documents `ipBytes[4]` as literally `{192,168,3,99}`, and `IpSetting` exposes
  only a DHCP flag plus `ipFourthByte`. The SDK's addressing model assumes **the host is on the
  camera's /24**. That is the intended topology.

So the fix is a host interface on the camera's L2 segment (direct-attach with e.g.
`192.168.3.10/24`, or a secondary address on the interface facing it). Routing between subnets is
not sufficient; if the camera must stay routed, a UDP broadcast relay is required.
`PROPERTY_EXT_IP_SETTING` / `PROPERTY_EXT_CAMERA_IP` can move the camera onto another subnet
permanently, but only after one successful connection — chicken-and-egg.

Do not attempt this from inside Docker or a VM. Broadcast discovery through container bridge
networking (and, on macOS, through the colima VM on top of that) will not reach a physical LAN.
Run the driver natively on a Linux host sitting on the camera's segment.

On failure the node spends ~20 s in its direct-connect fallback before logging
`Direct connect failed (ret=2)`. Wait it out before concluding anything.

## Open questions pending hardware

Nothing below has been validated against a real camera; all measurements so far come from a
synthetic benchmark and a Docker run with no device attached.

- **`qos_profile` default changed from `reliable` to `sensor_data`.** The user explicitly flagged
  that they may want `reliable` back for a reason of their own. The change was made from
  throughput reasoning (reliable QoS on ~30 MB clouds causes head-of-line blocking) and only
  verified to the extent that `ros2 topic info -v` reports the profile. Test both against the
  consumers that actually matter (MoveIt hand-eye, RViz, rosbag) and compare dropped frames
  before treating either as settled. Ask which consumers matter rather than assuming.
- **Real frame rate is unmeasured.** The 304 ms -> 6.6 ms cloud figure is a synthetic benchmark.
  `revo bench` reports the measured rate once a camera is streaming.
- **Which optional SDK properties this model accepts is unknown.** Every one fails soft, so
  startup will log `Failed to set ...` for unsupported ones. `revo caps` and the startup log are
  the source of truth.
- The 1-pixel colour registration fix and the dropped-invalid-points change alter what downstream
  consumers see; both are believed correct but have not been eyeballed on real data.

## Presets and the launch file

`launch/revo_launch.py` merges three layers **in Python** inside an
`OpaqueFunction`: `BASE_PARAMS`, then `presets/<preset>.yaml`, then only those launch arguments
the user actually passed (everything else sits at the `UNSET` sentinel and is dropped).

The merge is deliberately not delegated to launch's `parameters=[dict, file, dict]` list — that
was tried and **the later dict does not override the earlier file**, which silently made every
command-line override lose to the preset. Keep the explicit Python merge.

Adding an overridable argument means adding it to `OVERRIDABLE` with its type; the type is used
to convert the string launch gives you, and a bad value raises with a readable message rather
than reaching the node as the wrong type.

## Optional SDK properties

`configure_advanced()` and `configure_rgb_controls()` expose device properties beyond the
original depth exposure/laser/LED set. Two rules hold throughout:

- **Opt-in.** Sentinels are `-1` for numbers, `""` for strings, empty for lists. Unset means no
  property write happens at all, so adding a parameter can never change existing behaviour.
- **Fail soft.** `set_ext()` / `set_prop()` log a warning and continue when the model does not
  implement a property. Models differ; a failed write is not a startup failure.

`camera_mutex_` guards `camera_` against concurrent use by the service callbacks, the
diagnostics timer, and shutdown. The frame callback never touches `camera_`, so it stays
lock-free.

## QoL surface

- `presets/*.yaml` — parameter presets (`default`, `fast`, `quality`, `moveit_handeye`, `lean`).
- `scripts/revo` — bash CLI installed to `lib/<pkg>/`, so `ros2 run revopoint_camera revo ...`
  works. Namespace comes from `REVO_NS` (default `camera`).
- `rviz/revopoint.rviz` — RViz config; its topics are hardcoded to the `camera` namespace.
- `Makefile` — the same shortcuts, plus `make docker-build` for the container compile check.

## Conventions

- 2-space indent, `member_variable_` trailing underscore, ROS 2 / ament style; `-Wall -Wextra -Wpedantic`
  is on, keep it warning-clean.
- SDK calls return `ERROR_CODE`; compare against `SUCCESS` and log `ret=%d` on failure, matching the
  surrounding code.
- Every new tunable is declared in `RevopointCameraNode()` via `declare_parameter`. Add it to
  `BASE_PARAMS` in `launch/revo_launch.py` only if the launch default should differ from the node's;
  add it to `OVERRIDABLE` if it should be settable as `arg:=value`.
- Commit messages are imperative one-liners ("Add …", "Fix …", "Extract …").
