# Architecture

`revopoint_camera` is a single-node ROS 2 (ament_cmake) driver for Revopoint 3D
cameras, built on the vendored closed-source **3DCamera SDK**. Essentially all
logic lives in one file, `src/revo_3dcamera_node.cpp` (~2600 lines, one class
`RevopointCameraNode`).

This document covers how that file is put together and which parts are
load-bearing.

For the network and discovery model see [networking.md](networking.md); for
device-specific measurements and traps see [hardware-notes.md](hardware-notes.md).

## Startup sequence (`start_camera()`)
`queryCameras()` → connect (by `serial` param, else first camera) → optional `restart_and_reconnect()` if
`initial_reset` → optional frame-buffer clear → `wait_for_stream_infos()` → `select_streams()` →
`start_streams()` → `configure_depth_exposure/laser/leds/depth_controls()` → `update_camera_info()`.

`start_camera()` only *acquires* the handle - by `queryCameras()` + `connect(info)`, or by
`connect()` directly when discovery finds nothing. Everything after that is in
**`bring_up_camera()`**, which both routes call, so the startup flow exists in exactly one place.
Keep it that way: inlining it back into either branch means every future change to the startup
sequence has to be made twice, and one of the two will eventually be missed.

The camera is flaky on reconnect, hence the layered retries: `wait_for_stream_infos()` (10 × 500 ms),
`reconnect_after_restart()` (20 × 500 ms), `restart_and_reconnect()` (5 attempts, 8 s settle after restart).
Property setters (`configure_*`) log warnings on failure rather than aborting startup — a camera that
partially configures still streams.

## Data path (`frame_pair_callback` → `handle_frame_pair`)
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

## Point cloud construction
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

## Two depth scales
- `depth_scale_sdk_` — **mm per unit** (typ. 0.1), what `cs::Pointcloud::generatePoints()` expects.
- `depth_scale_` — **meters per unit**, published as the ROS `depth_scale` parameter.

Both come from `PROPERTY_EXT_DEPTH_SCALE`; if that read fails they fall back to 0.1 / 0.0001.
The SDK emits point coordinates in **mm**, converted to meters (`* 0.001f`) when copied into `PointCloud2`.

## TF
The SDK's `Extrinsics` describe **depth → RGB**. `publish_extrinsics_transform()` publishes the
**inverse** as a static transform with `frame_id = rgb_frame_id_`, `child_frame_id = depth_frame_id_`,
translation converted mm → m. Note the SDK stores rotation column-major; the `tf2::Matrix3x3` construction
transposes accordingly.

## Watchdog
A 1 Hz wall timer (`check_stream_health`) compares `last_frame_time_ns_` (a `std::atomic<int64_t>` of
`steady_clock` nanoseconds, written from the SDK callback thread) against `frame_timeout_seconds_`.
Warns past half the timeout, errors past it, and dumps `frame_count_`/`error_count_`. It only detects and
logs — it does not attempt recovery.

## Releasing the camera handle (`release_camera()`)

Never let the last `cs::ICameraPtr` go out of scope. `cs::Camera::~Camera()` calls
`pthread_cond_destroy()` on a condition variable that the SDK's own threads
(`cs::System::cameraMonitorThread` and the whole `cs::CameraProbeNetWorking` discovery set) are
still blocked on, without stopping them first - a backtrace of the hang shows 12 waiters on that
condvar. Destroying a condvar with waiters is undefined behaviour and glibc blocks forever, so the
node could only be killed with `SIGKILL` and every `ros2 launch` Ctrl-C left an orphan holding the
camera.

`release_camera()` deliberately leaks one reference so that destructor never runs, and **both**
release sites (`stop_camera()` and `reconnect_after_restart()`) go through it. lib3DCamera.so is
closed source, so this is the only fix available from our side. With it, SIGINT exits cleanly in
~7 s. If you add another place that drops the handle, use `release_camera()`.

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
