# Hardware notes

Behaviour observed on real devices, and the traps that look like driver bugs but
are not. Measurements come from the unit identified below; other models differ.

## Known quirks

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

## Measured on hardware

Unit `A25700159H6N00D68`, fw `v5.5.62.20251024`, at 192.168.3.99, on ROS 2 Jazzy.

| depth profile | measured_fps | errors | clouds dropped |
|---|---|---|---|
| 1600x1200 (device default) | 4.70 | 0 | 0 |
| `800x600x12` | 11-12 | 0 | 0 |

- **1600x1200 really is capped near 6 Hz**; `depth_profile:=800x600x12` reaches the full 12 Hz.
  The async cloud worker keeps up at both, so a low frame rate is the device, not the pipeline.
- **`ros2 topic hz` under-reports badly** - it showed ~1.7 Hz while the node was doing 4.7. The CLI
  subscriber cannot keep up with multi-megabyte messages. Trust `revo bench` / `/diagnostics`
  (`measured_fps`), which count in the SDK callback.
- **Depth scale is 0.05 mm/unit on this unit**, not the 0.1 fallback.
- **Clouds are ~7.4 MB at 1600x1200, not ~30 MB.** Only ~24% of depth pixels are valid and invalid
  points are dropped, so the cloud is a quarter of the theoretical `w*h*16`.
- Depth intrinsics are `fx=fy=1823.73, cx=538.04, cy=641.34` at a calibration resolution of
  1600x1200. `cx` sits well left of centre; that is the factory calibration, not a driver bug -
  `initialize_projection_parameters()` rescales by `stream / Intrinsics::{width,height}` correctly
  (at 800x600 it halves to 911.86 / 269.02).
- Data was checked end to end: published depth median 384 mm against cloud z mean 0.388 m, and
  cloud point count within ~20 of the valid-pixel count. That agreement is what proves the
  mm-conversion and the step 5/6 ordering are right.

### Optional properties this unit accepts

Confirmed by writing them: **hardware sync, fast scan mode, RGB gain, depth filter level,
multiframe fusion, HDR mode**. Confirmed **rejected: RGB focus** (fixed-focus module).
Anything not in either list has simply never been attempted - see `revo caps`.

## Still open

- **`qos_profile`: `sensor_data` (the default) vs `reliable`.** Unsettled. At 1600x1200 with a
  reliable subscriber attached the node holds 4.71 fps with 0 dropped clouds, statistically
  identical to `sensor_data`'s 4.70, so the head-of-line-blocking argument for best-effort is
  weak here - clouds are ~7 MB, not the ~30 MB a full `w*h*16` would suggest. But the only
  subscriber exercised was the Python CLI. Test against the consumers that actually matter
  (MoveIt hand-eye, RViz, rosbag) and compare dropped frames before treating either default as
  settled, and confirm with whoever owns those consumers which ones actually matter.
- **Several optional parameters are not launch-overridable.** `hardware_sync`, `trigger_mode`,
  `rgb_focus`, `rgb_gain` and `fast_scan_mode` are declared in the node but absent from
  `OVERRIDABLE` in `launch/revo_launch.py`, so `ros2 launch ... hardware_sync:=1` is accepted and
  then silently ignored. Set them with `ros2 run ... --ros-args -p`, or add them to `OVERRIDABLE`.
- **Colour registration has not been checked against a target.** The cloud carries plausible
  per-point RGB and the geometry checks out in aggregate, but nothing has been eyeballed against
  a calibration target. Note the registration is deliberately sampled at the texel the projection
  lands on: normalising the RGB projection by `width` and rescaling by `width - 1` samples
  `(u-1, v-1)` instead, which is wrong by one pixel and easy to reintroduce.
