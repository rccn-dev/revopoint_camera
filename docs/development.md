# Development

Building, running, and the conventions this package follows.

## Build and run

```bash
# From the colcon workspace root (this package is one package inside it)
colcon build --packages-select revopoint_camera
source install/setup.bash
ros2 launch revopoint_camera revo_launch.py
```

Verified on **ROS 2 Jazzy** (the robot host, `ws-northwood-d`) and compile-checked on Humble.
`camera_info_manager` is not in the base ROS image; install `ros-<distro>-camera-info-manager`
before the first build.

There are no tests, no linters wired into the build (no `ament_lint_auto` in package.xml), and no CI.
"Verifying a change" means building and running against real hardware.

Docker covers both the no-hardware and the with-hardware case:

```bash
make docker-build              # compile-check, defaults to DISTRO=jazzy
make docker-build DISTRO=humble
make docker-run                # build AND run against the real camera (--network host)
```

**Compile-check** (`docker-build`) works anywhere, including a Mac under colima. It compiles, links
against the vendored SDK, and is enough to check compilation, RPATH resolution, launch-file wiring
and publisher QoS. With no camera reachable the node starts, declares its parameters, creates its
publishers and then fails at connect. Colima only mounts paths under `/Users`, so stage scratch
files there, not in `/tmp`. Both Jazzy and Humble compile.

**Running against the camera** (`docker-run`) works on a Linux host that sits on the camera's
segment, and needs `--network host` (already in the target) because the SDK reaches the camera by
IP. Verified streaming at full rate from a container; see [networking.md](networking.md).

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

## QoL surface

- `presets/*.yaml` — parameter presets (`default`, `fast`, `quality`, `moveit_handeye`, `lean`).
- `scripts/revo` — bash CLI installed to `lib/<pkg>/`, so `ros2 run revopoint_camera revo ...`
  works. Namespace comes from `REVO_NS` (default `camera`). Edit it and **rebuild** - `ros2 run`
  executes the copy in `install/`, not `scripts/`.
  - `revo caps` calls the node's `~/report_capabilities` service, which reports the outcome of
    every optional property write `set_ext()`/`set_prop()` attempted. It does not scrape logs.
  - `have_node` accepts either `ros2 node list` or the driver's topics as proof the node is up:
    under `docker run --network host`, `ros2 node list` does not report a node in the same
    container even though its topics resolve fine.
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
