# CI/CD Configuration

This repository uses GitHub Actions for continuous integration. The CI workflow automatically builds and tests the ROS2 package on every push and pull request.

## CI Workflow

The workflow is defined in `.github/workflows/ros2_ci.yml` and performs the following steps:

1. **Checkout code** - Clones the repository into the appropriate workspace structure
2. **Setup ROS environment** - Configures the ROS2 environment
3. **Install rosdep** - Installs the ROS dependency management tool
4. **Initialize rosdep** - Initializes rosdep (runs `sudo rosdep init`)
5. **Update rosdep** - Updates the rosdep database (`rosdep update`)
6. **Install dependencies** - Installs all package dependencies using rosdep with the `PIP_BREAK_SYSTEM_PACKAGES=1` environment variable
7. **Build workspace** - Builds the package using colcon
8. **Run tests** - Executes package tests

## Configuring for CI Setup

Either environment variables or configuration files can be used with your CI system. Which one you choose will depend on how your CI environment is configured.

### Environment Variables Method

The most straightforward approach is to set the environment variable in the shell or script execution context before invoking rosdep:

```bash
sudo rosdep init
rosdep update
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -r --from-paths src/ --ignore-src -y
```

This is the method used in the GitHub Actions workflow (see line 48-51 in `.github/workflows/ros2_ci.yml`).

### Why PIP_BREAK_SYSTEM_PACKAGES is needed

Starting with PEP 668 (externally managed environments), Python 3.11+ and some Linux distributions prevent `pip` from installing packages outside of virtual environments to avoid conflicts with system packages. ROS2 rosdep may invoke pip to install Python dependencies, so the `PIP_BREAK_SYSTEM_PACKAGES=1` environment variable is required to allow these installations in CI environments where system package installation is expected.

## Supported ROS2 Distributions

The CI workflow tests against multiple ROS2 distributions:
- ROS2 Rolling (latest development release)
- ROS2 Jazzy (current LTS)
- ROS2 Iron

## Local Testing

To replicate the CI environment locally:

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rccn-dev/revopoint_camera.git

# Install dependencies
cd ~/ros2_ws
sudo rosdep init  # Only needed once on your system
rosdep update
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -r --from-paths src/ --ignore-src -y

# Build
source /opt/ros/rolling/setup.bash
colcon build --packages-select revopoint_camera

# Test
source install/setup.bash
colcon test --packages-select revopoint_camera
```

## Customizing the Workflow

To modify the CI configuration, edit `.github/workflows/ros2_ci.yml`. Common customizations include:

- **Adding/removing ROS distributions**: Modify the `matrix.ros_distribution` list
- **Adding additional build flags**: Add options to the `colcon build` command
- **Running specific tests**: Modify the test step to target specific test suites
- **Installing additional dependencies**: Add steps before the build step
