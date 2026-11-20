# QoL shortcuts. Run from the package directory; WS is the colcon workspace.
WS ?= $(HOME)/ros2_ws
PKG := revopoint_camera
SECS ?= 10

.PHONY: help build rebuild clean run fast quality handeye rviz \
        rates bench diag watch topics params caps docker-build

help:
	@grep -E '^[a-z-]+:.*?## ' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}{printf "  %-14s %s\n", $$1, $$2}'

build:        ## colcon build this package (Release)
	cd $(WS) && colcon build --packages-select $(PKG)
rebuild:      ## clean rebuild of this package
	cd $(WS) && rm -rf build/$(PKG) install/$(PKG) && colcon build --packages-select $(PKG)
clean:        ## remove this package's build artifacts
	cd $(WS) && rm -rf build/$(PKG) install/$(PKG)

run:          ## launch with the default preset
	ros2 launch $(PKG) revo_launch.py
fast:         ## launch tuned for frame rate (800x600x12)
	ros2 launch $(PKG) revo_launch.py preset:=fast
quality:      ## launch tuned for depth quality (1600x1200x6 + HDR)
	ros2 launch $(PKG) revo_launch.py preset:=quality
handeye:      ## launch for MoveIt hand-eye calibration
	ros2 launch $(PKG) revo_launch.py preset:=moveit_handeye
rviz:         ## open RViz with the bundled config
	ros2 run $(PKG) revo rviz

rates:        ## measure per-topic publish rates
	ros2 run $(PKG) revo rates $(SECS)
bench:        ## frame rate + bandwidth summary over SECS seconds
	ros2 run $(PKG) revo bench $(SECS)
diag:         ## print the latest diagnostics
	ros2 run $(PKG) revo diag
watch:        ## live diagnostics, refreshed every second
	ros2 run $(PKG) revo watch
topics:       ## list camera topics with their QoS
	ros2 run $(PKG) revo topics
params:       ## dump the node's current parameters
	ros2 run $(PKG) revo params
caps:         ## show which optional SDK features are available
	ros2 run $(PKG) revo caps

docker-build: ## compile-check in a ROS container (no camera needed)
	docker run --rm -v "$(CURDIR)":/ws/src/$(PKG) -w /ws ros:humble-ros-base bash -lc '\
	  apt-get update -qq && apt-get install -y -qq ros-humble-camera-info-manager ros-humble-diagnostic-msgs >/dev/null && \
	  . /opt/ros/humble/setup.bash && colcon build --packages-select $(PKG)'
