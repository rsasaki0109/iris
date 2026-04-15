#!/bin/bash
set -e

REPO_DIR=/iris_repo
COLCON_WS=/colcon_ws

# Initialize OpenVSLAM submodule
cd "$REPO_DIR"
git submodule update --init --recursive

# rosdep
rosdep update
rosdep install --from-paths "$REPO_DIR/ros2" --ignore-src -r -y

# Build
# Use --base-paths pointing directly to each ros2 package to avoid CATKIN_IGNORE
# at ros2/ blocking package discovery (catkin_pkg respects CATKIN_IGNORE).
source /opt/ros/humble/setup.bash
cd "$COLCON_WS"
colcon build --symlink-install \
  --base-paths \
    "$REPO_DIR/ros2/iris" \
    "$REPO_DIR/ros2/openvslam_bridge" \
  --build-base "$COLCON_WS/build" \
  --install-base "$COLCON_WS/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source $COLCON_WS/install/setup.bash" >> ~/.bashrc
