#!/bin/bash
set -e

REPO_DIR=/colcon_ws/src/iris
COLCON_WS=/colcon_ws

# Initialize OpenVSLAM submodule
cd "$REPO_DIR"
git submodule update --init --recursive

# rosdep
rosdep update
rosdep install --from-paths "$COLCON_WS/src" --ignore-src -r -y

# Build (ros2 packages only)
source /opt/ros/humble/setup.bash
cd "$COLCON_WS"
colcon build --symlink-install \
  --packages-select iris openvslam_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source $COLCON_WS/install/setup.bash" >> ~/.bashrc
