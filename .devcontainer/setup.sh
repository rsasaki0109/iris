#!/bin/bash
set -e

REPO_DIR=/catkin_ws/src/iris
CATKIN_WS=/catkin_ws

# Initialize submodules (openvslam)
cd "$REPO_DIR"
git submodule update --init --recursive

# rosdep
rosdep update
rosdep install --from-paths "$CATKIN_WS/src" --ignore-src -r -y

# Build
source /opt/ros/noetic/setup.bash
cd "$CATKIN_WS"
catkin_make -DCMAKE_BUILD_TYPE=Release

# Persist workspace sourcing
echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc
