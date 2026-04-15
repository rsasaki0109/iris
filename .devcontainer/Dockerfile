FROM ros:noetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Install apt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-pcl-ros \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libsuitesparse-dev \
    libboost-all-dev \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Build g2o from source (CMake exported targets required)
RUN git clone https://github.com/RainerKuemmerle/g2o.git /tmp/g2o && \
    cd /tmp/g2o && \
    git checkout 20230806_git && \
    cmake -S . -B build \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_SHARED_LIBS=ON \
          -DG2O_BUILD_EXAMPLES=OFF \
          -DG2O_BUILD_APPS=OFF && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig && \
    rm -rf /tmp/g2o

# Build DBoW2 (shinsumicco custom version, pinned to known-good commit)
RUN git clone https://github.com/shinsumicco/DBoW2.git /tmp/DBoW2 && \
    cd /tmp/DBoW2 && \
    git checkout e8cc74d && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig && \
    rm -rf /tmp/DBoW2

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
