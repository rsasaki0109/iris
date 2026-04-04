FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-std-msgs \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-message-filters \
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

# Build DBoW2 (shinsumicco custom version)
RUN git clone https://github.com/shinsumicco/DBoW2.git /tmp/DBoW2 && \
    cd /tmp/DBoW2 && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig && \
    rm -rf /tmp/DBoW2

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
