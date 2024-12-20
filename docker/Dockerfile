FROM ubuntu:20.04

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Beijing
ENV WS=/root/ws
ENV SHELL=/bin/bash

# Arguments for build
ARG OPENCV_VERSION=4.6.0
ARG CERES_VERSION=2.2.0
ARG ROS_VERSION=noetic

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    libgtk2.0-dev \
    lsb-release \
    net-tools \
    cmake \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    git \
    wget \
    curl \
    htop \
    xterm \
    libpcap-dev \
    binutils-dev \
    libdw-dev \
    libdwarf-dev \
    gdb && \
    apt-get clean

# Install ROS
RUN sh -c 'echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y \
    ros-${ROS_VERSION}-ros-base \
    ros-${ROS_VERSION}-nav-msgs \
    ros-${ROS_VERSION}-sensor-msgs \
    ros-${ROS_VERSION}-cv-bridge \
    ros-${ROS_VERSION}-rviz \
    ros-${ROS_VERSION}-pcl-ros \
    ros-${ROS_VERSION}-image-transport-plugins \
    ros-${ROS_VERSION}-rqt-gui \
    ros-${ROS_VERSION}-rqt-common-plugins \
    ros-${ROS_VERSION}-catkin \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep \
    python3-catkin-tools && \
    apt-get clean && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc

# Install Mavros and Mavlink
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_VERSION}-mavros \
    ros-${ROS_VERSION}-mavros-msgs \
    ros-${ROS_VERSION}-mavros-extras \
    ros-${ROS_VERSION}-mavlink && \
    apt-get clean && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# Install python3 packages
RUN apt-get update && \
    apt-get install -y \
    python3-pip && \
    apt-get clean && \
    python3 -m pip install evo --upgrade --no-binary evo && \
    python3 -m pip install packaging

# Update cmake
RUN apt-get update && \
    apt-get install -y \
    ca-certificates \
    gpg \
    wget \
    software-properties-common && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && \
    apt-get install -y cmake && \
    apt-get clean

#Install OpenCV4
ENV OPENCV_SOURCE_DIR="/root/3rdparty"
ENV INSTALL_DIR="/usr/local"
RUN mkdir -p $OPENCV_SOURCE_DIR && cd $OPENCV_SOURCE_DIR && \
    wget -O opencv.zip https://github.com/Itseez/opencv/archive/${OPENCV_VERSION}.zip && \
    wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/${OPENCV_VERSION}.zip && \
    apt-get install -y unzip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    cd $OPENCV_SOURCE_DIR/opencv-${OPENCV_VERSION} && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        -D ENABLE_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_V4L=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_GSTREAMER_0_10=OFF \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${OPENCV_VERSION}/modules \
        ../ && \
        make -j$(nproc) && \
        make install && \
        ldconfig && \
        rm -rf $OPENCV_SOURCE_DIR/*

# ceres-solver-2.2.0
RUN cd $OPENCV_SOURCE_DIR && \
    wget http://ceres-solver.org/ceres-solver-${CERES_VERSION}.tar.gz && \
    tar zxf ceres-solver-${CERES_VERSION}.tar.gz && \
    mkdir ceres-bin && \
    cd ceres-bin && \
    cmake ../ceres-solver-${CERES_VERSION} && \
    make -j$(nproc) && \
    make install && \
    rm -rf $OPENCV_SOURCE_DIR/*

# Install backword-cpp
RUN cd $OPENCV_SOURCE_DIR && \
    git clone https://github.com/bombela/backward-cpp.git && \
    cd backward-cpp && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf $OPENCV_SOURCE_DIR/*
