# FROM ros:noetic
FROM osrf/ros:noetic-desktop-full

    #Update system, install necessary utilities, hector-trajectory
RUN apt-get update && \
    apt-get install -y \
    git \
    ros-noetic-hector-trajectory-server

    # google-glog + gflags
    # Use ATLAS for BLAS & LAPACK
    # Eigen3
    # SuiteSparse (optional)
RUN apt-get install -y \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev
 
    # PCL
RUN apt-get install -y \
    libpcl-dev

    # GTSAM
RUN apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
    apt-get install -y \
    libgtsam-dev libgtsam-unstable-dev

    #Ceres Solver
RUN git clone --depth 1 -b 2.1.0 https://ceres-solver.googlesource.com/ceres-solver /opt/ceres-solver && \
    mkdir -p /opt/ceres-solver/ceres-bin && \
    cd /opt/ceres-solver/ceres-bin && \
    cmake .. && \
    make install -j5

    #Folder for laserOP messages, unnecessary utilities
RUN mkdir -p /root/message/Scans && \
    apt-get install -y \
    psmisc gdb

    #Clone and make SC FLOAM
# RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && \
#     git clone https://github.com/box-of-voodoo/Optimized-SC-F-LOAM.git src/Optimized-SC-F-LOAM && \
#     catkin_make && \

    
