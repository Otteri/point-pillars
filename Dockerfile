###################################################################
# Author: Joni Airaksinen (Otteri)
# Build: $ docker build -t pointpillars:latest .
# Run:   $ docker run --gpus all --rm -it \
#          -v `pwd`/config:/app/config/ --network=host pointpillars
#
# Keywords: CUDA 11.3, cudnn 8, TensorRT 8, ubuntu20.04
###################################################################

# Path to binary release. Assumes it to be in repo root by default.
ARG TENSORRT_TAR=TensorRT-8.0.3.4.Linux.x86_64-gnu.cuda-11.3.cudnn8.2.tar.gz

FROM nvidia/cuda:11.3.0-cudnn8-devel-ubuntu20.04 AS dependency-stage

# Setup user account
RUN useradd -ms /bin/bash pointpillars

# Ubuntu 20.04 requres this, because otherwise we get tzdata dialogue
ENV DEBIAN_FRONTEND noninteractive
ENV TENSORRT_INSTALL=/app/TensorRT-8.0.3.4

# Install common build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf \
    automake \
    build-essential \
    curl \
    dialog apt-utils \
    git \
    g++ \
    libboost-all-dev \
    libssl-dev \
    libtool \
    libyaml-cpp-dev \
    lsb-release \
    make \
    pkg-config \
    python3-pip \
    python3-setuptools \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install python packages
COPY requirements.txt /app/requirements.txt
RUN python3 -m pip install --upgrade pip && pip install -r /app/requirements.txt
RUN ln -s /usr/bin/python3 /usr/bin/python

# Copy Nvidia sources
COPY third_party/onnx-tensorrt /app/onnx-tensorrt/
COPY third_party/TensorRT /app/TensorRT/

# First get the TensorRT binary release
ARG TENSORRT_TAR
COPY ${TENSORRT_TAR} /app/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${TENSORRT_INSTALL}/lib
RUN tar -xvzf /app/${TENSORRT_TAR} -C /app/ \
    && rm /app/${TENSORRT_TAR} \
    && export TRT_LIBPATH=${TENSORRT_INSTALL} \
    # Next build steps are done in TensorRT repo
    && cd /app/TensorRT/ \
    && export TRT_SOURCE=`pwd` \
    && mkdir -p build && cd build \
    && cmake .. \
        -DTRT_LIB_DIR=$TRT_LIBPATH/lib \
        -DTRT_OUT_DIR=`pwd`/out \
        -DCUDA_VERISON=11.3.0 \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SAMPLES=OFF \
    && make -j$(nproc)

# Install protobuf, onnx-tensort needs it
RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v3.8.0/protobuf-cpp-3.8.0.tar.gz \
    && tar -xzvf protobuf-cpp-3.8.0.tar.gz \
    && cd protobuf-3.8.0 \
    && ./configure \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# Build onnx-tenssort, so we can generate trt-models
RUN cd /app/onnx-tensorrt \
	&& mkdir build && cd build \
	&& cmake .. -DTENSORRT_ROOT=${TENSORRT_INSTALL} \
    && make -j$(nproc) \
    && make install

# Install ROS noetic on Ubuntu (http://wiki.ros.org/noetic/Installation/Ubuntu)
ENV ROS_DISTRO=noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN apt update && apt install -y --no-install-recommends python3-catkin-tools

# [Spconv]
RUN pip install spconv-cu113

# [OpenPCDet]
# Building this requires dirty tricks.
# Copy .git, because for some reason it is needed:
# /app/OpenPCDet/tools/onnx_utils# python3 trans_pfe.py
# fatal: not a git repository: /app/OpenPCDet/../.git/modules/OpenPCDet
#
# Make CUDA detectable, otherwise we get following error while building openpcdet:
# Found no NVIDIA driver on your system. Please check that you
# have an NVIDIA GPU and installed a driver from...
COPY OpenPCDet /app/OpenPCDet/
COPY .git /app/.git/
ENV TORCH_CUDA_ARCH_LIST=Turing

# Then start building with instructions given in the Makefile
COPY Makefile /app/Makefile
RUN cd /app && make build-openpcdet

# Finally, move on to the Pointpillars project itself
COPY src /app/src/
COPY PointPillars /app/PointPillars/

################################################################################
FROM dependency-stage AS development-stage

# Build pointpillars library
RUN /bin/bash -c "cd /app/PointPillars/ && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Debug -DTENSORRT_ROOT=${TENSORRT_INSTALL} && \
    make -j$(nproc)"

# Build ROS detector
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /app && \
    catkin config --init --install && \
    catkin clean -yb && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DTENSORRT_ROOT=${TENSORRT_INSTALL}"

USER pointpillars
WORKDIR /app

################################################################################
FROM dependency-stage AS release-stage

# Build pointpillars library
RUN /bin/bash -c "cd /app/PointPillars/ && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DTENSORRT_ROOT=${TENSORRT_INSTALL} && \
    make -j$(nproc) && \
    make install"

# Build ROS detector
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /app && \
    catkin config --init --install && \
    catkin clean -yb && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DTENSORRT_ROOT=${TENSORRT_INSTALL}"

################################################################################
FROM ros:noetic-ros-core AS production-stage

RUN useradd -ms /bin/bash pointpillars

# Copy needed libraries and binaries for running the app
COPY --from=release-stage /app/install/ /app/install/
COPY --from=dependency-stage /usr/local/cuda/bin /usr/local/cuda/bin
COPY --from=dependency-stage /usr/local/cuda/lib64 /usr/local/cuda/lib64
COPY --from=dependency-stage /app/TensorRT-8.0.3.4/lib /app/TensorRT-8.0.3.4/lib
COPY --from=dependency-stage /app/TensorRT-8.0.3.4/bin /app/TensorRT-8.0.3.4/bin

COPY --from=dependency-stage \
    /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.6 \
    /usr/lib/x86_64-linux-gnu/libcudnn.so.8 \
    /usr/lib/x86_64-linux-gnu/libcudnn_ops_infer.so.8 \
    /usr/lib/x86_64-linux-gnu/libcudnn_cnn_infer.so.8 \
    /usr/lib/x86_64-linux-gnu/

ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/app/TensorRT-8.0.3.4/lib:/usr/local/cuda/lib64"
ENV PATH=${PATH}:/usr/local/cuda/bin

USER pointpillars
ENTRYPOINT bash -c "cd /app/ && source install/setup.bash && roslaunch lidar_detector lidar_node.launch"
