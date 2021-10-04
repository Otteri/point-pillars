###################################################################
# Author: Joni Airaksinen (Otteri)
# Build: $ docker build -f Dockerfile -t pointpillars .
# Run:   $ docker run --gpus all --rm -it \
#          -v `pwd`/config:/app/config/ --network=host pointpillars
#
# Keywords: CUDA 10.2, cudnn 7 & 8, TensorRT-7.1.3.4, ubuntu18.04
###################################################################

# Path to binary release. Assumes it to be in repo root by default.
ARG TENSORRT=TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz

FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04 AS dependency-stage

SHELL ["/bin/bash", "-c"]

# Install common build tools
# --assume-yes for git
RUN apt-get update && apt-get install -y --no-install-recommends --assume-yes \
    autoconf \
    automake \
    build-essential \
    curl \
    git \
    g++ \
    libboost-all-dev \
    libssl-dev \
    libtool \
    libyaml-cpp-dev \
    make \
    python3-pip \
    python3-setuptools \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install recent CMake version (apt doesn't have enough fresh)
RUN mkdir cmake && cd cmake \
    && wget https://cmake.org/files/v3.21/cmake-3.21.2.tar.gz \
    && tar -xzvf cmake-3.21.2.tar.gz \
    && cd cmake-3.21.2 \
    && ./bootstrap \
    && make -j$(nproc) \
    && make install

# Common python tools
RUN python3 -m pip install --upgrade pip && pip3 install --no-cache-dir \
    pathlib \
    wheel

# Install needed python packages
COPY requirements.txt /app/requirements.txt
RUN pip install -r /app/requirements.txt

# Copy Nvidia sources
COPY onnx-tensorrt /app/onnx-tensorrt/
COPY TensorRT /app/TensorRT/

# Install cudnn8, because TensorRT build requires it,
# but we want to still use cudnn7 with spconv
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=8.0.5.39-1+cuda10.2 \
    && rm -rf /var/lib/apt/lists/*

# First get the TensorRT binary release
ARG TENSORRT
COPY ${TENSORRT} /app/
RUN tar -xvzf /app/TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz -C /app/ \
    && rm /app/TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz \
    && export TRT_RELEASE=/app/TensorRT-7.1.3.4 \
    # Next build steps are done in TensorRT repo
    && cd /app/TensorRT/ \
    && export TRT_SOURCE=`pwd` \
    && mkdir -p build && cd build \
    && cmake .. -DTRT_LIB_DIR=$TRT_RELEASE/lib -DTRT_OUT_DIR=`pwd`/out -DCUDA_VERSION=10.2 \
    && make -j$(nproc)

# Build protobuf system wide (TensoRT and onnx-tensorrt dependency)
# We can use the version in TensorRT/third_party, so we don't need to
# duplicate the repo ourselves and this also ensures that we use good version
RUN cd /app/TensorRT/third_party/protobuf \
    && ./autogen.sh \
    && ./configure \
    && make -j$(nproc) \
    && make check \
    && make install \
    && cp /app/TensorRT-7.1.3.4/include/* /usr/include/ \
    && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/app/TensorRT-7.1.3.4/lib \
    && ldconfig

# Build onnx-tenssort, so we can generate trt-models
RUN cd /app/onnx-tensorrt \
	&& mkdir build && cd build \
	&& cmake .. -DTENSORRT_ROOT=/app/TensorRT-7.1.3.4 && make -j$(nproc) \
	&& export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH \
    && make install

# Install ROS melodic on Ubuntu (http://wiki.ros.org/melodic/Installation/Ubuntu)
RUN /bin/sh -c echo 'Etc/UTC' > /etc/timezone &&     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime &&     apt-get update &&     apt-get install -q -y --no-install-recommends tzdata &&     rm -rf /var/lib/apt/lists/*
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-ros-base
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN apt update && apt install -y --no-install-recommends python3-catkin-tools

# Copy project related sources
COPY OpenPCDet /app/OpenPCDet/
COPY spconv /app/spconv/
COPY Makefile /app/Makefile

# Run separately for better caching
RUN cd /app && make build-spconv

# For some reason OpenPCDet needs .git:
# /app/OpenPCDet/tools/onnx_utils# python3 trans_pfe.py
# fatal: not a git repository: /app/OpenPCDet/../.git/modules/OpenPCDet
COPY .git /app/.git/

# AS a last thing, the project
# So when we do changes here, we need to redo only one layer
COPY src /app/src/
COPY PointPillars /app/PointPillars/


################################################################################
FROM dependency-stage as debug-stage

# Build pointpillars library
RUN /bin/bash -c "cd /app/PointPillars/ && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Debug -DTENSORRT_ROOT=/app/TensorRT-7.1.3.4 && \
    make -j$(nproc) && \
    make install"

# Build ROS detector
RUN /bin/bash -c "source /opt/ros/melodic/setup.sh && \
    cd /app && \
    catkin config --init --install && \
    catkin clean -yb && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DTENSORRT_ROOT=/app/TensorRT-7.1.3.4"

WORKDIR /app


################################################################################
FROM dependency-stage as release-stage

# Build pointpillars library
RUN /bin/bash -c "cd /app/PointPillars/ && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DTENSORRT_ROOT=/app/TensorRT-7.1.3.4 && \
    make -j$(nproc) && \
    make install"

# Build ROS detector
RUN /bin/bash -c "source /opt/ros/melodic/setup.sh && \
    cd /app && \
    catkin config --init --install && \
    catkin clean -yb && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DTENSORRT_ROOT=/app/TensorRT-7.1.3.4"


################################################################################
FROM ros:melodic-ros-core as production-stage

# Copy needed libraries and binaries for running the app
COPY --from=release-stage /app/install/ /app/install/
COPY --from=dependency-stage /usr/local/cuda/bin /usr/local/cuda/bin
COPY --from=dependency-stage /usr/local/cuda/lib64 /usr/local/cuda/lib64
COPY --from=dependency-stage /app/TensorRT-7.1.3.4/lib /app/TensorRT-7.1.3.4/lib
COPY --from=dependency-stage /app/TensorRT-7.1.3.4/bin /app/TensorRT-7.1.3.4/bin

COPY --from=dependency-stage \
    /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.5 \
    /usr/lib/x86_64-linux-gnu/libcublas.so.10 \
    /usr/lib/x86_64-linux-gnu/libcudnn.so.8 \
    /usr/lib/x86_64-linux-gnu/libcublasLt.so.10 \
    /usr/lib/x86_64-linux-gnu/libcudnn_ops_infer.so.8 \
    /usr/lib/x86_64-linux-gnu/libcudnn_cnn_infer.so.8 \
    /usr/lib/x86_64-linux-gnu/

ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/app/TensorRT-7.1.3.4/lib:/usr/local/cuda/lib64"
ENV PATH=${PATH}:/usr/local/cuda/bin

ENTRYPOINT bash -c "cd /app/ && source install/setup.bash && roslaunch lidar_detector lidar_node.launch"
