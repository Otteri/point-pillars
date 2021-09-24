###################################################################
# Build: $ docker build -f Dockerfile -t pointpillars .
# Run:   $ docker run --gpus all -it pointpillars
#
# Keywords: CUDA 10.2, cudnn7, TensorRT-7.1.3.4, ubuntu18.04
###################################################################

# nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04
FROM nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04 AS base

SHELL ["/bin/bash", "-c"]

# Install common build tools
## --assume-yes is for git
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

# Copy sources
COPY PointPillars_MultiHead_40FPS /app/PointPillars_MultiHead_40FPS/
COPY OpenPCDet /app/OpenPCDet/
COPY spconv /app/spconv/
COPY onnx-tensorrt /app/onnx-tensorrt/
COPY TensorRT /app/TensorRT/
#COPY protobuf /app/protobuf/
COPY Makefile /app/Makefile
COPY .git/ /app/.git
COPY .gitmodules /app/.gitmodules

# Get third party repos
# For example, we need certain version of protobuf, shipped with TensorRT for building it
RUN cd /app && git submodule update --recursive --init

# First get the TensorRT binary release
COPY TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz /app/
RUN tar -xvzf /app/TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz -C /app/ \
    && rm /app/TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz \
    && export TRT_RELEASE=/app/TensorRT-7.1.3.4 \
    # Next build steps are done in TensorRT repo
    && cd /app/TensorRT/ \
    && export TRT_SOURCE=`pwd` \
    && mkdir -p build && cd build \
    && cmake .. -DTRT_LIB_DIR=$TRT_RELEASE/lib -DTRT_OUT_DIR=`pwd`/out -DCUDA_VERSION=10.2 \
    && make -j$(nproc)

# Compile
# RUN \
#     make build-spconv

WORKDIR /app
