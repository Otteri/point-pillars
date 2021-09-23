###################################################################
# Build: $ docker build -f Dockerfile -t pointpillars .
# Run:   $ docker run --gpus all -it pointpillars
###################################################################

# nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04
FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04 AS base

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
    && ls \
    && tar -xzvf cmake-3.21.2.tar.gz\
    && cd cmake-3.21.2 \
    && ./bootstrap \
    && make -j$(nproc) \
    && make install

# Common python tools
RUN python3 -m pip install --upgrade pip && pip3 install --no-cache-dir \
    pathlib \
    wheel

# Install neede python packages
COPY requirements.txt /app/requirements.txt
RUN pip install -r /app/requirements.txt

# Copy sources
COPY PointPillars_MultiHead_40FPS /app/PointPillars_MultiHead_40FPS/
COPY OpenPCDet /app/OpenPCDet/
COPY spconv /app/spconv/
COPY onnx-tensorrt /app/onnx-tensorrt/
COPY TensorRT /app/TensorRT/
COPY protobuf /app/protobuf/
COPY Makefile /app/Makefile
COPY .git/ /app/.git
COPY .gitmodules /app/.gitmodules


# Compile
# RUN \
#     make build-spconv

WORKDIR /app/
