###################################################################
# Build: $ docker build -f Dockerfile -t pointpillars .
# Run:   $ docker run --gpus all -it pointpillars
#
# Keywords: CUDA 10.2, cudnn 7 & 8, TensorRT-7.1.3.4, ubuntu18.04
###################################################################

# Path to binary release. Assumes it to be in repo root by default.
ARG TENSORRT=TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz

# nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04
FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04 AS base

SHELL ["/bin/bash", "-c"]

# Install common build tools
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
    && ldconfig

# Build onnx-tenssort, so we can generate trt-models
RUN cd /app/onnx-tensorrt \
	&& mkdir build && cd build \
	&& cmake .. -DTENSORRT_ROOT=/app/TensorRT-7.1.3.4 && make -j \
	&& export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH \
    && make install

# Make onnx2trt findable from anywhere
ENV PATH "$PATH:/usr/local/bin"

# Copy project related sources
COPY PointPillars_MultiHead_40FPS /app/PointPillars_MultiHead_40FPS/
COPY OpenPCDet /app/OpenPCDet/
COPY spconv /app/spconv/
COPY Makefile /app/Makefile

# Build sources
#RUN cd /app && make build

WORKDIR /app
