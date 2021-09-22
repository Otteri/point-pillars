##########################################¤¤
# Build: $ docker build -f Dockerfile -t pointpillars .
# Run:   $ docker run --gpus all -it pointpillars
############################################

FROM nvcr.io/nvidia/pytorch:20.02-py3 AS base

RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*

# Copy sources
COPY PointPillars_MultiHead_40FPS /app/PointPillars_MultiHead_40FPS
COPY OpenPCDet /app/OpenPCDet
COPY spconv /app/spconv
COPY onnx-tensorrt /app/onnx-tensorrt
COPY TensorRT /app/TensorRT
COPY protobuf /app/protobuf

WORKDIR /app/
