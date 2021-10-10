# point-pillars
A fast PointPillar-Multihead lidar detector

## Installation
First, setup the repository by calling: `make setup`
This downloads submodules, so docker build doesn't need to always clone them from remotes.

Then, download [TensorRT 7.1.3.4 GA](https://developer.nvidia.com/nvidia-tensorrt-7x-download) (Ubuntu 1804 CUDA 10.2) TAR package and place
it into the root of this repository, from where docker is able to find it.
(So, TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz goes to point-pillars/)
This must be done manually, because downloading requires personal nvidia account.

Download pretrained [pytorch model](https://drive.google.com/file/d/1p-501mTWsq0G9RzroTWSXreIMyTUUpBM/view?usp=sharing)

Download example [lidar data](https://drive.google.com/file/d/1KD0LT0kzcpGUysUu__dfnfYnHUW62iwN/view?usp=sharing)

Then, create a docker container: `make docker-build` and `make docker-launch`.

Run, `make generate-onnx` and `make generate-trt`. At this point,
you should have ONNX and TRT models in `model` directory and you have already inferenced with the model. Everything is setup. If you want to visualize,
download and install `open3d` to your local system (outside docker) and call `make visualize`.
We want to avoid installing open3d inside container as this introduces a lot of new unnecessary
dependencies just for rendering the output. There are chances that you have many of these open3d dependencies installed already on your system.


Note: it is better to place yaml-copies to PointPillars or model directory and use these instead of modifying OpenPCDet configs. This is because changes in OpenPCDet repo, force us to build the project again as docker cache gets busted.


## Env variables
Project is picky about versions. We don't want to replace system libraries with older versions, so dependencies has to be built from source and their location must be specified manually with environment/build variables.

User must set `TENSORRT_ROOT`, before pointpillars can be built succesfully. Variable can be set from terminal:
`export TENSORRT_ROOT=/path/to/TensorRT-7.1.3.4`.


## Repo structure

Initially structure looks like this
```
├── config
├── Dockerfile
├── onnx-tensorrt
├── OpenPCDet
├── PointPillars
├── spconv
├── src
├── TensorRT
└── third_party
```

But when you have gotten all configured and running,
there should be a few directories more
```
├── build
├── config
├── devel
├── logs
├── model
├── onnx-tensorrt
├── OpenPCDet
├── PointPillars
├── spconv
├── src
├── TensorRT
├── TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz
└── third_party
```

## Q.A

 fatal error: NvInfer.h: No such file or directory
 #include "NvInfer.h"

 TensorRt TCheck that `TENSORRT_ROOT` is pointing to correct location.
