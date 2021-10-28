# point-pillars
A fast PointPillar-Multihead lidar detector

## Short guide
1. `$ make setup` to clone submodules
2. Download TensorRT-7.1 GA and place to repo root
3. `$ make docker-build`  
4. Obtain model files and place to `config/`
5. `$ make docker-launch`  

ROS node and model should be running now. If not, then continue to more detailed guide.

## Installation
First, setup the repository by calling: `make setup`
This downloads submodules, so docker build doesn't need to always clone them from remotes.

Then, download [TensorRT 7.1.3.4 GA](https://developer.nvidia.com/nvidia-tensorrt-7x-download) (Ubuntu 1804 CUDA 10.2) TAR package and place
it into the root of this repository, from where docker is able to find it.
(So, TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz goes to point-pillars/)
This must be done manually, because downloading requires personal nvidia account. File should be placed to the repository root, so it exists inside the docker build context.

- Download pretrained [pytorch model](https://drive.google.com/file/d/1p-501mTWsq0G9RzroTWSXreIMyTUUpBM/view?usp=sharing)

- Download example [lidar data](https://drive.google.com/file/d/1KD0LT0kzcpGUysUu__dfnfYnHUW62iwN/view?usp=sharing)

- create a docker container: `make docker-build` and `make docker-launch`.

Run, `make generate-onnx` and `make generate-trt`. At this point,
you should have ONNX and TRT models in `config` directory and you have already inferenced with the model. Everything is setup. If you want to visualize inference without ROS,
download and install `open3d` to your local system (outside docker) and call `make visualize`.
We want to avoid installing open3d inside container as this introduces a lot of new unnecessary
dependencies just for rendering the detector output. There are chances that you have many of these open3d dependencies installed already on your system.

## Visualization
If you want to visualize detections, go to `src/rviz_detections` and build the visualizer image with build command given below. Then, you can launch a container that runs the visualizer node by using given make command in the repository root. 
```
$ docker build -t rviz-detections .
$ make docker-launch-viz
```
Now, if objects are detected, the visualizer node publishes a corresponding bounding box as a marker array, which can be visualized with Rviz.

## Repo structure
```
├── config
├── onnx-tensorrt
├── OpenPCDet
├── PointPillars
├── spconv
├── src
├── TensorRT
├── TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz
└── third_party
```

ROS code is inside `src/` and pointpillars detector library lies in `Pointpillars/` directory. Inside `config/`, you should have configuration files and the model. Other directories include dependencies.

Note: it is better to duplicate `yaml` files to `config` directory and use these instead of modifying OpenPCDet configs. This way, we can easily mount config and use external files to adjust detector behaviour inside container online. Moreover, config changes do not then bust the docker build cache.

### Manual build
Project is picky about versions. We don't want to replace system libraries with older versions, so dependencies has to be built from source and their location must be specified manually with environment/build variables.

Many of dependencies are included as submodules, so you don't need to go through finding versions that happen to work togehter. First, build all dependencies. You may use existing make command to build modules from source.

User must set `TENSORRT_ROOT`, before pointpillars can be built succesfully. Variable can be set from terminal:
`export TENSORRT_ROOT=/path/to/TensorRT-7.1.3.4`. Then, build the pointpillars library.

Finally, build ROS detector with `catkin`. Now you can review `Dockerfile` for guidance.

## Q.A.

 fatal error: NvInfer.h: No such file or directory
 #include "NvInfer.h"

 TensorRt TCheck that `TENSORRT_ROOT` is pointing to correct location.

### Tasks:
- Move submodules inside `third_party`
- Try to update CUDA 11
- Instead of building whole protobuf from source, copy only C++ zip.
- Update this readme and paths