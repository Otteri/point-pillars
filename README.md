# point-pillars
A fast PointPillar-Multihead lidar detector

# Installation
First, setup the repository by calling: `make setup`
This downloads submodules, so docker build doesn't always need to clone them from remote.

Then, download [TensorRT 7.1.3.4 GA](https://developer.nvidia.com/nvidia-tensorrt-7x-download) (Ubuntu 1804 CUDA 10.2) TAR package and place
it into the root of this repository, from where docker is able to find it.
(So, TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz goes to point-pillars/)
This must be done manually, because downloading requires personal nvidia account.


Download pretrained [pytorch model](https://drive.google.com/file/d/1p-501mTWsq0G9RzroTWSXreIMyTUUpBM/view?usp=sharing)

Download example [lidar data](https://drive.google.com/file/d/1KD0LT0kzcpGUysUu__dfnfYnHUW62iwN/view?usp=sharing)

Then, create a docker container: `make docker-build` and `make docker-launch`.
Build spconv, OpenPCDet and PointPillars by running: `make build` inside container.

Then, `make generate-onnx` and `make generate-trt`. At this point,
you should have ONNX and TRT models in `model` directory and you have already done inference with the model. Everything is setup. If you want to visualize,
download and install `open3d` on your local system (outside docker) and call `make visualize`.
We want to avoid installing open3d inside container as this introduces a lot of new unnecessary
dependencies just for rendering the output. There are chances that you have many of these open3d dependencies installed already on your system.


Note: it is better to place yaml-copies to PointPillars or model directory and use these instead of modifying OpenPCDet configs. This is because changes in OpenPCDet repo, force us to build the project again as docker cache gets busted.
