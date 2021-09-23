# point-pillars
A fast PointPillar-Multihead lidar detector

# Installation
First, setup the repository by calling: `make setup`
This downloads submodules, so docker doesn't always need to clone them from remote.

Then, download TensorRT 7.1.3.4 GA (Ubuntu 1804 CUDA 10.2) TAR package and place
it into the root of this repository, from where docker is able to find it.
(So, TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz goes to point-pillars/)
This must be done manually, because downloading requires personal nvidia account.
Download from here: https://developer.nvidia.com/nvidia-tensorrt-7x-download

Then, everything else can be handled automatically. Just call: `make docker-build`

