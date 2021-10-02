setup: ## Setup environment
	git submodule update --recursive --init

docker-setup:
	cp /app/TensorRT-7.1.3.4/include/* /usr/include/
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/app/TensorRT-7.1.3.4/lib

docker-build: ## Build production image
	docker build --target production-stage -t pointpillars .

docker-launch: ## Launch pointpillars application container
	docker run \
	--gpus all \
	--rm \
	-it \
	-v `pwd`/config:/app/config/ \
	--network=host \
	pointpillars

docker-build-debug: ## Build debug docker image
	docker build --target debug-stage -t pointpillars-debug .

docker-launch-debug: ## Launch interactive debug container
	docker run \
	--gpus all \
	--rm \
	-it \
	-v `pwd`/config:/app/config/ \
	--network=host \
	pointpillars-debug

build: ## Build all submodules and PointPillars
	${MAKE} build-spconv
	${MAKE} build-openpcdet
	${MAKE} build-pointpillars

# https://github.com/traveller59/spconv
build-spconv: ## Build spconv (3rd party dependency)
	cd ./spconv; \
	python3 setup.py bdist_wheel; \
	cd ./dist && pip3 install *

# https://github.com/hova88/OpenPCDet/blob/master/docs/INSTALL.md
build-openpcdet: ## Build OpenPCDet (3rd party dependency)
	cd ./OpenPCDet; \
	pip3 install -r requirements.txt; \
	python3 setup.py develop

# https://github.com/hova88/PointPillars
build-pointpillars: ## Builds PointPillars
	cd PointPillars; \
	mkdir build && cd build; \
	cmake .. -DNVONNXPARSERS=/app/TensorRT-7.1.3.4/lib/libnvonnxparser.so -DNVINFER=/app/TensorRT-7.1.3.4/lib/libnvinfer.so && make -j$(nproc) && ./test/test_model

# https://github.com/hova88/OpenPCDet#changelog
generate-onnx: ## Converts Pytorch model to ONNX
	cd ./OpenPCDet/tools/onnx_utils; \
	python3 trans_pfe.py; \
    python3 trans_backbone_multihead.py

# https://github.com/hova88/OpenPCDet#changelog
generate-trt: ## Converts ONNX model to TensorRT
	cd ./model; \
	onnx2trt cbgs_pp_multihead_pfe.onnx -o cbgs_pp_multihead_pfe.trt -b 1 -d 16; \
    onnx2trt cbgs_pp_multihead_backbone.onnx -o cbgs_pp_multihead_backbone.trt -b 1 -d 16

# https://github.com/hova88/PointPillars
visualize:
	cd PointPillars/tools; \
	python3 viewer.py

help: ## Display callable targets.
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'
