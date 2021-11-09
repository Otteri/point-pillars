# If TENSORRT_ROOT is not set, default to docker app-path
TENSORRT_ROOT := $(or $(TENSORRT_ROOT),/app/TensorRT-7.1.3.4)

setup: ## Setup environment
	git submodule update --recursive --init

docker-setup:
	cp ${TENSORRT_ROOT}/include/* /usr/include/
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${TENSORRT_ROOT}/lib

docker-build: ## Build production image
	docker build --target production-stage -t pointpillars:latest .

docker-launch: ## Launch pointpillars application container
	docker run \
	--gpus all \
	--rm \
	-it \
	-v `pwd`/config:/app/config/ \
	-v `pwd`/config/lidar_node.launch:/app/install/share/lidar_detector/launch/lidar_node.launch \
	--network=host \
	pointpillars:latest

docker-build-debug: ## Build debug docker image
	docker build --target debug-stage -t pointpillars-debug .

docker-launch-debug: ## Launch interactive debug container
	docker run \
	--gpus all \
	--rm \
	-it \
	-v `pwd`/config:/app/config/ \
	-v /home/cosmo/data/nuscenes:/app/OpenPCDet/data/nuscenes/ \
	-v `pwd`/output:/app/OpenPCDet/output/app/config/cbgs_pp_multihead/default/ \
	--network=host \
	pointpillars-debug

docker-launch-viz:
	docker run \
	-d \
	--rm \
	-v `pwd`/config/rviz_detections.launch:/app/install/share/rviz_detections/rviz_detections.launch \
	--network=host \
	rviz-detections:latest

build: ## Build all submodules and PointPillars
	${MAKE} build-spconv
	${MAKE} build-openpcdet
	${MAKE} build-pointpillars

# https://github.com/google/googletest/blob/release-1.11.0/googletest/README.md
build-googletest:
	cd ./third_party/googletest; \
	mkdir build && cd build; \
	cmake -DBUILD_GMOCK=OFF -DCMAKE_INSTALL_PREFIX=../../install ..; \
	make install

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
	cmake ..  -DTENSORRT_ROOT=${TENSORRT_ROOT} && make -j$(nproc)

test-pointpillars:  ## Runs pointpillar tests
	cd PointPillars/build; \
	./test/test_model

# https://github.com/hova88/PointPillars
# You must run test first, before running this
visualize-pointpillars:  ## Visualizes the test environment
	${MAKE} test-pointpillars; \
	cd PointPillars/tools; \
	python3 viewer.py

clean-pointpillars:
	rm -r PointPillars/build

# https://github.com/hova88/OpenPCDet#changelog
generate-onnx: ## Converts Pytorch model to ONNX
	cd ./OpenPCDet/tools/onnx_utils; \
	python3 trans_pfe.py; \
    python3 trans_backbone_multihead.py

# https://github.com/hova88/OpenPCDet#changelog
generate-trt: ## Converts ONNX model to TensorRT
	cd ./config; \
	onnx2trt cbgs_pp_multihead_pfe.onnx -o cbgs_pp_multihead_pfe.trt -b 1 -d 16; \
    onnx2trt cbgs_pp_multihead_backbone.onnx -o cbgs_pp_multihead_backbone.trt -b 1 -d 16

build-lidar-detector:
	catkin clean -yb; \

	make clean-pointpillars; \
	make build-pointpillars; \
	catkin build --cmake-args -DTENSORRT_ROOT=${TENSORRT_ROOT} -DCMAKE_BUILD_TYPE=Release

test-lidar-detector:
	cd build/lidar_detector; \
	make run_tests

test-lidar-detector-with-text:
	rostest lidar_detector lidar_node.test --text

deploy:
	mkdir deploy;\
	cp Makefile deploy/;\
	cp -r config/ deploy/;\
	cp src/rviz_detections/launch/rviz_detections.launch deploy/config/rviz_detections.launch;\
	docker save -o `pwd`/deploy/pointpillars.tar pointpillars:latest;\
	docker save -o `pwd`/deploy/rviz-detections.tar rviz-detections:latest

help: ## Display callable targets.
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'
