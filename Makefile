setup: ## Setup environment
	git submodule update --recursive --init

docker-build: ## Build docker image
	docker build -f Dockerfile -t pointpillars .

docker-launch: ## Launch interactive docker container
	docker run --gpus all -it pointpillars

build: ## Build all submodules and PointPillars
	${MAKE} setup
	${MAKE} build-spconv

build-spconv: ## Build spconv (3rd party dependency)
	cd ./spconv; \
	python3 setup.py bdist_wheel; \
	cd ./dist && pip3 install *

build-openpcdet: ## Build OpenPCDet (3rd party dependency)
	cd ./OpenPCDet; \
	pip3 install -r requirements.txt; \
	python3 setup.py develop

build-protobuf: ## Build protobuf (3rd party dependency)
	cd ./protobuf; \
	./autogen.sh; \
	./configure; \
	make; \
	make check; \
	make install; \
	ldconfig

help: ## Display callable targets.
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'
