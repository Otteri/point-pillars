cmake_minimum_required(VERSION 3.18)
project(lidar_detector)

option(TEST_LIDAR_DETECTOR              "BUILD TEST"                 ON)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    message("Release build, using O3 optimization...")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

message("Using TENSORRT_ROOT: ${TENSORRT_ROOT}")

find_package(catkin REQUIRED)
############################################################
# add 3th_part library
############################################################

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

find_package(Threads)
find_package(OpenMP REQUIRED)
add_definitions("-DELPP_NO_LOG_TO_FILE")

find_package(CUDA REQUIRED QUIET)
include_directories(${CUDA_INCLUDE_DIRS})
if (CUDA_VERSION VERSION_LESS "11.0")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++11 -Wno-deprecated-gpu-targets)
else ()
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++14 -Wno-deprecated-gpu-targets)
endif ()

#################
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PointPillars/
    ${TENSORRT_ROOT}/include/
)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    sensor_msgs
    message_filters
    rviz_detections
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
  roscpp
)

## Catkin package definition
catkin_package(
  INCLUDE_DIRS .
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(${catkin_INCLUDE_DIRS})

set(src
    main.cc
    lidar_node.cc
    pclbuff.cc
)

add_executable(${PROJECT_NAME} ${src})

target_link_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PointPillars/build/lib
    ${TENSORRT_ROOT}/lib # libnvonxxparser, libnvinfer, ...
)

set(PROJECT_LIBS
  ${CUDA_LIBRARIES}
  #pointpillars
  ${CMAKE_CURRENT_SOURCE_DIR}/../../PointPillars/build/lib/libpointpillars.so
  yaml-cpp
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS lidar_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

message("TEST_LIDAR_DETECTOR: ${TEST_LIDAR_DETECTOR}")
#############
## Testing ##
#############
if (TEST_LIDAR_DETECTOR)
    enable_testing()
    add_subdirectory("test")
endif ()
