project(lidar_detector)

find_package(catkin REQUIRED)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    sensor_msgs
    message_filters
    vision_msgs
    odts_msgs
    #tf
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
  roscpp
  pcl_conversions
  pcl_ros
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
    #detector.cc
)

## Make definition of project libs
set(PROJECT_LIBS
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME} ${src})

target_link_libraries(${PROJECT_NAME}
  ${LINK_LIBRARY} 
  ${PROJECT_LIB_NAME}
  ${catkin_LIBRARIES}
)
