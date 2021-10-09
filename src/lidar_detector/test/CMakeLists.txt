#add_subdirectory(gtest)

message("Building lidar-detector tests")

include_directories(
    ../../../third_party/install/include/gtest
)

# set(test_src
#   pclbuff_test.cpp
# )

#add_executable(node_tester ${test_src})

# target_link_libraries(node_tester
#   gtest
#   ${LINK_LIBRARY} 
#   ${PROJECT_LIB_NAME}
# )

# target_link_libraries(node_tester
# ${CUDA_LIBRARIES}
# pointpillars
# yaml-cpp
# ${catkin_LIBRARIES}
# )

# add_test(
#   NAME tests
#   COMMAND node_tester
# )


find_package(rostest REQUIRED)
add_rostest_gtest(
    lidar_node_tester
    ../launch/lidar_node_tester.launch
    pclbuff_test.cpp
)
