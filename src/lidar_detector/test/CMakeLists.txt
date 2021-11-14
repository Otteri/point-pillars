#add_subdirectory(gtest)

message("Building lidar-detector tests")

include_directories(
    ../../../third_party/install/include/gtest
    ../
)

find_package(rostest REQUIRED)
add_rostest_gtest(
    lidar_node_tester
    ../launch/lidar_node.test
    pclbuff_test.cpp

    #
    ../pclbuff.cpp
)

target_link_libraries(lidar_node_tester ${PROJECT_LIBS})
