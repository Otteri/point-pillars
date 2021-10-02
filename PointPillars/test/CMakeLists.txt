add_subdirectory(gtest)

include_directories(
    ./gtest/googletest/include
    ${TENSORRT_ROOT}/include/
)

set(test_src
  run_all.cc
  pointpillars_test.cc
)

add_executable(test_model ${test_src})

target_link_libraries(test_model
  gtest
  ${LINK_LIBRARY} 
  ${PROJECT_LIB_NAME}
)

add_test(
  NAME tests
  COMMAND test_model
)
