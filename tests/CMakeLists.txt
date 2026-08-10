find_package(GTest REQUIRED)

set(TEST_SOURCES
    test_crash_monitor.cc
)

add_executable(orb_slam3_tests ${TEST_SOURCES})

# The top-level CMakeLists reassigns CMAKE_RUNTIME_OUTPUT_DIRECTORY many times
# for the example targets, so the last value would leak into this subdirectory.
# Pin the test binary next to the build tree instead.
set_target_properties(orb_slam3_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

target_include_directories(orb_slam3_tests PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(orb_slam3_tests
    ${PROJECT_NAME}
    GTest::gtest
    GTest::gtest_main
)

add_test(NAME orb_slam3_tests COMMAND orb_slam3_tests)
