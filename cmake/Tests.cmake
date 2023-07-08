
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.13.0
)

# Testing
enable_testing()

if(NOT googletest_POPULATED)
    message(STATUS "Fetching Google Test ${googletest_VERSION}")

    # FetchContent_MakeAvailable(etl)
    FetchContent_Populate(googletest)

    add_subdirectory(${googletest_SOURCE_DIR} EXCLUDE_FROM_ALL)
endif()

set(TEST_SOURCES
    ${CMAKE_SOURCE_DIR}/tests/yaqle_tests.cpp
    ${CMAKE_SOURCE_DIR}/tests/quat/quat_tests.cpp
    ${CMAKE_SOURCE_DIR}/tests/dquat/dquat_tests.cpp
    ${CMAKE_SOURCE_DIR}/src/dquat.cpp
    ${CMAKE_SOURCE_DIR}/src/quat.cpp
    ${CMAKE_SOURCE_DIR}/src/vector3d.cpp
)
add_executable(tests EXCLUDE_FROM_ALL ${TEST_SOURCES})
target_link_libraries(tests PRIVATE gtest gtest_main)
target_include_directories(tests PUBLIC ${CMAKE_SOURCE_DIR}/tests)

target_compile_definitions(tests PUBLIC YAQLE_USE_COUT=1)