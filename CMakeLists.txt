cmake_minimum_required(VERSION 3.17.3)
get_filename_component(ROOT ${CMAKE_CURRENT_SOURCE_DIR} NAME)

Include(FetchContent)

set(PROJECT_NAME ${ROOT})
project(${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(Colors)
message(STATUS "${Green}Processing " ${ROOT} ${ColourReset})
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/_deps/etl-src/include)

file(GLOB SOURCES "src/*.cpp")

FetchContent_Declare(
    etl
    GIT_REPOSITORY https://github.com/ETLCPP/etl
    GIT_TAG 20.36.1
)

if(NOT etl_POPULATED)
    message(STATUS "Fetching ETL ${etl_VERSION}")

    # FetchContent_MakeAvailable(etl)
    FetchContent_Populate(etl)

    add_subdirectory(${etl_SOURCE_DIR} EXCLUDE_FROM_ALL)
endif()

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE etl::etl)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

if(YAQLE_USE_COUT)
    target_compile_definitions(${PROJECT_NAME} PUBLIC YAQLE_USE_COUT)
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC
    ETL_NO_STL=1
    ETL_USING_STL=0
    ETL_FORCE_STD_INITIALIZER_LIST=1
)

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    message(STATUS "Compilation in Debug mode")
    target_compile_options(${PROJECT_NAME} PUBLIC -Og -g)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ETL_USING_EXCEPTIONS=1)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "ReleaseWithException")
    message(STATUS "Compilation in Release mode with exceptions")
    target_compile_options(${PROJECT_NAME} PUBLIC -O1)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ETL_USING_EXCEPTIONS=1)
else()
    message(STATUS "Compilation in Release mode")
    target_compile_options(${PROJECT_NAME} PUBLIC -O1)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ETL_NO_CHECKS=1)
endif()

include(Tests)

# Lib generation
# add_library(lib${PROJECT_NAME} src/quat.cpp src/vector3d.cpp)
# set_target_properties(lib${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})