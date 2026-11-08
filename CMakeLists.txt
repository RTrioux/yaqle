cmake_minimum_required(VERSION 3.17.3)
get_filename_component(ROOT ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(YAQLE_USE_COUT "Enable cout-based output" OFF)

Include(FetchContent)

set(PROJECT_NAME ${ROOT})
project(${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(Colors)
message(STATUS "${Green}Processing " ${ROOT} ${ColourReset})
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/_deps/etl-src/include)

#file(GLOB SOURCES "src/*.cpp")
set(SOURCES "src/main.cpp" "src/vector3d.cpp" "src/quat.cpp"  "src/dquat.cpp")

FetchContent_Declare(
    etl
    GIT_REPOSITORY https://github.com/ETLCPP/etl
    GIT_TAG 20.49.0
)

message(STATUS "Fetching ETL")

FetchContent_MakeAvailable(etl)

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

if(YAQLE_BUILD_TESTS)
    include(Tests)
endif()

# Lib generation (static: lib<name>.a, shared: lib<name>.so)
set(LIB_SOURCES src/quat.cpp src/vector3d.cpp src/dquat.cpp)
set(LIB_TARGETS lib${PROJECT_NAME})
add_library(lib${PROJECT_NAME} STATIC ${LIB_SOURCES})

# Bare-metal targets (e.g. arm-none-eabi) cannot build shared libraries
get_property(SUPPORTS_SHARED GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS)
if(SUPPORTS_SHARED)
    add_library(lib${PROJECT_NAME}_shared SHARED ${LIB_SOURCES})
    set(LIB_TARGETS ${LIB_TARGETS} lib${PROJECT_NAME}_shared)
endif()

foreach(LIB ${LIB_TARGETS})
    target_link_libraries(${LIB} PRIVATE etl::etl)

    if(YAQLE_USE_COUT)
        target_compile_definitions(${LIB} PUBLIC YAQLE_USE_COUT)
    endif()

    target_compile_definitions(${LIB} PUBLIC
        ETL_NO_STL=1
        ETL_USING_STL=0
        ETL_FORCE_STD_INITIALIZER_LIST=1
    )

    if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        target_compile_options(${LIB} PUBLIC -Og -g)
        target_compile_definitions(${LIB} PUBLIC ETL_USING_EXCEPTIONS=1)
    elseif("${CMAKE_BUILD_TYPE}" STREQUAL "ReleaseWithException")
        target_compile_options(${LIB} PUBLIC -O1)
        target_compile_definitions(${LIB} PUBLIC ETL_USING_EXCEPTIONS=1)
    else()
        target_compile_options(${LIB} PUBLIC -O1)
        target_compile_definitions(${LIB} PUBLIC ETL_NO_CHECKS=1)
    endif()

    set_target_properties(${LIB} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
endforeach()
