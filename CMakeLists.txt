cmake_minimum_required(VERSION 3.17.3)
get_filename_component(ROOT ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME ${ROOT})
project(${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(Colors)
message(STATUS "${Green}Processing "  ${ROOT} ${ColourReset})
include_directories(inc)


file(GLOB SOURCES "src/*.cpp")

#add_library(ginac SHARED IMPORTED)
#set(IMPORTED_LOCATION /usr/lib)


#message(STATUS "PROJECT_NAME =" ${PROJECT_NAME})
#message(STATUS "Sources =" ${SOURCES})
#message(STATUS "IMPORTED_LOCATION =" ${IMPORTED_LOCATION})
#message(STATUS "ROOT =" ${ROOT})

add_executable(${PROJECT_NAME} ${SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

add_library(lib${PROJECT_NAME} src/quat.cpp src/vector3d.cpp)
set_target_properties(lib${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

