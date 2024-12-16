cmake_minimum_required(VERSION 3.10)

project(PIDdemo)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

aux_source_directory(. SRC_LIST)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

include_directories(${Python3_INCLUDE_DIRS})
link_libraries(${Python3_LIBRARIES})

include_directories(.)

#add_executable(demo main.cc pid.cc)
add_executable(demo ${SRC_LIST})