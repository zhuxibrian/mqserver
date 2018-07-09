cmake_minimum_required(VERSION 3.10)
project(mqserver)

set(CMAKE_CXX_STANDARD 11)

add_executable(mqserver main.cpp robot_state.h robot_state.cpp memUtil.hpp
        include/endian.h include/global.h include/xmlStructs.h)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.67.0 REQUIRED COMPONENTS thread system filesystem)

#添加头文件搜索路径
include_directories(/usr/local/include)

#添加库文件搜索路径
link_directories(/usr/local/lib)

target_link_libraries(mqserver ${Boost_LIBRARIES})