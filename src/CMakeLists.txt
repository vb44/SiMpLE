cmake_minimum_required(VERSION 3.0)

project(SiMpLE)

add_compile_options(-std=c++17 -O3)
set(CMAKE_CXX_STANDARD 17)

# find required packages
find_package(nanoflann REQUIRED)
find_package(TBB REQUIRED)
find_package(dlib REQUIRED)
find_package(Threads REQUIRED)

# add an executable called simple
add_executable(simple main.cpp helper.cpp objectiveFunction.cpp)

# make sure the include path is used
target_link_libraries(simple nanoflann::nanoflann)
target_link_libraries(simple dlib::dlib)
target_link_libraries(simple TBB::tbb)

# for this example to find "utils.h"
target_include_directories(simple PRIVATE ".")