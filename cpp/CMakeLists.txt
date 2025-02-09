cmake_minimum_required(VERSION 3.10)

project(SiMpLE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build by default in release mode
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find the required packages
find_package(nanoflann REQUIRED)
find_package(TBB REQUIRED) 
find_package(dlib REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Define source files
set(SOURCES
    src/main.cpp
    src/ObjectiveFunction.cpp
    src/ConfigParser.cpp
    src/PointCloud.cpp
    src/PointMap.cpp
    src/Register.cpp
    src/utils.cpp
)

# Add the executable
add_executable(simple ${SOURCES})

# Link required libraries
target_link_libraries(simple 
    nanoflann::nanoflann
    dlib::dlib
    TBB::tbb
    yaml-cpp::yaml-cpp
    Threads::Threads
)