cmake_minimum_required(VERSION 3.14)

project(simple)

add_compile_options(-std=c++17 -O3 -Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)

# Find the required packages
find_package(nanoflann REQUIRED)
find_package(TBB REQUIRED)
find_package(dlib REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# PGO
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

# Define source files (excluding main.cpp, so library is reusable)
set(SOURCES
    src/ObjectiveFunction.cpp
    src/ConfigParser.cpp
    src/PointCloud.cpp
    src/PointMap.cpp
    src/PoseGraphOptimization.cpp
    src/Register.cpp
    src/Scancontext.cpp
    src/utils.cpp
)

# Create a library target for the reusable code
add_library(simple_lib ${SOURCES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

target_include_directories(simple_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
)

# Link required libraries
target_link_libraries(simple_lib
  nanoflann::nanoflann
  dlib::dlib
  TBB::tbb
  yaml-cpp::yaml-cpp
  Threads::Threads
  ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

# Add the executable
add_executable(simple src/main.cpp)
target_link_libraries(simple simple_lib)
