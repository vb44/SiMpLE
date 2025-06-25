cmake_minimum_required(VERSION 3.8)
project(simple_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_odometry src/lidar_odometry.cpp)
target_link_libraries(lidar_odometry PRIVATE rclcpp::rclcpp ${sensor_msgs_TARGETS})

target_include_directories(lidar_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(lidar_odometry PUBLIC c_std_99 cxx_std_17)

install(TARGETS lidar_odometry
  DESTINATION lib/${PROJECT_NAME})

ament_package()
