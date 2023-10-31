cmake_minimum_required(VERSION 3.0)

project(evaluateOdom)

add_compile_options(-std=c++17 -O3)

# add an executable called evaluate_odometry
add_executable(evaluate_odometry evaluate_odometry.cpp matrix.cpp)