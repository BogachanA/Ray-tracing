cmake_minimum_required(VERSION 3.15)
project(ray_tracing_example)

set(CMAKE_CXX_STANDARD 14)


add_executable(ray_tracing_example hittable.h rtworld.h task6.cpp aabb.h)
include_directories(glm)