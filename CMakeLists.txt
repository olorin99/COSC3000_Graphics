cmake_minimum_required(VERSION 4.0)
project(graphics_project)

set(CMAKE_CXX_STANDARD 20)

add_executable(graphics_project src/main.cpp)

add_subdirectory(third_party)

target_link_libraries(graphics_project Ende Canta)