cmake_minimum_required(VERSION 4.0)
project(graphics_project)

set(CMAKE_CXX_STANDARD 23)

add_executable(graphics_project src/main.cpp)

add_subdirectory(third_party)

embed_shaders(
        NAME graphics_project
        SHADERS "main.vert:${CMAKE_CURRENT_SOURCE_DIR}/src/main.vert" "main.frag:${CMAKE_CURRENT_SOURCE_DIR}/src/main.frag"
)

target_include_directories(graphics_project PRIVATE third_party)
target_link_libraries(graphics_project Ende Canta graphics_project_SHADERS)