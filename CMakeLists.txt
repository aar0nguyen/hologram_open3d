cmake_minimum_required(VERSION 3.14)  

project(Open3DProject)

cmake_policy(SET CMP0074 NEW)  

set(CMAKE_PREFIX_PATH "/home/boolmon/Open3D/Open3D/build")

# Locate Open3D
find_package(Open3D REQUIRED)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Add executable target
add_executable(main src/main.cpp)

# Link Open3D library
target_link_libraries(main PRIVATE Open3D::Open3D)
