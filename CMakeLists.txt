cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0077 NEW)


project(Open3DProject)

set(CMAKE_PREFIX_PATH "/home/boolmon/Open3D/Open3D/build")

# Locate Open3D
find_package(Open3D REQUIRED)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Add executable target
add_executable(main_voxel_model_cartesian src/main_voxel_model_cartesian.cpp)
add_executable(main_voxel_model_cylinder src/main_voxel_model_cylinder.cpp)


# Link Open3D library
target_link_libraries(main_voxel_model_cartesian PRIVATE Open3D::Open3D)
target_link_libraries(main_voxel_model_cylinder PRIVATE Open3D::Open3D)
