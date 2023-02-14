#
#   device_3d
#   created by Ilya Shishkin
#   cortl@8iter.ru
#   https://github.com/cortl0/device_3d
#   licensed by GPL v3.0
#

cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(device-3d-cuda LANGUAGES CXX CUDA)

include_directories(
    ${CMAKE_SOURCE_DIR}/bnn/src/gpu/cuda
    ${CMAKE_SOURCE_DIR}/bnn/src/gpu/cuda/external
    /usr/local/cuda
    )

add_compile_definitions(BNN_ARCHITECTURE_CUDA)
add_compile_definitions(DEVICE_3D_THREADS_QUANTITY_IN_POWER_OF_TWO=4)
add_compile_definitions(DEVICE_3D_NEURONS_QUANTITY_IN_POWER_OF_TWO=16)
add_compile_definitions(DEVICE_3D_RANDOM_SIZE_IN_POWER_OF_TWO=26)
include(${CMAKE_SOURCE_DIR}/CMakeLists_common.txt)
target_link_libraries(${PROJECT_NAME} PRIVATE bnn-cuda)
