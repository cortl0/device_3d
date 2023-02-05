#
#   device_3d
#   created by Ilya Shishkin
#   cortl@8iter.ru
#   https://github.com/cortl0/device_3d
#   licensed by GPL v3.0
#

cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(device-3d-cpu LANGUAGES CXX)

add_compile_definitions(BNN_ARCHITECTURE_CPU)
add_compile_definitions(DEVICE_3D_THREADS_QUANTITY=2)
include(${CMAKE_SOURCE_DIR}/CMakeLists_common.txt)
target_link_libraries(${PROJECT_NAME} PRIVATE bnn-cpu)
