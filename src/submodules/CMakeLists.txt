#
#   device_3d
#   created by Ilya Shishkin
#   cortl@8iter.ru
#   https://github.com/cortl0/device_3d
#   licensed by GPL v3.0
#

cmake_minimum_required(VERSION 3.0)

add_subdirectory(bnn/src/cpu)
add_subdirectory(bnn/src/gpu/cuda)
add_subdirectory(logger/src)
