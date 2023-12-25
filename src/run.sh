#
#   device_3d
#   created by Ilya Shishkin
#   cortl@8iter.ru
#   https://github.com/cortl0/device_3d
#   licensed by GPL v3.0
#

rm -r build;
echo "build device-3d";
mkdir build;
cd build;
cmake ..;
make -j6;
echo "run device-3d-cpu";
cd device_3d/cpu;
./device-3d-cpu;
echo $?;
cd ../..;
echo "run device-3d-cuda";
cd device_3d/gpu/cuda;
./device-3d-cuda;
echo $?;
cd ../../..;
echo "end";
cd ..;
