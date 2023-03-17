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
cd cpu;
./device-3d-cpu;
cd ..;
echo $?;
echo "run device-3d-cuda";
cd gpu/cuda;
./device-3d-cuda;
cd ../..;
echo $?;
echo "end";
cd ..;
