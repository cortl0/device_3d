#
#   device_3d
#   created by Ilya Shishkin
#   cortl@8iter.ru
#   http://8iter.ru/ai.html
#   https://github.com/cortl0/device_3d
#   licensed by GPL v3.0
#

TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
bnn/src/common/sources/brain_tools.cpp \
application/application.cpp \
application/tripod.cpp \
conductors/conductor.cpp \
conductors/conductor_circle.cpp \
conductors/dream.cpp \
conductors/kick.cpp \
conductors/tail.cpp \
creatures/bike/bike.cpp \
creatures/creature.cpp \
creatures/table/joint.cpp \
creatures/table/leg.cpp \
creatures/table/table.cpp \
data_processing_methods/data_processing_method.cpp \
data_processing_methods/data_processing_method_binary.cpp \
data_processing_methods/data_processing_method_linearly.cpp \
data_processing_methods/data_processing_method_linearly_single.cpp \
data_processing_methods/data_processing_method_logarithmic.cpp \
main.cpp \
physical_objects/cube.cpp \
physical_objects/figure.cpp \
physical_objects/sphere.cpp \
scenes/bike_scene.cpp \
scenes/scene.cpp \
scenes/table_scene.cpp \
sensors/acceleration.cpp \
sensors/distance.cpp \
sensors/gyroscope.cpp \
sensors/time.cpp \
sensors/velocity.cpp \
sensors/video.cpp \
teachers/teacher.cpp \
teachers/teacher_walking.cpp

HEADERS += \
sensors/acceleration.h \
sensors/distance.h \
sensors/gyroscope.h \
sensors/time.h \
sensors/velocity.h \
sensors/video.h

INCLUDEPATH += \
/usr/include/boost/system \
/usr/local/include/ode \
/usr/local/include/OGRE \
/usr/local/include/OGRE/Bites \
/usr/local/include/OGRE/RTShaderSystem \
/usr/local/include/OGRE/Overlay \
bnn/src/bnn_cpu/headers \
bnn/src/common/headers \
./ \
application \
conductors \
creatures \
creatures/bike \
creatures/table \
data_processing_methods \
physical_objects \
scenes \
teachers

LIBS += \
-lX11 \
-lstdc++fs \
-lpthread \
-lboost_system \
-lode \
-lOgreBites \
-lOgreMain \
-lOgreRTShaderSystem \
-lOgreOverlay \
$${_PRO_FILE_PWD_}/bnn/src/bnn_cpu/build/libbnn_cpu.a
