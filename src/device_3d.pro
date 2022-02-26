#
#   device_3d
#   created by Ilya Shishkin
#   cortl@8iter.ru
#   https://github.com/cortl0/device_3d
#   licensed by GPL v3.0
#

TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        bnn/src/brain/brain.cpp \
        bnn/src/brain_tools.cpp \
        bnn/src/brain/m_sequence.cpp \
        bnn/src/brain/neurons/binary.cpp \
        bnn/src/brain/neurons/motor.cpp \
        bnn/src/brain/neurons/neuron.cpp \
        bnn/src/brain/neurons/sensor.cpp \
        bnn/src/brain/random/random.cpp \
        bnn/src/brain/storage.cpp \
        bnn/src/brain/thread.cpp \
        conductors/conductor.cpp \
        conductors/conductor_circle.cpp \
        creatures/creature.cpp \
        creatures/joint.cpp \
        creatures/leg.cpp \
        creatures/sensors/acceleration.cpp \
        creatures/sensors/distance.cpp \
        creatures/sensors/gyroscope.cpp \
        creatures/sensors/veloсity.cpp \
        data_processing_methods/data_processing_method.cpp \
        data_processing_methods/data_processing_method_binary.cpp \
        data_processing_methods/data_processing_method_linearly.cpp \
        data_processing_methods/data_processing_method_linearly_single.cpp \
        data_processing_methods/data_processing_method_logarithmic.cpp \
        main.cpp \
        phys_obj/cube.cpp \
        phys_obj/figure.cpp \
        phys_obj/sphere.cpp \
        scene/tripod.cpp \
        scene/world_3d.cpp \
        teachers/teacher.cpp \
        teachers/teacher_walking.cpp

HEADERS += \
    bnn/src/brain/brain.h \
    bnn/src/brain/config.h \
    bnn/src/brain_tools.h \
    bnn/src/brain/m_sequence.h \
    bnn/src/brain/neurons/binary.h \
    bnn/src/brain/neurons/motor.h \
    bnn/src/brain/neurons/neuron.h \
    bnn/src/brain/neurons/sensor.h \
    bnn/src/brain/random/config.h \
    bnn/src/brain/random/random.h \
    bnn/src/brain/simple_math.h \
    bnn/src/brain/state.h \
    bnn/src/brain/storage.h \
    bnn/src/brain/thread.h \
    conductors/conductor.h \
    conductors/conductor_circle.h \
    config.h \
    creatures/creature.h \
    creatures/joint.h \
    creatures/leg.h \
    creatures/sensors/acceleration.h \
    creatures/sensors/distance.h \
    creatures/sensors/gyroscope.h \
    creatures/sensors/veloсity.h \
    data_processing_methods/data_processing_method.h \
    data_processing_methods/data_processing_method_binary.h \
    data_processing_methods/data_processing_method_linearly.h \
    data_processing_methods/data_processing_method_linearly_single.h \
    data_processing_methods/data_processing_method_logarithmic.h \
    phys_obj/cube.h \
    phys_obj/figure.h \
    phys_obj/sphere.h \
    scene/tripod.h \
    scene/world_3d.h \
    teachers/teacher.h \
    teachers/teacher_walking.h

INCLUDEPATH += \
/usr/include/boost/system \
/usr/local/include/ode \
/usr/local/include/OGRE \
/usr/local/include/OGRE/Bites \
/usr/local/include/OGRE/RTShaderSystem

LIBS += \
-lX11 \
-lstdc++fs \
-lpthread \
-lboost_system \
-lode \
-lOgreBites \
-lOgreMain \
-lOgreRTShaderSystem
