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
        bnn/src/brain/brain.cpp \
        bnn/src/brain_tools.cpp \
        bnn/src/brain/m_sequence.cpp \
        bnn/src/brain/neurons/binary.cpp \
        bnn/src/brain/neurons/motor.cpp \
        bnn/src/brain/neurons/neuron.cpp \
        bnn/src/brain/neurons/sensor.cpp \
        bnn/src/brain/random/random.cpp \
        bnn/src/brain/thread.cpp \
        conductors/conductor.cpp \
        conductors/conductor_circle.cpp \
        creatures/creature.cpp \
        creatures/joint.cpp \
        creatures/leg.cpp \
        data_processing_methods/data_processing_method.cpp \
        data_processing_methods/data_processing_method_binary.cpp \
        data_processing_methods/data_processing_method_linearly.cpp \
        data_processing_methods/data_processing_method_linearly_single.cpp \
        data_processing_methods/data_processing_method_logarithmic.cpp \
        main.cpp \
        physical_objects/cube.cpp \
        physical_objects/figure.cpp \
        physical_objects/sphere.cpp \
        scene/tripod.cpp \
        scene/world_3d.cpp \
        sensors/acceleration.cpp \
        sensors/distance.cpp \
        sensors/gyroscope.cpp \
        sensors/velocity.cpp \
        teachers/teacher.cpp \
        teachers/teacher_walking.cpp

HEADERS += \
    bnn/src/brain/brain.h \
    bnn/src/brain/config.hpp \
    bnn/src/brain/random/config.hpp \
    bnn/src/brain/simple_math.hpp \
    bnn/src/brain/state.hpp \
    bnn/src/brain/storage.hpp \
    bnn/src/brain_tools.h \
    bnn/src/brain/m_sequence.h \
    bnn/src/brain/neurons/binary.h \
    bnn/src/brain/neurons/motor.h \
    bnn/src/brain/neurons/neuron.h \
    bnn/src/brain/neurons/sensor.h \
    bnn/src/brain/random/random.h \
    bnn/src/brain/thread.h \
    conductors/conductor.h \
    conductors/conductor_circle.h \
    config.h \
    creatures/creature.h \
    creatures/joint.h \
    creatures/leg.h \
    data_processing_methods/data_processing_method.h \
    data_processing_methods/data_processing_method_binary.h \
    data_processing_methods/data_processing_method_linearly.h \
    data_processing_methods/data_processing_method_linearly_single.h \
    data_processing_methods/data_processing_method_logarithmic.h \
    physical_objects/cube.h \
    physical_objects/figure.h \
    physical_objects/sphere.h \
    scene/tripod.h \
    scene/world_3d.h \
    sensors/acceleration.h \
    sensors/distance.h \
    sensors/gyroscope.h \
    sensors/velocity.h \
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
