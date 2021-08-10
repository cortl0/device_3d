#
#   device_3d
#   created by Ilya Shishkin
#   cortl@8iter.ru
#   https://github.com/cortl0/device_3d
#   licensed by GPL v3.0
#

TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        bnn/src/brain/thread.cpp \
        creature.cpp \
        leg.cpp \
        main.cpp \
        phys_obj/cube.cpp \
        phys_obj/figure.cpp \
        phys_obj/sphere.cpp \
        tripod.cpp \
        world_3d.cpp

HEADERS += \
    config.h \
    creature.h \
    leg.h \
    phys_obj/cube.h \
    phys_obj/figure.h \
    phys_obj/sphere.h \
    tripod.h \
    world_3d.h

INCLUDEPATH += \
/usr/local/include/ode \
/usr/local/include/OGRE \
/usr/local/include/OGRE/Bites \
/usr/local/include/OGRE/RTShaderSystem \
/usr/include/boost/system

LIBS += -lX11 \
-lode \
-lpthread \
-lboost_system \
/usr/local/lib/libOgreBites.so \
/usr/local/lib/libOgreBites.so.1.12.11 \
/usr/local/lib/libOgreMain.so \
/usr/local/lib/libOgreMain.so.1.12.11 \
/usr/local/lib/libOgreMeshLodGenerator.so \
/usr/local/lib/libOgreMeshLodGenerator.so.1.12.11 \
/usr/local/lib/libOgreOverlay.so \
/usr/local/lib/libOgreOverlay.so.1.12.11 \
/usr/local/lib/libOgrePaging.so \
/usr/local/lib/libOgrePaging.so.1.12.11 \
/usr/local/lib/libOgreProperty.so \
/usr/local/lib/libOgreProperty.so.1.12.11 \
/usr/local/lib/libOgreRTShaderSystem.so \
/usr/local/lib/libOgreRTShaderSystem.so.1.12.11 \
/usr/local/lib/libOgreTerrain.so \
/usr/local/lib/libOgreTerrain.so.1.12.11 \
/usr/local/lib/libOgreVolume.so \
/usr/local/lib/libOgreVolume.so.1.12.11 \
/usr/local/lib/OGRE/Codec_EXR.so \
/usr/local/lib/OGRE/Codec_FreeImage.so \
/usr/local/lib/OGRE/Codec_STBI.so \
/usr/local/lib/OGRE/Plugin_BSPSceneManager.so \
/usr/local/lib/OGRE/Plugin_DotScene.so \
/usr/local/lib/OGRE/Plugin_OctreeSceneManager.so \
/usr/local/lib/OGRE/Plugin_OctreeZone.so \
/usr/local/lib/OGRE/Plugin_ParticleFX.so \
/usr/local/lib/OGRE/Plugin_PCZSceneManager.so \
/usr/local/lib/OGRE/RenderSystem_GL.so \
/usr/local/lib/OGRE/RenderSystem_GL3Plus.so

LIBS += \
    -lstdc++fs \
    -pthread

SOURCES +=  \
    ./bnn/src/brain_friend.cpp \
    ./bnn/src/brain/brain.cpp \
    ./bnn/src/brain/m_sequence.cpp \
    ./bnn/src/brain/neurons/binary.cpp \
    ./bnn/src/brain/neurons/motor.cpp \
    ./bnn/src/brain/neurons/neuron.cpp \
    ./bnn/src/brain/neurons/sensor.cpp \
    ./bnn/src/brain/random_put_get.cpp

HEADERS += \
    ./bnn/src/brain_friend.h \
    ./bnn/src/brain/brain.h \
    ./bnn/src/brain/m_sequence.h \
    ./bnn/src/brain/random_put_get.h \
    ./bnn/src/brain/simple_math.h \
    ./bnn/src/brain/config.h


