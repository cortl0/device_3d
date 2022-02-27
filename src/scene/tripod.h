/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SCENE_TRIPOD_H
#define BNN_DEVICE_3D_SCENE_TRIPOD_H

#include "ode.h"
#include "Ogre.h"

#include "config.h"

namespace bnn_device_3d::scene
{

struct tripod
{
    dBodyID target;
    dBodyID detector;
    dBodyID lower;
    dBodyID upper;

    const dReal* target_pos;
    const dReal* detector_pos;
    const dReal* detector_vel;
    const dReal* lower_pos;
    const dReal* lower_vel;
    const dReal* upper_pos;
    const dReal* upper_vel;
    const dReal* upper_quat;

    float detector_dx;
    float detector_dy;
    float detector_dz;

    float dx;
    float dy;
    float dz;

    Ogre::SceneNode* cam_node;

    tripod(dWorldID world, Ogre::SceneNode* cam_node, dBodyID target);

    void step();
};

} // namespace bnn_device_3d::scene

#endif // BNN_DEVICE_3D_SCENE_TRIPOD_H
