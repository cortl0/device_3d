/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef TRIPOD_H
#define TRIPOD_H

#include "ode.h"
#include "Ogre.h"

#include "config.h"

struct tripod
{
    dBodyID target;
    dBodyID detector;
    dBodyID upper_detector;
    dBodyID cam_base;

    float cam_dz;

    const dReal* tar ;
    const dReal* look;
    const dReal* look_vel;
    const dReal* look_up;
    const dReal* look_up_vel;
    const dReal* bas;
    const dReal* bas_vel;

    const dReal velosity_ignore_coef = 1.0f;
    const dReal force_coef = 5.0f;
    float dx;
    float dy;
    float dz;

    Ogre::SceneNode* cam_node;

    tripod(dWorldID world, Ogre::SceneNode* cam_node, dBodyID target);

    void step();
};

#endif // TRIPOD_H