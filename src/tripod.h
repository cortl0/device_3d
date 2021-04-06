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

    const dReal coef_d = 10.0f;
    float dx;
    float dy;
    float dz;

    Ogre::SceneNode* cam_node;

    tripod(dWorldID world, Ogre::SceneNode* cam_node, dBodyID target);

    void step();
};

#endif // TRIPOD_H
