/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef SPHERE_H
#define SPHERE_H

#include "figure.h"

struct sphere : public figure
{
    sphere() : figure() {}

    sphere(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal radius);
};

#endif // SPHERE_H
