/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef CUBE_H
#define CUBE_H

#include "figure.h"

struct cube : public figure
{
    cube() : figure() {}

    cube(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal lx, dReal ly, dReal lz);
};

#endif // CUBE_H

