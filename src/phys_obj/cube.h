#ifndef CUBE_H
#define CUBE_H

#include "figure.h"

struct cube : public figure
{
    cube() : figure() {}
    cube(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal lx, dReal ly, dReal lz, uint8 a = 255);
};

#endif // CUBE_H

