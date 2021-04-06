#ifndef SPHERE_H
#define SPHERE_H

#include "figure.h"

struct sphere : public figure
{
    sphere() : figure() {}
    sphere(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal radius);
};

#endif // SPHERE_H
