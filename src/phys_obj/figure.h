#ifndef FIGURE_H
#define FIGURE_H

#include "ode.h"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"

using namespace std;
using namespace Ogre;

struct figure
{
    Ogre::SceneManager* scnMgr;
    Ogre::SceneNode* node;
    Ogre::Entity* ent;

    dWorldID world;
    dSpaceID space;
    dBodyID body;
    dGeomID geom;
    dMass mass;
    dQuaternion q;

    ~figure();
    figure() {}
    figure(Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass);
    static MaterialPtr create_material(int size, int step, uint8 color1, uint8 color2, uint8 a = 255);
    void step();
};

#endif // FIGURE_H
