/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

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

    /**
      @param color0, color1 - BGRA
     */
    static MaterialPtr create_material_chess(int size, int step, uint32 color0, uint32 color1);

    void set_material(MaterialPtr materialPtr);
    void step();
};

#endif // FIGURE_H
