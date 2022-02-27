/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_PHYSICAL_OBJECTS_FIGURE_H
#define BNN_DEVICE_3D_PHYSICAL_OBJECTS_FIGURE_H

#include "ode.h"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"

namespace bnn_device_3d::physical_objects
{

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
    figure();
    figure(Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass);

    /**
     * @param color0, color1 - BGRA
     */
    static Ogre::MaterialPtr create_material_chess(int size, int step, Ogre::uint32 color0, Ogre::uint32 color1);

    static Ogre::MaterialPtr create_material_body_sign(size_t size);

    void set_material(Ogre::MaterialPtr materialPtr);
    void step();
};

} // namespace bnn_device_3d::physical_objects

#endif // BNN_DEVICE_3D_PHYSICAL_OBJECTS_FIGURE_H
