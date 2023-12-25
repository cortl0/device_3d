/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_PHYSICAL_OBJECTS_SPHERE_H
#define BNN_DEVICE_3D_PHYSICAL_OBJECTS_SPHERE_H

#include "figure.h"

namespace bnn_device_3d::physical_objects
{

struct sphere : public figure
{
    sphere();

    sphere(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal radius);
};

} // namespace bnn_device_3d::physical_objects

#endif // BNN_DEVICE_3D_PHYSICAL_OBJECTS_SPHERE_H
