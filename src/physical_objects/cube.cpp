/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "cube.h"

namespace bnn_device_3d::physical_objects
{

cube::cube() : figure()
{

}

cube::cube(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal lx, dReal ly, dReal lz)
    : figure(scnMgr, world, space, mass)
{
    ent = scnMgr->createEntity(name, Ogre::SceneManager::PT_CUBE);

    // the default diameter of sphere is 100.0f
//    Ogre::Real scale = radius * 2 / 100.0f;
    node->setScale(Ogre::Vector3(lx/100.0f, ly/100.0f, lz/100.0f));

    node->attachObject(ent);

    geom = dCreateBox (space, lx, ly, lz);

    //dMassSetBox (&this->mass, 0.0001, lx, ly, lz);

    dMassSetBoxTotal (&this->mass, mass, lx, ly, lz);
    dBodySetMass(body, &this->mass);
    dGeomSetBody(geom, body);

    dBodySetPosition(body, 0, 0, 0);
    //dBodySetQuaternion (body, dQuaternion{1,0,0,0});

    //dSpaceAdd (space, geom);
}

} // namespace bnn_device_3d::physical_objects
