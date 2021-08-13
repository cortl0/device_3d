/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "cube.h"

cube::cube(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal lx, dReal ly, dReal lz, uint8 a)
    : figure(scnMgr, world, space, mass)
{
    ent = scnMgr->createEntity(name, Ogre::SceneManager::PrefabType::PT_CUBE);

    // the default diameter of sphere is 100.0f
//    Ogre::Real scale = radius * 2 / 100.0f;
    node->setScale(Ogre::Vector3(lx/100.0f, ly/100.0f, lz/100.0f));

    node->attachObject(ent);


    ent->setMaterial(create_material(64, 16, 191, 127, a));

    geom = dCreateBox (space, lx, ly, lz);

    //dMassSetBox (&this->mass, 0.0001, lx, ly, lz);

    dMassSetBoxTotal (&this->mass, mass, lx, ly, lz);
    dBodySetMass(body, &this->mass);
    dGeomSetBody(geom, body);

    dBodySetPosition (body, 0, 0, 0);
    //dBodySetQuaternion (body, dQuaternion{1,0,0,0});

    //dSpaceAdd (space, geom);
}