#include "sphere.h"

sphere::sphere(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass, dReal radius)
    : figure(scnMgr, world, space, mass)
{
    ent = scnMgr->createEntity(name, Ogre::SceneManager::PrefabType::PT_SPHERE);

    // the default diameter of sphere is 100.0f
    Ogre::Real scale = radius * 2 / 100.0f;
    node->setScale(Ogre::Vector3(scale, scale, scale));

    node->attachObject(ent);


    ent->setMaterial(create_material(256, 32, 191, 127));

    geom = dCreateSphere(space, radius);
    dMassSetSphereTotal(&this->mass, mass, radius);
    //dMassAdjust (&this->mass, 1);
    dBodySetMass(body, &this->mass);
    dGeomSetBody(geom, body);

    dBodySetPosition (body, 0, 0, 0);
    //dBodySetQuaternion (body, dQuaternion{1,0,0,0});

    //dSpaceAdd (space, geom);
}
