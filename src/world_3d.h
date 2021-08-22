/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef WORLD_3D_H
#define WORLD_3D_H

#include <iostream>
#include <list>
#include <memory>
#include <thread>
#include <unistd.h>

#include "ode.h"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"

#include "config.h"
#include "phys_obj/cube.h"
#include "phys_obj/sphere.h"
#include "creature.h"
#include "tripod.h"

using namespace Ogre;

static dWorldID world_st;
static dJointGroupID contactgroup_st;

class world_3d : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
    Ogre::Root* root;
    Ogre::SceneManager* scnMgr;

    dWorldID world;
    dSpaceID space;

    std::list<dGeomID> stationary_colliding_geoms;
    std::list<dGeomID> movable_colliding_geoms;
    std::list<dGeomID> creature_colliding_geoms;

    std::list<std::unique_ptr<figure>> stepping_figures;

    std::shared_ptr<std::vector<uint32>> input_from_world;

    creature creature_;

    std::list<Ogre::SceneNode*> bounding_nodes;

//    std::unique_ptr<figure> ground;

    Ogre::Camera* cam;

//    Ogre::SceneNode* node_ogrehead;
//    Ogre::SceneNode* node1;

    dJointGroupID contactgroup;

    Ogre::SceneNode* camNode;
    Vector3 velocity = Vector3(0, 0, 0);

    const float f = static_cast<float>(pow(0.5, 0.5));

    std::unique_ptr<std::thread> cycle_thread;
    std::unique_ptr<tripod> tripod_;
    dGeomID plane;
    bnn::state state_ = bnn::state::stopped;

public:
    ~world_3d();
    world_3d();
    void collide_action();
    void fill_it_up();
    void setup(void);
    void setup_ogre();
    void setup_ode();
    bool keyReleased(const OgreBites::KeyboardEvent& evt);
    void cycle();
    void load();
    void save();
    void start();
    void stop();
};

#endif // WORLD_3D_H
