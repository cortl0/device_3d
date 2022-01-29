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

#include "config.h"
#include "creatures/creature.h"
#include "phys_obj/cube.h"
#include "phys_obj/sphere.h"
#include "tripod.h"
#include "conductors/conductor_circle.h"
#include "conductors/conductor_circle.h"

using namespace Ogre;

static dWorldID world_st;
static dJointGroupID contactgroup_st;

class world_3d : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
    ~world_3d();
    world_3d();
    void collide_action();
    void fill_it_up();
    virtual void setup(void);
    void setup_ogre();
    void setup_ode();
    bool keyPressed(const OgreBites::KeyboardEvent& evt);
    bool keyReleased(const OgreBites::KeyboardEvent& evt);
    static void function(world_3d *me);
    void load();
    void save();
    void start();
    void stop();

private:
    Ogre::Camera* cam;
    Ogre::SceneNode* camNode;
    Ogre::Root* root;
    Ogre::SceneManager* scnMgr;

    dJointGroupID contactgroup;
    dGeomID plane;
    dSpaceID space;
    dWorldID world;

    std::list<dGeomID> stationary_colliding_geoms;
    std::list<dGeomID> movable_colliding_geoms;
    std::list<dGeomID> creature_colliding_geoms;
    std::list<figure> stepping_figures;
    std::unique_ptr<conductor> conductor_;
    std::vector<_word> input_from_world;
    std::unique_ptr<creature> creature_;
    std::list<Ogre::SceneNode*> bounding_nodes;
    std::unique_ptr<std::thread> thread_;
    std::unique_ptr<tripod> tripod_;
    bnn::state state_ = bnn::state::stopped;

    const float f = static_cast<float>(pow(0.5, 0.5));

    static void collide_action2(world_3d *me, dGeomID o1, dGeomID o2);
};

#endif // WORLD_3D_H
