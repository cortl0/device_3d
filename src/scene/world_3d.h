/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SCENE_WORLD_3D_H
#define BNN_DEVICE_3D_SCENE_WORLD_3D_H

#include <iostream>
#include <list>
#include <memory>
#include <thread>

#include "ode.h"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"

#include "creatures/creature.h"
#include "physical_objects/cube.h"
#include "physical_objects/sphere.h"
#include "tripod.h"
#include "conductors/conductor_circle.h"
#include "conductors/conductor_circle.h"

using namespace Ogre;

namespace cond = bnn_device_3d::conductors;

namespace pho = bnn_device_3d::physical_objects;

namespace bnn_device_3d::scene
{

class world_3d : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
    ~world_3d();
    world_3d();
    void collide_action();
    void fill_it_up();
    virtual void setup() override;
    void setup_ogre();
    void setup_ode();
    bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    bool keyReleased(const OgreBites::KeyboardEvent& evt) override;
    static void function(world_3d *me);
    void load();
    void render_go();
    void save();
    void save_random();
    void start();
    void stop();

private:
    bool shutdown = false;
    Ogre::Camera* third_person_camera;
    Ogre::Camera* creature_camera;
    Ogre::SceneNode* third_person_camera_node;
    Ogre::SceneNode* creature_camera_node;
    Ogre::Root* root;
    Ogre::SceneManager* scnMgr;

    dJointGroupID contactgroup;
    dGeomID plane;
    dSpaceID space;
    dWorldID world;

    std::list<dGeomID> stationary_colliding_geoms;
    std::list<dGeomID> movable_colliding_geoms;
    std::list<dGeomID> creature_colliding_geoms;
    std::list<pho::figure> stepping_figures;
    std::unique_ptr<cond::conductor> conductor_;
    std::unique_ptr<creatures::creature> creature_;
    std::list<Ogre::SceneNode*> bounding_nodes;
    std::unique_ptr<std::thread> thread_;
    std::unique_ptr<tripod> tripod_;
    bnn::state state_ = bnn::state::stopped;

    const float f = static_cast<float>(pow(0.5, 0.5));

    bool verbose = false;

    static void collide_action2(world_3d *me, dGeomID o1, dGeomID o2);
};

} // namespace bnn_device_3d::scene

#endif // BNN_DEVICE_3D_SCENE_WORLD_3D_H
