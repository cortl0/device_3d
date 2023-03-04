/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_APPLICATION_APPLICATION_H
#define BNN_DEVICE_3D_APPLICATION_APPLICATION_H

#include <iostream>
#include <list>
#include <memory>

#include "ode.h"
#include "Ogre.h"
#include "OgreApplicationContext.h"

#include "config.h"
#include "common/state.h"
#include "scenes/scene.h"

using namespace Ogre;

namespace bnn_device_3d::application
{

class application : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
    virtual ~application();
    application();
    dWorldID get_world() const;
    dJointGroupID get_gontact_group() const;
    void run();

private:
    void collide_action();
    void collide_action2(dGeomID o1, dGeomID o2);
    virtual void setup() override;
    bool keyPressed(const OgreBites::KeyboardEvent&) override;
    bool keyReleased(const OgreBites::KeyboardEvent&) override;
    void load();
    void save();
    void save_random();
    void start();
    void stop();

    dJointGroupID contact_group;
    dWorldID world;
    std::unique_ptr<bnn_device_3d::scenes::scene> scene;
    std::list<dGeomID> stationary_colliding_geoms;
    std::list<dGeomID> movable_colliding_geoms;
    std::list<dGeomID> creature_colliding_geoms;
    std::list<Ogre::SceneNode*> bounding_nodes;
    std::list<bnn_device_3d::physical_objects::figure> stepping_figures;
    bnn::state state_ = bnn::state::stopped;
    bool verbose = true;
    config config_;

};

} // namespace bnn_device_3d::application

#endif // BNN_DEVICE_3D_APPLICATION_APPLICATION_H
