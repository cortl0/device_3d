/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SCENES_SCENE_H
#define BNN_DEVICE_3D_SCENES_SCENE_H

#include <Ogre.h>
#include <ode.h>

#include "config.hpp"
#include "conductors/conductor.h"
#include "physical_objects/figure.h"
#include "creatures/creature.h"
#include "application/tripod.h"

namespace bnn_device_3d::scenes
{

class scene
{
public:
    virtual ~scene();
    scene() = delete;
    scene(Ogre::RenderWindow*, Ogre::SceneManager*);
    virtual void load(std::ifstream&);
    virtual void save(std::ofstream&) const;
    virtual void setup(const bnn_device_3d::application::config::device_3d::bnn&) = 0;
    virtual void start();
    virtual void stop();
    virtual void step(keys_states&, dReal frame_length_dReal) = 0;

    std::unique_ptr<creatures::creature> creature_;

protected:
    void collide_action();
    void create_light(Ogre::Real x, Ogre::Real y, Ogre::Real z, const std::string name);

    int width{0};
    int height{0};
    Ogre::RenderWindow* render_window;
    Ogre::SceneManager* scene_manager;
    Ogre::SceneNode* third_person_camera_node;
    Ogre::SceneNode* creature_camera_node;
    std::list<Ogre::SceneNode*> bounding_nodes;
    std::list<bnn_device_3d::physical_objects::figure> stepping_figures;
    std::list<dGeomID> stationary_colliding_geoms;
    std::list<dGeomID> movable_colliding_geoms;
    std::list<dGeomID> creature_colliding_geoms;
    dJointGroupID contact_group;
    dWorldID world;
    Ogre::Camera* third_person_camera;
    Ogre::Camera* creature_camera;
    dSpaceID space;
    std::vector<std::unique_ptr<bnn_device_3d::conductors::conductor>> conductors;
    std::unique_ptr<application::tripod> tripod_;
    const float f = static_cast<float>(pow(0.5, 0.5));

private:
    void collide_action2(dGeomID o1, dGeomID o2);
};

} // namespace bnn_device_3d::scenes

#endif // BNN_DEVICE_3D_SCENES_SCENE_H
