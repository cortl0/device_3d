/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SCENES_BIKE_H
#define BNN_DEVICE_3D_SCENES_BIKE_H

#include "scene.h"

namespace bnn_device_3d::scenes
{

class bike : public scene
{
public:
    ~bike();
    bike() = delete;
    bike(Ogre::RenderWindow*, Ogre::SceneManager*);

    void setup(
            std::list<dGeomID>& stationary_colliding_geoms,
            std::list<dGeomID>& movable_colliding_geoms,
            std::list<dGeomID>& creature_colliding_geoms,
            std::list<Ogre::SceneNode*>& bounding_nodes,
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
            dWorldID
            ) override;

    void step(
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
            std::list<Ogre::SceneNode*>& bounding_nodes,
            keys_states&
            ) override;

private:
    void creating_creature(
            std::list<dGeomID>& creature_colliding_geoms,
            std::list<Ogre::SceneNode*>& bounding_nodes,
            dWorldID
            );

    void creating_movable_objects(
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
            std::list<dGeomID>& movable_colliding_geoms,
            std::list<Ogre::SceneNode*>& bounding_nodes,
            dWorldID
            );

    void creating_stationary_objects(
            std::list<dGeomID>& stationary_colliding_geoms,
            std::list<Ogre::SceneNode*>& bounding_nodes,
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
            dWorldID
            );

    bool is_fail();

    void keyboard_polling(
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
            keys_states&
            );

    void ogre_setup();

    void print_distance();
};

} // namespace bnn_device_3d::scenes

#endif // BNN_DEVICE_3D_SCENES_BIKE_H
