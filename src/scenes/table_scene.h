/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SCENES_TABLE_H
#define BNN_DEVICE_3D_SCENES_TABLE_H

#include "scene.h"

namespace bnn_device_3d::scenes
{

class table : public scene
{
public:
    table() = delete;
    table(Ogre::RenderWindow*, Ogre::SceneManager* scene_manager);
    void setup(
            std::list<dGeomID>& stationary_colliding_geoms,
            std::list<dGeomID>& movable_colliding_geoms,
            std::list<dGeomID>& creature_colliding_geoms,
            std::list<Ogre::SceneNode*>& bounding_nodes,
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
            dWorldID world) override;

    void step(
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
            std::list<Ogre::SceneNode*>& bounding_nodes,
            keys_states& keys_states_) override;
};

} // namespace bnn_device_3d::scenes

#endif // BNN_DEVICE_3D_SCENES_TABLE_H
