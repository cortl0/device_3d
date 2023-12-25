/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_CREATURES_CREATURE_H
#define BNN_DEVICE_3D_CREATURES_CREATURE_H

#include <memory>
#include <vector>

#include "application/config.h"
#include "physical_objects/figure.h"
#include "physical_objects/cube.h"
#include "sensors/video.h"

#define coordinates_count 3

namespace bnn_device_3d::creatures
{

class creature
{
public:
    std::unique_ptr<bnn::bnn_tools> bnn_;
    std::unique_ptr<sensors::video> video_;

    virtual ~creature();
    creature();
    virtual bnn_device_3d::physical_objects::figure& get_body() = 0;
    virtual Ogre::Vector3 get_camera_place() = 0;
    virtual std::vector<bnn_device_3d::physical_objects::figure*> get_figures() = 0;
    virtual dReal get_level() = 0;
    virtual void set_position(dReal x, dReal y, dReal z) = 0;

//    std::list<dGeomID> creature_colliding_geoms;
//    std::list<Ogre::SceneNode*> bounding_nodes;

    void start();
    virtual void step(std::string& debug_str, bool& verbose) = 0;
    void stop();
};

} // namespace bnn_device_3d::creatures

#endif // BNN_DEVICE_3D_CREATURES_CREATURE_H
