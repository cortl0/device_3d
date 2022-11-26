/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_CREATURES_TABLE_TABLE_H
#define BNN_DEVICE_3D_CREATURES_TABLE_TABLE_H

#include <algorithm>
#include <experimental/filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stdlib.h>
#include <vector>

#include "ode.h"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"

#include "config.hpp"
#include "../creature.h"
#include "data_processing_methods/data_processing_method_linearly.h"
#include "data_processing_methods/data_processing_method_binary.h"
#include "data_processing_methods/data_processing_method_linearly_single.h"
#include "data_processing_methods/data_processing_method_logarithmic.h"
#include "sensors/distance.h"
#include "sensors/gyroscope.h"
#include "sensors/time.h"
#include "sensors/velocity.h"
#include "sensors/video.h"
#include "leg.h"
#include "physical_objects/cube.h"
#include "teachers/teacher_walking.h"

//#define body_length (200 * device_3d_SCALE)
//#define body_width (75 * device_3d_SCALE)
//#define body_height (25 * device_3d_SCALE)
#define body_length (200 * device_3d_SCALE)
#define body_width (100 * device_3d_SCALE)
#define body_height (50 * device_3d_SCALE)
#define body_mass (body_length * body_width * body_height * device_3d_MASS_SCALE)

#define NUMBER_OF_FRONT_LEFT_LEG 0
#define NUMBER_OF_FRONT_RIGHT_LEG 1
#define NUMBER_OF_REAR_LEFT_LEG 2
#define NUMBER_OF_REAR_RIGHT_LEG 3
#define QUANTITY_BITS_PER_JOINT 8
#define QUANTITY_OF_EYES 2
#define QUANTITY_OF_SHAPES_WHICH_I_SEE  2

namespace bnn_device_3d::creatures::table
{

class table : public creature
{
public:
    bnn_device_3d::physical_objects::cube body;
    bnn_device_3d::physical_objects::cube body_sign;
    std::vector<leg> legs;
    sensors::gyroscope gyroscope_;
    sensors::time time_;
    sensors::velocity speedometer_;

    virtual ~table();
    explicit table(Ogre::RenderWindow*, Ogre::SceneManager*, dWorldID);
    bnn_device_3d::physical_objects::figure& get_body() override;
    std::vector<bnn_device_3d::physical_objects::figure*> get_figures() override;
    Ogre::Vector3 get_camera_place() override;
    dReal get_level() override;

    void set_position(dReal x, dReal y, dReal z) override;
    void step(std::string& debug_str, bool& verbose) override;

private:
    u_word quantity_of_neurons_in_power_of_two = 16;
    u_word threads_count_in_power_of_two = 2;
    u_word input_length;
    u_word output_length;
    std::vector<double> force;
    std::vector<double> distance;
    dSpaceID space;
    std::list<dGeomID> colliding_geoms;
    std::unique_ptr<bnn_device_3d::data_processing_methods::data_processing_method> data_processing_method_;
    std::unique_ptr<bnn_device_3d::teachers::teacher> teacher_;

    static constexpr u_word i_feel_my_velosity_quantity_bits = 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;
    static constexpr u_word i_feel_my_orientation_quantity_bits = 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;
};

} // namespace bnn_device_3d::creatures::table

#endif // BNN_DEVICE_3D_CREATURES_TABLE_TABLE_H
