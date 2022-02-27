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

#include "bnn/src/brain_tools.h"
#include "config.h"
#include "data_processing_methods/data_processing_method_linearly.h"
#include "data_processing_methods/data_processing_method_binary.h"
#include "data_processing_methods/data_processing_method_linearly_single.h"
#include "data_processing_methods/data_processing_method_logarithmic.h"
#include "sensors/distance.h"
#include "sensors/gyroscope.h"
#include "sensors/velocity.h"
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
#define coordinates_count 3
#define QUANTITY_OF_SHAPES_WHICH_I_SEE  2
#define creature_sees_world

#ifdef creature_sees_world
#define inputs_from_world_objests (QUANTITY_OF_SHAPES_WHICH_I_SEE * QUANTITY_OF_EYES * QUANTITY_OF_BITS_IN_WORD)
#endif

namespace dpm = bnn_device_3d::data_processing_methods;

namespace pho = bnn_device_3d::physical_objects;

namespace tch = bnn_device_3d::teachers;

namespace bnn_device_3d::creatures
{

class creature
{
public:
    pho::cube body;
    pho::cube body_sign;
    std::unique_ptr<bnn::brain_tools> brain_;
    std::vector<leg> legs;
    sensors::distance distance_;
    sensors::gyroscope gyroscope_;
    sensors::velocity speedometer_;

    ~creature();
    creature(Ogre::SceneManager* scnMgr, dWorldID world);
    std::vector<pho::figure*> get_figures();
    void set_position(dReal x, dReal y, dReal z);
    void start();
    void step(std::list<dGeomID>& distance_geoms, bool& verbose);
    void stop();

private:
    u_word random_array_length_in_power_of_two = 27;
    u_word quantity_of_neurons_in_power_of_two = 20;
    u_word threads_count_in_power_of_two = 2;
    u_word input_length;
    u_word output_length;
    std::vector<double> force;
    std::vector<double> distance;
    dSpaceID space;
    std::list<dGeomID> colliding_geoms;
    std::unique_ptr<dpm::data_processing_method> data_processing_method_;
    std::unique_ptr<tch::teacher> teacher_;
};

} // namespace bnn_device_3d::creatures

#endif // BNN_DEVICE_3D_CREATURES_CREATURE_H
