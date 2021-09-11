/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef CREATURE_H
#define CREATURE_H

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

#include "bnn/src/brain_friend.h"
#include "config.h"
#include "data_processing_methods/data_processing_method_linearly.h"
#include "data_processing_methods/data_processing_method_binary.h"
#include "data_processing_methods/data_processing_method_linearly_single.h"
#include "data_processing_methods/data_processing_method_logarithmic.h"
#include "leg.h"
#include "phys_obj/cube.h"
#include "teachers/teacher_walking.h"

#define body_length (200 * device_3d_SCALE)
#define body_width (75 * device_3d_SCALE)
#define body_height (25 * device_3d_SCALE)
#define body_mass (body_length * body_width * body_height * device_3d_MASS_SCALE)

#define leg_count 4
#define leg_fl 0
#define leg_fr 1
#define leg_rl 2
#define leg_rr 3
#define joint_in_leg_count 2
#define eye_count 2
#define coordinates_count 3
#define i_sees_moveable_figures_number 2
#define force_distance_count (leg_count * 2)
#define creature_sees_world

#ifdef creature_sees_world
#define inputs_from_world_objests (i_sees_moveable_figures_number * eye_count * _word_bits)
#endif

class creature
{
    _word random_array_length_in_power_of_two = 24;

    _word quantity_of_neurons_in_power_of_two = 20;

    _word input_length =
            // I feel my legs
            leg_count * bits_in_byte * joint_in_leg_count
            // I feel my velosity [2 bytes / coordinate]
            + 2 * bits_in_byte * coordinates_count
            // I feel my orientation by the three dots on my body [2 bytes / coordinate]
            + 3 * 2 * bits_in_byte * coordinates_count
        #ifdef creature_sees_world
            // I see figures with two eyes
            + inputs_from_world_objests
        #endif
            ;

    // I control my legs
    _word output_length = leg_count * 2 * joint_in_leg_count;

    float force[force_distance_count];
    float distance[force_distance_count];
    dSpaceID space;
    std::list<dGeomID> colliding_geoms;
    std::shared_ptr<std::vector<uint32>> input_from_world;
    std::unique_ptr<data_processing_method> data_processing_method_;
    std::unique_ptr<teacher> teacher_;

public:
    std::unique_ptr<bnn::brain> brain_;
    std::unique_ptr<bnn::brain_friend> brain_friend_;
    cube body;
    leg legs[leg_count];

    creature();
    creature(Ogre::SceneManager* scnMgr, dWorldID world, std::shared_ptr<std::vector<uint32>> input_from_world);
    void set_position(dReal x, dReal y, dReal z);
    void start();
    void step();
    void stop();
};

#endif // CREATURE_H
