/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef CREATURE_H
#define CREATURE_H

#include <iostream>

#include <vector>
#include <stdexcept>
#include <stdlib.h>
#include <algorithm>
#include <experimental/filesystem>
#include <vector>

#include "ode.h"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"

#include "config.h"
#include "phys_obj/cube.h"
#include "bnn/src/brain_friend.h"

#include "leg.h"

#define body_length (200 * device_3d_SCALE)
#define body_width (75 * device_3d_SCALE)
#define body_height (25 * device_3d_SCALE)
#define body_mass (body_length * body_width * body_height * device_3d_MASS_SCALE)

//#define body_sph_r 35
//#define body_sph_mass (body_sph_r * body_sph_r * body_sph_r * mass_scale)

const float f = static_cast<float>(pow(0.5, 0.5));

#define leg_count 4
#define bits_in_byte 8
#define joint_in_leg_count 2
#define eye_count 2
#define coordinates_count 3

#define creature_sees_world

struct creature
{
    std::shared_ptr<std::vector<uint32>> input_from_world;

    _word random_array_length_in_power_of_two = 24;

    _word quantity_of_neurons_in_power_of_two = 20;

    _word input_length =
            // I feel my legs
            leg_count * bits_in_byte * joint_in_leg_count
            // I feel my velosity
            + bits_in_byte * coordinates_count
            // I feel my orientation by the three dots on my body
            + 3 * bits_in_byte * coordinates_count
        #ifdef creature_sees_world
        #define inputs_from_world_objests (3 * eye_count * sizeof(uint32) * bits_in_byte)
            // I see three shapes at two eyes
            + inputs_from_world_objests
        #endif
            ;

    _word output_length =
            // I control my legs
            leg_count * 2 * joint_in_leg_count;

#define force_distance_count (leg_count * 2)

    float force[force_distance_count];
    float distance[force_distance_count];

    void* wrld;

    std::unique_ptr<bnn::brain> brn;
    std::unique_ptr<bnn::brain_friend> brn_frnd;

    std::list<dGeomID> colliding_geoms;

    cube body;
    dSpaceID space;

    leg legs[leg_count];

#define leg_fl 0
#define leg_fr 1
#define leg_rl 2
#define leg_rr 3

    creature() {}
    creature(Ogre::SceneManager* scnMgr, dWorldID world, std::shared_ptr<std::vector<uint32>> input_from_world);
    void set_position(dReal x, dReal y, dReal z);
    void start();
    void step();
    void stop();
};

#endif // CREATURE_H
