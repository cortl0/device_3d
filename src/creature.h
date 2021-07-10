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
#include "./phys_obj/cube.h"
#include "./phys_obj/sphere.h"
#include "./bnn/src/brain_friend.h"

//#include "free_fly.h"

//#define scale 1.0f
//#define mass_scale 0.0001f
#define scale 1.0f
#define mass_scale 0.0001f


#define body_length (200 * scale)
#define body_width (50 * scale)
#define body_height (25 * scale)
#define body_mass (body_length * body_width * body_height * mass_scale)

//#define body_sph_r 35
//#define body_sph_mass (body_sph_r * body_sph_r * body_sph_r * mass_scale)

#define first_x (25 * scale)
#define first_y (25 * scale)
#define first_z (25 * scale)
#define first_mass (first_x * first_y * first_z * mass_scale)

#define second_x (70 * scale)
#define second_y (25 * scale)
#define second_z (25 * scale)
#define second_mass (second_x * second_y * second_z * mass_scale)

#define third_x (100 * scale)
#define third_y (20 * scale)
#define third_z (20 * scale)
#define third_mass (third_x * third_y * third_z * mass_scale)

const float f = static_cast<float>(pow(0.5, 0.5));

#define leg_count 4
#define bits_in_byte 8
#define joint_in_leg_count 2
#define coordinates_count 3
#define eye_count 2

struct creature
{
    void* owner;
    void (*owner_clock_cycle_handler)(void* owner);

    std::shared_ptr<std::vector<uint32>> input_from_world;

    _word input_length =
            88;

//            // I feel my legs
//            leg_count * bits_in_byte * joint_in_leg_count
//            // I feel my plase & orientation by the dots on my body
//            + 3 * bits_in_byte * coordinates_count
//            // I see three shapes at two coordinates
//            + 3 * 2 * sizeof(uint32) * bits_in_byte;

    _word output_length =
            // I control my legs
            leg_count * 2 * joint_in_leg_count;

#define force_distance_count (leg_count * 2)

    float force[force_distance_count];
    float distance[force_distance_count];

    void* wrld;

    std::unique_ptr<bnn::brain> brn;
    std::unique_ptr<bnn::brain_friend> brn_frnd;

    static void brain_clock_cycle_handler(void* me);

    std::list<dGeomID> colliding_geoms;

    struct leg
    {
        cube first;
        cube second;
        cube third;

        dJointID j_fs;
        dJointID j_st;

        leg() {}
        leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dQuaternion q, float dir_lr, float dir_fr);
        void relocate(dReal dx, dReal dy, dReal dz, dQuaternion q);

        /// fs - input torque [-1, 1], output angle between first & second [-1, 1]
        /// st - input torque [-1, 1], output angle between second & third [-1, 1]
        void step(float& fs, float& st);
    };

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
    void start(void* owner, void (*owner_clock_cycle_handler)(void* owner));
    void step();
    void stop();
};

#endif // CREATURE_H
