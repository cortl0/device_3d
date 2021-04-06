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
//#include "./bnn/src/brain/brain.h"
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

//#define fourth_x (120 * scale)
//#define fourth_y (15 * scale)
//#define fourth_z (15 * scale)
//#define fourth_mass (fourth_x * fourth_y * fourth_z * mass_scale)

//#define fifth_r 15
//#define fifth_mass (fifth_r * fifth_r * fifth_r * mass_scale)


const float f = static_cast<float>(pow(0.5, 0.5));

struct creature
{
#define leg_count 4//6

    bool cycle = true;

    std::shared_ptr<std::vector<uint32>> input_from_world;

    _word input_length = 24 * leg_count + 72 + /*input_from_world.size()*/6 * sizeof(uint32)*8;
    _word output_length = 24 * leg_count;

#define for_dis_count (leg_count * 3)

    float force[for_dis_count];
    float distance[for_dis_count];

    void* wrld;
    bool ft = true;


    volatile bool stop = false;

    //std::vector<short> distanses;

    std::unique_ptr<bnn::brain> brn;
    std::unique_ptr<bnn::brain_friend> brn_frnd;
    //    std::unique_ptr<bnn::brain_friend_dev> brn_frnd;

    static void brain_clock_cycle_handler(void* owner);
    //    void read_sensor_state();
    //    static void write_motor_state(device* d);

    //public:
    //    device();
    //    void log_cycle();
    //    void run();

    std::list<dGeomID> colliding_geoms;

    struct leg
    {
        cube first;
        cube second;
        cube third;
        //cube fourth;
        //sphere fifth;

        dJointID j_fs;
        dJointID j_st;
        //dJointID j_tf;

        leg() {}
        leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dQuaternion q, float dir_lr, float dir_fr);
        void relocate(dReal dx, dReal dy, dReal dz, dQuaternion q);

        /// fs - input torque [-1, 1], output angle between first & second [-1, 1]
        /// st - input torque [-1, 1], output angle between second & third [-1, 1]
        /// tf - input torque [-1, 1], output angle between third & fourth [-1, 1]
        void step(float& fs, float& st/*, float& tf*/);
    };

    cube body;
    //    sphere body_sph0;
    //    sphere body_sph1;
    //    sphere body_sph2;
    dSpaceID space;

    leg legs[leg_count];

#define leg_fl 0
#define leg_fr 1
    //#define leg_ml 2
    //#define leg_mr 3
#define leg_rl 2
#define leg_rr 3

    creature() {}
    creature(Ogre::SceneManager* scnMgr, dWorldID world, std::shared_ptr<std::vector<uint32>> input_from_world);
    void set_position(dReal x, dReal y, dReal z);
    void step();
};

#endif // CREATURE_H
