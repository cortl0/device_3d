/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef CREATURE_LEG_H
#define CREATURE_LEG_H

#include <iostream>

#include "config.h"
#include "phys_obj/cube.h"
#include "phys_obj/sphere.h"
#include "joint.h"

#define first_x (45 * device_3d_SCALE)
#define first_y (45 * device_3d_SCALE)
#define first_z (45 * device_3d_SCALE)
#define first_mass (first_x * first_y * first_z * device_3d_MASS_SCALE)

#define second_x (80 * device_3d_SCALE)
#define second_y (35 * device_3d_SCALE)
#define second_z (35 * device_3d_SCALE)
#define second_mass (second_x * second_y * second_z * device_3d_MASS_SCALE)

#define third_x (100 * device_3d_SCALE)
#define third_y (30 * device_3d_SCALE)
#define third_z (30 * device_3d_SCALE)
#define third_mass (third_x * third_y * third_z * device_3d_MASS_SCALE)

#define LEG_FIRST_JOINT_TORQUE_COEFFICENT 32.0f
#define LEG_SECOND_JOINT_TORQUE_COEFFICENT 8.0f

#define FIRST_JOINT 0
#define SECOND_JOINT 1

class leg
{
public:
    cube first;
    cube second;
    cube third;

    leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space,
        dReal x, dReal y, dReal z,
        dQuaternion q, float direction, uint32 color);
    void relocate(dReal dx, dReal dy, dReal dz, dQuaternion q);

    /**
     * @param
     * before calling this method
     * fs - input torque between first & second [-1, 1]
     * st - input torque between second & third [-1, 1]
     * after calling this method
     * fs - output angle between first & second [-1, 1]
     * st - output angle between second & third [-1, 1]
    */
    void step(double& fs, double& st);

private:
    dJointGroupID jg_fs = dJointGroupCreate (0);
    dJointGroupID jg_st = dJointGroupCreate (0);

    std::vector<joint> joints;
};

#endif // CREATURE_LEG_H
