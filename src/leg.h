/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef LEG_H
#define LEG_H

#include "config.h"
#include "phys_obj/cube.h"
#include "phys_obj/sphere.h"

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

class leg
{
    dJointGroupID jg_fs = dJointGroupCreate (0);
    dJointGroupID jg_st = dJointGroupCreate (0);
    dJointID j_fs, j_st;
    float fs_low ,fs_hi, st_low, st_hi;
    float torque_coef = 16.0f;

public:
    cube first;
    cube second;
    cube third;

    leg() {}
    leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dQuaternion q, float dir_lr, float dir_fr, uint32 color);
    void move_forcibly(bool move);
    void relocate(dReal dx, dReal dy, dReal dz, dQuaternion q);
    void SetHingeParams(float fs_low, float fs_hi, float st_low, float st_hi);

    /// fs - input torque [-1, 1], output angle between first & second [-1, 1]
    /// st - input torque [-1, 1], output angle between second & third [-1, 1]
    void step(float& fs, float& st);
};

#endif // LEG_H
