/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef CREATURE_LEG_JOINT_H
#define CREATURE_LEG_JOINT_H

#include "config.h"
#include "phys_obj/cube.h"
#include "phys_obj/sphere.h"

class joint
{
    dJointGroupID Joint_group_id = dJointGroupCreate (0);
public:
    dJointID joint_id;
    float angle_start ,angle_end;

    joint(dWorldID world, dBodyID first_body_id, dBodyID second_body_id, float dir_y, float dir_z, float anchor_x, float anchor_y, float anchor_z);
    void set_params(float angle_start, float angle_end);
};

#endif // CREATURE_LEG_JOINT_H
