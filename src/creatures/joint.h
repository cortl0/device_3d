/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef CREATURE_LEG_JOINT_H
#define CREATURE_LEG_JOINT_H

#include "ode.h"

namespace bnn_device_3d::creatures
{

class joint
{
public:
    joint(dWorldID world, dBodyID first_body_id, dBodyID second_body_id,
          double dir_y, double dir_z, double anchor_x,
          double anchor_y, double anchor_z,
          double angle_start, double angle_end,
          double torque_coefficient);
    double get_angle();
    void set_torque(double);

private:
    dJointGroupID Joint_group_id = dJointGroupCreate (0);
    dJointID joint_id;
    double angle_start ,angle_end;
    double torque_coefficient;
};

} // bnn_device_3d::creatures

#endif // CREATURE_LEG_JOINT_H
