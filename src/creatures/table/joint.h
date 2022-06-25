/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_CREATURES_TABLE_JOINT_H
#define BNN_DEVICE_3D_CREATURES_TABLE_JOINT_H

#include "ode.h"

namespace bnn_device_3d::creatures::table
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
    dJointGroupID Joint_group_id = dJointGroupCreate(0);
    dJointID joint_id;
    double angle_start ,angle_end;
    double torque_coefficient;
};

} // namespace bnn_device_3d::creatures::table

#endif // BNN_DEVICE_3D_CREATURES_TABLE_JOINT_H
