/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "joint.h"

joint::joint(dWorldID world, dBodyID first_body_id, dBodyID second_body_id, float dir_y, float dir_z, float anchor_x, float anchor_y, float anchor_z)
{
    joint_id = dJointCreateHinge (world, Joint_group_id);
    dJointAttach (joint_id, first_body_id, second_body_id);
    dJointSetHingeAnchor (joint_id, anchor_x, anchor_y, anchor_z);
    dJointSetHingeAxis (joint_id, 0, dir_y, dir_z);
}

void joint::set_params(float angle_start, float angle_end)
{
    dJointSetHingeParam (joint_id, dParamLoStop, angle_start);
    dJointSetHingeParam (joint_id, dParamHiStop, angle_end);
}
