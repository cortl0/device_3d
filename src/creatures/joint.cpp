/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "joint.h"

#include "../bnn/src/brain/config.hpp"

namespace bnn_device_3d::creatures
{

joint::joint(dWorldID world, dBodyID first_body_id, dBodyID second_body_id,
             double dir_y, double dir_z, double anchor_x,
             double anchor_y, double anchor_z,
             double angle_start, double angle_end,
             double torque_coefficient)
    : angle_start(angle_start), angle_end(angle_end), torque_coefficient(torque_coefficient)
{
    joint_id = dJointCreateHinge (world, Joint_group_id);
    dJointAttach (joint_id, first_body_id, second_body_id);
    dJointSetHingeAnchor (joint_id, anchor_x, anchor_y, anchor_z);
    dJointSetHingeAxis (joint_id, 0, dir_y, dir_z);
    dJointSetHingeParam (joint_id, dParamLoStop, angle_start);
    dJointSetHingeParam (joint_id, dParamHiStop, angle_end);
}

double value_in_range(const double& value, const double& range_start, const double& range_end)
{
    static constexpr double default_range_start = -1;
    static constexpr double default_range_end = 1;
    static constexpr double default_range_delta = default_range_end - default_range_start;

    if(value < range_start - default_range_delta || value > range_end + default_range_delta)
        throw_error("");

    return default_range_start + ((value - range_start) / (range_end - range_start)) * default_range_delta;
}

double joint::get_angle()
{
    return value_in_range(dJointGetHingeAngle(joint_id), angle_start, angle_end);
}

void joint::set_torque(double torque)
{
    dJointAddHingeTorque(joint_id, torque * torque_coefficient / (1 + abs(dJointGetHingeAngleRate(joint_id))));
}

} // namespace bnn_device_3d::creatures
