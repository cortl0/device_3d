/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "velocity.h"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace bnn_device_3d::sensors
{

velocity::velocity()
{
    data_processing_method_.reset(new dpm::data_processing_method_linearly());
}

void velocity::set_inputs(dBodyID body, bnn::brain& brain_, u_word& count_input, u_word length, float range_from, float range_to, std::string& debug_str, bool verbose)
{
    static const Ogre::Quaternion ort_x(0, 1, 0, 0);
    static const Ogre::Quaternion ort_y(0, 0, 1, 0);
    static const Ogre::Quaternion ort_z(0, 0, 0, 1);

    dReal x_scalar;
    dReal y_scalar;
    dReal z_scalar;

    const dReal* body_q = dBodyGetQuaternion(body);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();

    // Relative ort vectors
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    Ogre::Quaternion ort_y_rel = body_quat * ort_y * body_quat_inv;
    Ogre::Quaternion ort_z_rel = body_quat * ort_z * body_quat_inv;

    ort_x_rel.normalise();
    ort_y_rel.normalise();
    ort_z_rel.normalise();

    auto *vel = dBodyGetLinearVel(body);

    x_scalar = ort_x_rel[1] * vel[0] + ort_x_rel[2] * vel[1] + ort_x_rel[3] * vel[2];
    y_scalar = ort_y_rel[1] * vel[0] + ort_y_rel[2] * vel[1] + ort_y_rel[3] * vel[2];
    z_scalar = ort_z_rel[1] * vel[0] + ort_z_rel[2] * vel[1] + ort_z_rel[3] * vel[2];
    data_processing_method_->set_inputs(brain_, count_input, length, x_scalar, range_from, range_to, debug_str, verbose);
    debug_str += " ";
    data_processing_method_->set_inputs(brain_, count_input, length, y_scalar, range_from, range_to, debug_str, verbose);
    debug_str += " ";
    data_processing_method_->set_inputs(brain_, count_input, length, z_scalar, range_from, range_to, debug_str, verbose);
}

} // namespace bnn_device_3d::sensors
