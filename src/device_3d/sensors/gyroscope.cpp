/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "gyroscope.h"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace bnn_device_3d::sensors
{

gyroscope::gyroscope(dBodyID body_id, u_word input_offset, u_word input_length)
    : sensor("gyroscope", input_offset, input_length / 3), body_id(body_id)
{
    if(input_length % 3)
        throw ~0;

    data_processing_method_.reset(new dpm::data_processing_method_linearly());
}

void gyroscope::set_inputs(bnn::architecture& bnn, u_word& input_offset, std::string& debug_str, bool verbose)
{
    constexpr float range_from = -1.f;
    constexpr float range_to = 1.f;
    static const Ogre::Quaternion ort_x(0, 1, 0, 0);
    static const Ogre::Quaternion ort_y(0, 0, 1, 0);
    static const Ogre::Quaternion ort_z(0, 0, 0, 1);

    const dReal* body_q = dBodyGetQuaternion(body_id);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();

    // Relative ort vectors
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    Ogre::Quaternion ort_y_rel = body_quat * ort_y * body_quat_inv;
    Ogre::Quaternion ort_z_rel = body_quat * ort_z * body_quat_inv;

    ort_x_rel.normalise();
    ort_y_rel.normalise();
    ort_z_rel.normalise();

    auto vel = dBodyGetAngularVel(body_id);

    dReal x_scalar = ort_x_rel[1] * vel[0] + ort_x_rel[2] * vel[1] + ort_x_rel[3] * vel[2];
    dReal y_scalar = ort_y_rel[1] * vel[0] + ort_y_rel[2] * vel[1] + ort_y_rel[3] * vel[2];
    dReal z_scalar = ort_z_rel[1] * vel[0] + ort_z_rel[2] * vel[1] + ort_z_rel[3] * vel[2];

    data_processing_method_->set_inputs(bnn, input_offset, input_length, x_scalar, range_from, range_to, debug_str, verbose);
    debug_str += " ";
    data_processing_method_->set_inputs(bnn, input_offset, input_length, y_scalar, range_from, range_to, debug_str, verbose);
    debug_str += " ";
    data_processing_method_->set_inputs(bnn, input_offset, input_length, z_scalar, range_from, range_to, debug_str, verbose);
}

#if(0)
{
    // Set inputs by direction
    x_scalar = ort_x_rel[1] * 1 + ort_x_rel[2] * 0 + ort_x_rel[3] * 0;
    y_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 1 + ort_x_rel[3] * 0;
    z_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 0 + ort_x_rel[3] * 1;
    data_processing_method_->set_inputs(*brain_, count_input, length, x_scalar, -range, range, debug_str);
    debug_str += " ";
    data_processing_method_->set_inputs(*brain_, count_input, length, y_scalar, -range, range, debug_str);
    debug_str += " ";
    data_processing_method_->set_inputs(*brain_, count_input, length, z_scalar, -range, range, debug_str);
    debug_str += "  y: ";

    x_scalar = ort_y_rel[1] * 1 + ort_y_rel[2] * 0 + ort_y_rel[3] * 0;
    y_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 1 + ort_y_rel[3] * 0;
    z_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 0 + ort_y_rel[3] * 1;
    data_processing_method_->set_inputs(*brain_, count_input, length, x_scalar, -range, range, debug_str);
    debug_str += " ";
    data_processing_method_->set_inputs(*brain_, count_input, length, y_scalar, -range, range, debug_str);
    debug_str += " ";
    data_processing_method_->set_inputs(*brain_, count_input, length, z_scalar, -range, range, debug_str);
    debug_str += "  z: ";

    x_scalar = ort_z_rel[1] * 1 + ort_z_rel[2] * 0 + ort_z_rel[3] * 0;
    y_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 1 + ort_z_rel[3] * 0;
    z_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 0 + ort_z_rel[3] * 1;
    data_processing_method_->set_inputs(*brain_, count_input, length, x_scalar, -range, range, debug_str);
    debug_str += " ";
    data_processing_method_->set_inputs(*brain_, count_input, length, y_scalar, -range, range, debug_str);
    debug_str += " ";
    data_processing_method_->set_inputs(*brain_, count_input, length, z_scalar, -range, range, debug_str);
    debug_str += " ";
}
#endif

} // namespace bnn_device_3d::sensors
