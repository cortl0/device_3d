/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "distance.h"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace bnn_device_3d::sensors
{

distance::distance()
{
    data_processing_method_.reset(new dpm::data_processing_method_linearly());
    //data_processing_method_.reset(new data_processing_method_binary());
}

void distance::set_inputs(dBodyID body_first, dGeomID body_second, bnn::brain& brain_, u_word& count_input, u_word length, float range, std::string& debug_str)
{
    auto *pos_first = dBodyGetPosition(body_first);

    auto *pos_second = dGeomGetPosition(body_second);

    float value = static_cast<float>(pow(pow(pos_first[0] - pos_second[0], 2) + pow(pos_first[1] - pos_second[1], 2) + pow(pos_first[2] - pos_second[2], 2), 0.5));

    data_processing_method_->set_inputs(brain_, count_input, length, value, 0, range, debug_str);

    debug_str += " ";
}

} // namespace bnn_device_3d::sensors
