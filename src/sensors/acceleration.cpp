/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "acceleration.h"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace bnn_device_3d::sensors
{

acceleration::acceleration()
{
    data_processing_method_.reset(new dpm::data_processing_method_linearly());
}

void acceleration::set_inputs(dBodyID body, bnn::brain& brain_, u_word& count_input, u_word length, float range_from, float range_to, std::string& debug_str)
{

}

} // namespace bnn_device_3d::sensors
