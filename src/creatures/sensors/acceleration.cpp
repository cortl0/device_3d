/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "acceleration.h"

namespace bnn_device_3d::creatures::sensors
{

acceleration::acceleration()
{
    data_processing_method_.reset(new data_processing_method_linearly());
}

void acceleration::set_inputs(dBodyID body, bnn::brain& brain_, _word& count_input, _word length, float range_from, float range_to, std::string& debug_str)
{

}

} // bnn_device_3d::creatures::sensors
