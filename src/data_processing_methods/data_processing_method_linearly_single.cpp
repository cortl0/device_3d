/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "data_processing_method_linearly_single.h"

#include "config.hpp"

namespace bnn_device_3d::data_processing_methods
{

data_processing_method_linearly_single::~data_processing_method_linearly_single()
{

}

data_processing_method_linearly_single::data_processing_method_linearly_single()
{

}

bool data_processing_method_linearly_single::get_bool(float from, float to, float value, int levels_number, int level)
{
    to -= from;
    value -= from;

    float level_size = to / (levels_number + 1);

    float levels_size = level_size * level;

    return value >= levels_size && value < levels_size + level_size;
}

void data_processing_method_linearly_single::set_inputs(bnn::brain& b, u_word& count, u_word length, float value, float range_from, float range_to, std::string& s)
{
    for(uint8_t j = 0; j < length; j++)
    {
        b.set_input(count++, get_bool(range_from, range_to, value, length, j));
#ifdef show_debug_data
        s += std::to_string(get_bool(range_from, range_to, value, length, j));
#endif
    }
};

} // namespace bnn_device_3d::data_processing_methods
