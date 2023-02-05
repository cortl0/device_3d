/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "data_processing_method_linearly.h"

#include <string>

namespace bnn_device_3d::data_processing_methods
{

data_processing_method_linearly::~data_processing_method_linearly()
{

}

data_processing_method_linearly::data_processing_method_linearly()
{

}

bool data_processing_method_linearly::get_bool(float from, float to, float value, int levels_number, int level)
{
    to -= from;
    value -= from;

    float level_size = to / (levels_number + 1);

    float levels_size = level_size * (level + 1);

    return value >= levels_size;
}

void data_processing_method_linearly::set_inputs(bnn::architecture& b, u_word& count, u_word length, float value,
                                                 float range_from, float range_to, std::string& s, bool verbose)
{
    for(uint8_t j = 0; j < length; j++)
    {
        b.set_input(count++, get_bool(range_from, range_to, value, length, j));

        if(verbose)
            s += std::to_string(get_bool(range_from, range_to, value, length, j));
    }
};

} // namespace bnn_device_3d::data_processing_methods
