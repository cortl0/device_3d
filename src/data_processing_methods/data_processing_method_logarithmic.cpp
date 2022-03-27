/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "data_processing_method_logarithmic.h"

#include "config.hpp"

namespace bnn_device_3d::data_processing_methods
{

data_processing_method_logarithmic::~data_processing_method_logarithmic()
{

}

data_processing_method_logarithmic::data_processing_method_logarithmic()
{

}

u_word data_processing_method_logarithmic::get_bools(float from, float to, float value, int levels_number)
{
    to -= from;
    value -= from;

    value /= to;
    value *= (bnn::simple_math::two_pow_x(levels_number) - 1);

    if(value < 1.0f)
        return 0;

    u_word value_log = bnn::simple_math::log2_1(static_cast<u_word>(value));

    return (static_cast<u_word>(~0) >> (QUANTITY_OF_BITS_IN_WORD - levels_number)) >> (levels_number - value_log - 1);
}

void data_processing_method_logarithmic::set_inputs(bnn::brain& b, u_word& count, u_word length, float value, float range_from, float range_to, std::string& s)
{
    u_word bools = get_bools(range_from, range_to, value, length);
    for(uint8_t j = 0; j < QUANTITY_OF_BITS_IN_WORD; j++)
    {
        b.set_input(count++, (bools >> j) & 1);
#ifdef show_debug_data
        s += std::to_string((bools >> j) & 1);
#endif
    }
};

} // namespace bnn_device_3d::data_processing_methods
