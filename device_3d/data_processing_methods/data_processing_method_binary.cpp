/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "data_processing_method_binary.h"

#include <string>

namespace bnn_device_3d::data_processing_methods
{

data_processing_method_binary::~data_processing_method_binary()
{

}

data_processing_method_binary::data_processing_method_binary()
{

}

u_word data_processing_method_binary::get_bools(float from, float to, float value, int levels_number)
{
    // TODO
    auto bnn_math_two_pow_x = [](u_word x) -> u_word
    {
        return u_word(1) << x;
    };

    to -= from;
    value -= from;

    value /= to;
    value *= (bnn_math_two_pow_x(levels_number) - 1);

    return value;
}

void data_processing_method_binary::set_inputs(bnn::architecture& b, u_word& count, u_word length, float value,
                                               float range_from, float range_to, std::string& s, bool verbose)
{
    u_word bools = get_bools(range_from, range_to, value, length);
    for(uint8_t j = 0; j < length; j++)
    {
        b.set_input(count++, (bools >> j) & 1);

        if(verbose)
            s += std::to_string((bools >> j) & 1);
    }
};

} // namespace bnn_device_3d::data_processing_methods
