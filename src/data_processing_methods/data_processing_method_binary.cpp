#include "data_processing_method_binary.h"

data_processing_method_binary::~data_processing_method_binary()
{

}

data_processing_method_binary::data_processing_method_binary()
{

}

_word data_processing_method_binary::get_bools(float from, float to, float value, int levels_number)
{
    to -= from;
    value -= from;

    value *= bnn::simple_math::two_pow_x(levels_number);
    value /= to;

    return value;
}

void data_processing_method_binary::set_inputs(bnn::brain& brn, _word& count, float value, float range, std::string& str)
{
    _word bools = get_bools(-range, range, value, bits_in_byte);
    for(uint8_t j = 0; j < bits_in_byte; j++)
    {
        brn.set_in(count++, (bools >> j) & 1);
#ifdef show_debug_data
        str += std::to_string((bools >> j) & 1);
#endif
    }
};
