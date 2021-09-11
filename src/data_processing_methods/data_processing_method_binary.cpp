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

    value /= to;
    value *= (bnn::simple_math::two_pow_x(levels_number) - 1);

    return value;
}

void data_processing_method_binary::set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str)
{
    _word bools = get_bools(range_from, range_to, value, length);
    for(uint8_t j = 0; j < length; j++)
    {
        brn.set_in(count++, (bools >> j) & 1);
#ifdef show_debug_data
        str += std::to_string((bools >> j) & 1);
#endif
    }
};
