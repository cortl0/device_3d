#include "data_processing_method_logarithmic.h"

data_processing_method_logarithmic::~data_processing_method_logarithmic()
{

}

data_processing_method_logarithmic::data_processing_method_logarithmic()
{

}

_word data_processing_method_logarithmic::get_bools(float from, float to, float value, int levels_number)
{
    to -= from;
    value -= from;

    value /= to;
    value *= (bnn::simple_math::two_pow_x(levels_number) - 1);

    if(value < 1.0f)
        return 0;

    _word value_log = bnn::simple_math::log2_1(static_cast<_word>(value));

    return (static_cast<_word>(~0) >> (_word_bits - levels_number)) >> (levels_number - value_log - 1);
}

void data_processing_method_logarithmic::set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str)
{
    _word bools = get_bools(range_from, range_to, value, length);
    for(uint8_t j = 0; j < _word_bits; j++)
    {
        brn.set_in(count++, (bools >> j) & 1);
#ifdef show_debug_data
        str += std::to_string((bools >> j) & 1);
#endif
    }
};
