#include "data_processing_method_linearly.h"

data_processing_method_linearly::~data_processing_method_linearly()
{

}

data_processing_method_linearly::data_processing_method_linearly()
{

}

bool data_processing_method_linearly::get_bool(float from, float to, float value, int levels_number, int level)
{
#if(0)
    if(value <= from && level == 0)
        return true;

    if(value >= to && level == levels_number - 1)
        return true;

    to -= from;
    value -= from;
    from = 0;

    float level_size = to / levels_number;

    float l_low = level_size * level;

    float l_high = l_low + level_size;

    if(value >= l_low && value <= l_high)
        return true;

    return false;
#else
    to -= from;
    value -= from;
    from = 0;

    float level_size = to / levels_number;

    float l_low = level_size * level;

    if(value >= l_low)
        return true;

    return false;
#endif
}

void data_processing_method_linearly::set_inputs(bnn::brain& brn, _word& count, float value, float range, std::string& str)
{
    for(uint8_t j = 0; j < bits_in_byte; j++)
    {
        brn.set_in(count++, get_bool(-range, range, value, bits_in_byte, j));
#ifdef show_debug_data
        str += std::to_string(get_bool(-range, range, value, bits_in_byte, j));
#endif
    }
};
