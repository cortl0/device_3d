#include "data_processing_method_linearly_single.h"

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

    float level_size = to / levels_number;

    float l_low = level_size * level;

    if(value >= l_low && value < l_low + level_size)
        return true;

    return false;
}

void data_processing_method_linearly_single::set_inputs(bnn::brain& brn, _word& count, float value, float range, std::string& str)
{
    for(uint8_t j = 0; j < bits_in_byte; j++)
    {
        brn.set_in(count++, get_bool(-range, range, value, bits_in_byte, j));
#ifdef show_debug_data
        str += std::to_string(get_bool1(-range, range, value, bits_in_byte, j));
#endif
    }
};
