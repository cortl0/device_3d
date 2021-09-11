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

    float level_size = to / (levels_number + 1);

    float levels_size = level_size * level;

    return value >= levels_size && value < levels_size + level_size;
}

void data_processing_method_linearly_single::set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str)
{
    for(uint8_t j = 0; j < length; j++)
    {
        brn.set_in(count++, get_bool(range_from, range_to, value, length, j));
#ifdef show_debug_data
        str += std::to_string(get_bool(range_from, range_to, value, length, j));
#endif
    }
};
