/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "video.h"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace bnn_device_3d::sensors
{

video::video(uint32_t width, uint32_t height, uint32_t step)
    : width(width), height(height), step(step)
{
    data_processing_method_.reset(new dpm::data_processing_method_linearly());
    //data_processing_method_.reset(new data_processing_method_binary());

    size_t counter = 0;
    for(uint32_t i = 0; i < width; i += step)
        for(uint32_t j = 0; j < height; j += step)
            counter++;

    calc_data.resize(counter);
}

void video::calculate_data(uint8_t* data, uint32_t full_width, uint32_t)
{
    size_t counter = 0;
    for(uint32_t i = 0; i < height; i += step)
        for(uint32_t j = 0; j < width; j += step)
        {
            calc_data[counter] = 0;

#define QUANTITY_BYTES_PER_PIXEL 3

            for(int k = 0; k < QUANTITY_BYTES_PER_PIXEL; k++)
                calc_data[counter] += data[(i * full_width + j) * QUANTITY_BYTES_PER_PIXEL + k];

            counter++;
        }
}

void video::set_inputs(bnn::cpu& brain_, u_word& count_input, u_word length, float, std::string& debug_str, bool verbose)
{
    size_t counter = 0;

    for(uint32_t i = 0; i < height; i += step)
    {
        for(uint32_t j = 0; j < width; j += step)
        {
#define VALUE_PER_PIXEL (255 * 3)

            data_processing_method_->set_inputs(brain_, count_input, length, static_cast<float>(calc_data[counter++]), 0, VALUE_PER_PIXEL, debug_str, verbose);

            debug_str += " ";
        }

        debug_str += "\n";
    }
}

} // namespace bnn_device_3d::sensors
