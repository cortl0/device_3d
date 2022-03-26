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

video::video(uint32_t w, uint32_t h, uint32_t w2, uint32_t h2, uint32_t step)
    : w(w), h(h), w2(w2), h2(h2), step(step)
{
    data_processing_method_.reset(new dpm::data_processing_method_linearly());
    //data_processing_method_.reset(new data_processing_method_binary());

    size_t counter = 0;
    for(int i = 0; i < h2; i += step)
        for(int j = 0; j < w2; j += step)
            counter++;

    calc_data.resize(counter);
}

void video::calculate_data()
{
    size_t counter = 0;
    for(int i = 0; i < h2; i += step)
        for(int j = 0; j < w2; j += step)
        {
            calc_data[counter] = 0;

#define QUANTITY_BYTES_PER_PIXEL 3

            for(int k = 0; k < QUANTITY_BYTES_PER_PIXEL; k++)
                calc_data[counter] += data[(i * w + j) * QUANTITY_BYTES_PER_PIXEL + k];

            counter++;
        }
}

void video::set_inputs(bnn::brain& brain_, u_word& count_input, u_word length, float range, std::string& debug_str)
{
    size_t counter = 0;
    for(int i = 0; i < h2; i += step)
    {
        for(int j = 0; j < w2; j += step)
        {
#define VALUE_PER_PIXEL (255 * 3)

            data_processing_method_->set_inputs(brain_, count_input, length, static_cast<float>(calc_data[counter++]), 0, VALUE_PER_PIXEL, debug_str);

            debug_str += " ";
        }

        debug_str += "\n";
    }
}

} // namespace bnn_device_3d::sensors
