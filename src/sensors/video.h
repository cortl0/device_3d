/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SENSORS_VIDEO_H
#define BNN_DEVICE_3D_SENSORS_VIDEO_H

#include <vector>

#include "bnn/src/brain_tools.h"
#include "data_processing_methods/data_processing_method_linearly.h"

namespace bnn_device_3d::sensors
{

struct video
{
    video() = delete;

    video(uint32_t w, uint32_t h, uint32_t w2, uint32_t h2, uint32_t step);

    void calculate_data();

    void set_inputs(bnn::brain&, u_word& count, u_word length, float range_ignored, std::string& str);

    uint8_t* data;
    uint32_t w, h, w2, h2, step = 32;
    std::vector<int> calc_data;
private:
    std::unique_ptr<bnn_device_3d::data_processing_methods::data_processing_method> data_processing_method_;
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_VIDEO_H
