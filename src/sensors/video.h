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

#include "submodules/bnn/src/common/bnn_tools.h"

#include "data_processing_methods/data_processing_method_linearly.h"

namespace bnn_device_3d::sensors
{

struct video
{
    video() = delete;

    video(uint32_t width, uint32_t height, uint32_t step);

    void calculate_data(uint8_t* data, uint32_t full_width, uint32_t full_heigth);

    void set_inputs(bnn::architecture&, u_word& count, u_word length, float range_ignored, std::string& str, bool verbose);

    uint32_t width, height, step;
    std::vector<int> calc_data;
    static constexpr int length = 4;
private:
    std::unique_ptr<bnn_device_3d::data_processing_methods::data_processing_method> data_processing_method_;
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_VIDEO_H
