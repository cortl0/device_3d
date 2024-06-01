/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SENSORS_SENSOR_H
#define BNN_DEVICE_3D_SENSORS_SENSOR_H

#include <string>

#include "lib/bnn/src/common/architecture.h"

namespace bnn_device_3d::sensors
{

struct sensor
{
    sensor(std::string name, u_word input_offset, u_word input_length)
        : name(name), input_offset(input_offset), input_length(input_length) {}

    virtual void set_inputs(bnn::architecture&, u_word& input_offset, std::string& debug_str, bool verbose) = 0;

    std::string name;

protected:
    u_word input_offset;
    u_word input_length;
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_SENSOR_H
