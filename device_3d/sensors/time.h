/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SENSORS_TIME_H
#define BNN_DEVICE_3D_SENSORS_TIME_H

#include "lib/bnn/src/common/architecture.h"

#include "sensor.h"

namespace bnn_device_3d::sensors
{

struct time final : sensor
{
public:
    time(u_word input_offset, u_word input_length);
    virtual void set_inputs(bnn::architecture&, u_word& input_offset, std::string& str, bool verbose) override;
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_TIME_H
