/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SENSORS_GYROSCOPE_H
#define BNN_DEVICE_3D_SENSORS_GYROSCOPE_H

#include "ode.h"
#include "Ogre.h"

#include "common/bnn_tools.h"
#include "data_processing_methods/data_processing_method_linearly.h"
#include "sensor.h"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace bnn_device_3d::sensors
{

struct gyroscope final : sensor
{
    gyroscope(dBodyID body_id, u_word input_offset, u_word input_length);

    virtual void set_inputs(bnn::architecture&, u_word& input_offset, std::string& debug_str, bool verbose) override;

private:
    dBodyID body_id;
    std::unique_ptr<bnn_device_3d::data_processing_methods::data_processing_method> data_processing_method_;
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_GYROSCOPE_H
