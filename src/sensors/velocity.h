/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SENSORS_VELOCITY_H
#define BNN_DEVICE_3D_SENSORS_VELOCITY_H

#include "ode.h"
#include "Ogre.h"

#include "bnn/src/brain_tools.h"
#include "data_processing_methods/data_processing_method_linearly.h"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace bnn_device_3d::sensors
{

struct velocity
{
    velocity();

    void set_inputs(dBodyID, bnn::brain&, u_word& count, u_word length, float range_from, float range_to, std::string& str);

private:
    std::unique_ptr<dpm::data_processing_method> data_processing_method_;
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_VELOCITY_H
