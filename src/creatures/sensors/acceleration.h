/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef SENSORS_ACCELERATION_H
#define SENSORS_ACCELERATION_H

#include "ode.h"
#include "Ogre.h"

#include "bnn/src/brain_tools.h"
#include "data_processing_methods/data_processing_method_linearly.h"

namespace bnn_device_3d::creatures::sensors
{

struct acceleration
{
    acceleration();

    void set_inputs(dBodyID, bnn::brain&, _word& count, _word length, float range_from, float range_to, std::string& str);

private:
    std::unique_ptr<data_processing_method> data_processing_method_;
};

} // bnn_device_3d::creatures::sensors

#endif // SENSORS_ACCELERATION_H
