/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SENSORS_DISTANCE_H
#define BNN_DEVICE_3D_SENSORS_DISTANCE_H

#include <ode.h>
#include <Ogre.h>

#include "data_processing_methods/data_processing_method.h"

namespace bnn_device_3d::sensors
{

struct distance
{
    distance();

    void set_inputs(dBodyID, dGeomID, bnn::architecture&, u_word& count, u_word length, float range, std::string& str, bool verbose);

private:
    std::unique_ptr<bnn_device_3d::data_processing_methods::data_processing_method> data_processing_method_;
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_DISTANCE_H
