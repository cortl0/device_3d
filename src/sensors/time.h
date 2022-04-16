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

#include "bnn/src/brain_tools.h"

namespace bnn_device_3d::sensors
{

class time
{
public:
    time();
    static int get_data_size();
    void set_inputs(bnn::brain&, u_word& count, std::string& str, bool verbose);
};

} // namespace bnn_device_3d::sensors

#endif // BNN_DEVICE_3D_SENSORS_TIME_H
