/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef SENSORS_DISTANCE_H
#define SENSORS_DISTANCE_H

#include "ode.h"
#include "Ogre.h"

#include "bnn/src/brain_tools.h"
#include "data_processing_methods/data_processing_method_linearly.h"
#include "data_processing_methods/data_processing_method_binary.h"

using namespace std;
using namespace Ogre;

namespace bnn_device_3d::creatures::sensors
{

struct distance
{
    distance();

    void set_inputs(dBodyID, dGeomID, bnn::brain&, _word& count, _word length, float range, std::string& str);

private:
    std::unique_ptr<data_processing_method> data_processing_method_;
};

} // bnn_device_3d::creatures::sensors

#endif // SENSORS_DISTANCE_H
