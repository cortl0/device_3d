/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_CONDUCTORS_KICK_H
#define BNN_DEVICE_3D_CONDUCTORS_KICK_H

#include "conductor.h"

namespace bnn_device_3d::conductors
{

class kick : public conductor
{
public:
    struct data : public conductor::data
    {
        float body_x, body_y, body_z;
    };

    virtual ~kick();
    kick();
    virtual void step(float& x, float& y, float& z, conductor::data*) override;

private:
    bool trigger = false;
    float trigger_value = 10.0f;
    float body_x = 0, body_y = 0, body_z = 0;
};

} // namespace bnn_device_3d::conductors

#endif // BNN_DEVICE_3D_CONDUCTORS_KICK_H
