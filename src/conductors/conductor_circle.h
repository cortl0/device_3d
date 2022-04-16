/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_CONDUCTORS_CONDUCTOR_CIRCLE_H
#define BNN_DEVICE_3D_CONDUCTORS_CONDUCTOR_CIRCLE_H

#include "conductor.h"

namespace bnn_device_3d::conductors
{

class conductor_circle : public conductor
{
    const float c_x = 0.f;
    const float c_y = 0.f;
    const float c_z = 0.f;
    const float r = 10.f;
public:
    virtual ~conductor_circle();
    conductor_circle(float r);
    virtual void step(float& x, float& y, float& z) override;
};

} // namespace bnn_device_3d::conductors

#endif // BNN_DEVICE_3D_CONDUCTORS_CONDUCTOR_CIRCLE_H
