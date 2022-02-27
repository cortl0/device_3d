/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_CONDUCTORS_CONDUCTOR_H
#define BNN_DEVICE_3D_CONDUCTORS_CONDUCTOR_H

#include <cmath>

namespace bnn_device_3d::conductors
{

class conductor
{
public:
    virtual ~conductor();
    conductor();

    /**
     * @param
     * before calling this method
     * x, y, z - coordinates of the controlling body
     * after calling this method
     * x, y, z - vector to target point
    */
    virtual void step(float& x, float& y, float& z) = 0;
};

} // namespace bnn_device_3d::conductors

#endif // BNN_DEVICE_3D_CONDUCTORS_CONDUCTOR_H
