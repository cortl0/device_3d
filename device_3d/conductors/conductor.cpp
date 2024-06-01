/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "conductor.h"

namespace bnn_device_3d::conductors
{

conductor::~conductor()
{

}

conductor::conductor()
{

}

float conductor::get_length(float x, float y, float z)
{
    return pow(x*x + y*y + z*z, 0.5);
}

void conductor::normalize(float& x, float& y, float& z)
{
    float r = pow(x*x + y*y + z*z, 0.5);

    x /= r;
    y /= r;
    z /= r;
}

void conductor::scale_by_coef(float& x, float& y, float& z, float coef)
{
    x *= coef;
    y *= coef;
    z *= coef;
}

} // namespace bnn_device_3d::conductors
