/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "conductor_circle.h"

namespace bnn_device_3d::conductors
{

conductor_circle::~conductor_circle()
{

}

conductor_circle::conductor_circle(float r)
    : r(r)
{

}

void turn(float& x0, float& y0, float& z0, float x1, float y1, float z1, float& x2, float& y2, float& z2)
{
    x2 = y0 * z1 - z0 * y1;
    y2 = z0 * x1 - x0 * z1;
    z2 = x0 * y1 - y0 * x1;
}

void normalize(float& x, float& y, float& z)
{
    float r = pow(x*x + y*y + z*z, 0.5);

    x /= r;
    y /= r;
    z /= r;
}

void scale_by_coef(float& x, float& y, float& z, float coef)
{
    x *= coef;
    y *= coef;
    z *= coef;
}

void conductor_circle::step(float& x, float& y, float& z)
{
    x = c_x - x;
    y = 0;
    z = c_z - z;

    float x2, y2, z2;
    turn(x, y, z, 0, 1.0f, 0, x2, y2, z2);
    normalize(x2, y2, z2);

    float x3, y3, z3;

    x3 = x;
    y3 = y;
    z3 = z;

    normalize(x3, y3, z3);

    x -= x3 * r;
    y -= y3 * r;
    z -= z3 * r;

    float coef = 0.25f;

    scale_by_coef(x2, y2, z2, coef);

    x += x2;
    y += y2;
    z += z2;
}

} // namespace bnn_device_3d::conductors
