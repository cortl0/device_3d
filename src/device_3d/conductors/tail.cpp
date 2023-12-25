/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "tail.h"

namespace bnn_device_3d::conductors
{

tail::~tail()
{

}

tail::tail()
{

}

void tail::step(float &x, float &y, float &z, conductor::data* conductor_data)
{
    auto d = static_cast<tail::data*>(conductor_data);

    x = d->body_x - x;
    y = d->body_y - y;
    z = d->body_z - z;

    if(float length = get_length(x, y, z); length > 5.0f)
        d->conduct = true;
}

} // namespace bnn_device_3d::conductors
