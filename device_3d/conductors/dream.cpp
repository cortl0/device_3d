/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "dream.h"

namespace bnn_device_3d::conductors
{

dream::~dream()
{

}

dream::dream()
{

}

void dream::step(float &x, float &y, float &z, conductor::data* conductor_data)
{
    auto d = static_cast<dream::data*>(conductor_data);

    x = x - d->body_x;
    y = y - d->body_y;
    z = z - d->body_z;

    if(float length = get_length(x, y, z); length < 5.0f)
        d->conduct = true;
}

} // namespace bnn_device_3d::conductors
