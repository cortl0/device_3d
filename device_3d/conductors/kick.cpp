/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "kick.h"

#include <cmath>

namespace bnn_device_3d::conductors
{

kick::~kick()
{

}

kick::kick()
{

}

void kick::step(float &x, float &y, float &z, conductor::data* conductor_data)
{
    auto d = static_cast<kick::data*>(conductor_data);

    if(trigger)
    {
        x = d->body_x - x;
        y = d->body_y - y;
        z = d->body_z - z;

        trigger_value = std::abs(x) + std::abs(y) + std::abs(z);

        normalize(x, y, z);

        scale_by_coef(x, y, z, 10.0f);

        if(trigger_value < 3.0f)
        {
            trigger_value = 10.0f;
            trigger = false;
        }
        else
            d->conduct = true;
    }
    else
    {
        trigger_value += std::abs(body_x - d->body_x);
        trigger_value += std::abs(body_y - d->body_y);
        trigger_value += std::abs(body_z - d->body_z);
        trigger_value -= 0.002f;

        if(trigger_value > 10.0)
            trigger_value = 10.0;

        if(trigger_value < 0.0f)
        {
            x = d->body_x - x;
            y = d->body_y - y;
            z = d->body_z - z;

            trigger_value = std::abs(x) + std::abs(y) + std::abs(z);

            if(trigger_value < 3.0f)
            {
                x = -x * 100.0f;
                y = -y * 100.0f;
                z = -z * 100.0f;
                trigger_value = 10.0f;
            }
            else
            //trigger_value = 10.0f;
            trigger = true;
        }
    }

    body_x = d->body_x;
    body_y = d->body_y;
    body_z = d->body_z;
}

} // namespace bnn_device_3d::conductors
