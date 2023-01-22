/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "teacher.h"

#include <unistd.h>

#include "common/logger.h"
#include "config.hpp"

namespace bnn_device_3d::teachers
{

teacher::~teacher()
{
    if(bnn::state::started == state_)
        stop();
}

teacher::teacher()
{

}

u_word teacher::get_count()
{
    return count;
}

u_word teacher::get_count_max()
{
    return count_max;
}

u_word teacher::get_data()
{
    return data;
}

void teacher::start()
{
    if(bnn::state::stopped != state_)
        throw_error("busines logic error in teacher_base::start()");

    count = count_max;

    state_ = bnn::state::start;

    inner_start();

    while (bnn::state::started != state_)
        usleep(device_3d_LITTLE_TIME);
}

void teacher::stop()
{
    if(bnn::state::started != state_)
        throw_error("busines logic error in teacher_base::stop()");

    state_ = bnn::state::stop;

    while (bnn::state::stopped != state_)
        usleep(device_3d_LITTLE_TIME);
}

} // namespace bnn_device_3d::teachers
