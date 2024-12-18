/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@yandex.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "teacher.h"

#include <chrono>
#include <stdexcept>

#include "config.hpp"

namespace bnn_device_3d::teachers
{

using namespace std::chrono_literals;

teacher::~teacher()
{
    if(bnn_state::started == state_)
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
    if(bnn_state::stopped != state_)
        throw std::runtime_error("busines logic error in teacher_base::start()");

    count = count_max;

    state_ = bnn_state::start;

    inner_start();

    while (bnn_state::started != state_)
        std::this_thread::sleep_for(device_3d_LITTLE_TIMEms);
}

void teacher::stop()
{
    if(bnn_state::started != state_)
        throw std::runtime_error("busines logic error in teacher_base::stop()");

    state_ = bnn_state::stop;

    while (bnn_state::stopped != state_)
        std::this_thread::sleep_for(device_3d_LITTLE_TIMEms);
}

} // namespace bnn_device_3d::teachers
