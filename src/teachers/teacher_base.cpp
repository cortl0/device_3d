#include "teacher_base.h"

teacher_base::~teacher_base()
{
    if(bnn::state::started == state_)
        stop();
}

teacher_base::teacher_base()
{

}

_word teacher_base::get_count()
{
    return count;
}

_word teacher_base::get_count_max()
{
    return count_max;
}

_word teacher_base::get_data()
{
    return data;
}

void teacher_base::start()
{
    if(bnn::state::stopped != state_)
        throw std::runtime_error("busines logic error in teacher_base::start()");

    count = count_max;

    state_ = bnn::state::start;

    inner_start();

    do
    {
        sleep(1);
    }
    while (bnn::state::started != state_);
}

void teacher_base::stop()
{
    if(bnn::state::started != state_)
        throw std::runtime_error("busines logic error in teacher_base::stop()");

    state_ = bnn::state::stop;

    do
    {
        sleep(1);
    }
    while (bnn::state::stopped != state_);
}
